#include "core/worker_pool.h"

// Worker share: participant `index` of `participants` takes an even slice.
static void run_share(Worker_Pool *pool, u32 index, u32 participants)
{
    u64 first = (u64)pool->item_count * index / participants;
    u64 end = (u64)pool->item_count * (index + 1) / participants;
    if (first < end) {
        pool->function(pool->context, (u32)first, (u32)end);
    }
}

static void *run_worker(void *argument)
{
    Worker_Start *start = (Worker_Start *)argument;
    Worker_Pool *pool = start->pool;
    u64 seen = 0;
    for (;;) {
        pthread_mutex_lock(&pool->mutex);
        while (pool->generation == seen && !pool->quit) {
            pthread_cond_wait(&pool->work_ready, &pool->mutex);
        }
        if (pool->quit) {
            pthread_mutex_unlock(&pool->mutex);
            return NULL;
        }
        seen = pool->generation;
        pthread_mutex_unlock(&pool->mutex);

        run_share(pool, start->index + 1, pool->thread_count + 1);

        pthread_mutex_lock(&pool->mutex);
        if (--pool->busy == 0) {
            pthread_cond_signal(&pool->work_done);
        }
        pthread_mutex_unlock(&pool->mutex);
    }
}

bool create_worker_pool(Worker_Pool *pool, u32 threads)
{
    *pool = {};
    pthread_mutex_init(&pool->mutex, NULL);
    pthread_cond_init(&pool->work_ready, NULL);
    pthread_cond_init(&pool->work_done, NULL);
    u32 helpers = min(max(threads, 1u), (u32)WORKER_POOL_MAX_THREADS) - 1;
    for (u32 i = 0; i < helpers; i++) {
        pool->starts[i] = Worker_Start{pool, i};
        if (pthread_create(&pool->threads[i], NULL, run_worker, &pool->starts[i]) != 0) {
            log_warning("worker pool: could only start %u of %u threads", i, helpers);
            break;
        }
        pool->thread_count++;
    }
    return true;
}

void destroy_worker_pool(Worker_Pool *pool)
{
    pthread_mutex_lock(&pool->mutex);
    pool->quit = true;
    pthread_cond_broadcast(&pool->work_ready);
    pthread_mutex_unlock(&pool->mutex);
    for (u32 i = 0; i < pool->thread_count; i++) {
        pthread_join(pool->threads[i], NULL);
    }
    pthread_mutex_destroy(&pool->mutex);
    pthread_cond_destroy(&pool->work_ready);
    pthread_cond_destroy(&pool->work_done);
    pool->thread_count = 0;
}

void run_parallel(Worker_Pool *pool, Parallel_Function *function, void *context, u32 item_count)
{
    if (pool->thread_count == 0) {
        function(context, 0, item_count);
        return;
    }
    pthread_mutex_lock(&pool->mutex);
    pool->function = function;
    pool->context = context;
    pool->item_count = item_count;
    pool->busy = pool->thread_count;
    pool->generation++;
    pthread_cond_broadcast(&pool->work_ready);
    pthread_mutex_unlock(&pool->mutex);

    run_share(pool, 0, pool->thread_count + 1);

    pthread_mutex_lock(&pool->mutex);
    while (pool->busy > 0) {
        pthread_cond_wait(&pool->work_done, &pool->mutex);
    }
    pthread_mutex_unlock(&pool->mutex);
}
