#pragma once

#include <pthread.h>

#include "core/common.h"

// A few threads that split a loop between them, for work that is independent per item
// (LiDAR rays). The caller's thread takes a share too, and run_parallel returns when every
// item is done. With one thread it simply runs the loop.

#define WORKER_POOL_MAX_THREADS 32

// Called with a contiguous range [first, end) of the items.
typedef void Parallel_Function(void *context, u32 first, u32 end);

struct Worker_Pool;

struct Worker_Start {
    Worker_Pool *pool;
    u32 index;
};

struct Worker_Pool {
    pthread_t threads[WORKER_POOL_MAX_THREADS];
    Worker_Start starts[WORKER_POOL_MAX_THREADS];
    u32 thread_count; // helpers, not counting the caller
    pthread_mutex_t mutex;
    pthread_cond_t work_ready;
    pthread_cond_t work_done;
    u64 generation; // bumped for each run
    u32 busy; // helpers still working on this run
    bool quit;
    Parallel_Function *function;
    void *context;
    u32 item_count;
};

// threads counts the caller, so 1 starts no helper threads.
bool create_worker_pool(Worker_Pool *pool, u32 threads);
void destroy_worker_pool(Worker_Pool *pool);
void run_parallel(Worker_Pool *pool, Parallel_Function *function, void *context, u32 item_count);
