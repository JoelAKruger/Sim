#include <pthread.h>
#include <string.h>

#include "core/shared_state.h"
#include "tests/check.h"

static void test_ring_single_thread(Linear_Allocator *allocator)
{
    Ring_Buffer ring;
    CHECK(create_ring_buffer(&ring, allocator, sizeof(u32), 4));
    CHECK(!create_ring_buffer(&ring, allocator, sizeof(u32), 6)); // not a power of two
    CHECK(create_ring_buffer(&ring, allocator, sizeof(u32), 4));

    u32 value = 0;
    CHECK(!pop_ring_buffer(&ring, &value));
    for (u32 i = 0; i < 4; i++) {
        CHECK(push_ring_buffer(&ring, &i));
    }
    u32 extra = 99;
    CHECK(!push_ring_buffer(&ring, &extra)); // full
    CHECK(get_ring_buffer_count(&ring) == 4);
    for (u32 i = 0; i < 4; i++) {
        CHECK(pop_ring_buffer(&ring, &value));
        CHECK(value == i);
    }
    CHECK(!pop_ring_buffer(&ring, &value));

    // Indices wrap at 2^32; the arithmetic must not care.
    ring.head = ring.tail = 0xfffffffeu;
    for (u32 i = 0; i < 4; i++) {
        CHECK(push_ring_buffer(&ring, &i));
    }
    CHECK(!push_ring_buffer(&ring, &extra));
    for (u32 i = 0; i < 4; i++) {
        CHECK(pop_ring_buffer(&ring, &value));
        CHECK(value == i);
    }
    CHECK(get_ring_buffer_count(&ring) == 0);
}

#define RING_ITEMS 2000000ull

static void *produce_ring_items(void *context)
{
    Ring_Buffer *ring = (Ring_Buffer *)context;
    for (u64 i = 0; i < RING_ITEMS; i++) {
        while (!push_ring_buffer(ring, &i)) {
        }
    }
    return NULL;
}

// Every item arrives exactly once, in order, while both threads run flat out.
static void test_ring_threaded(Linear_Allocator *allocator)
{
    Ring_Buffer ring;
    CHECK(create_ring_buffer(&ring, allocator, sizeof(u64), 64));
    pthread_t producer;
    pthread_create(&producer, NULL, produce_ring_items, &ring);
    u64 expected = 0;
    u64 errors = 0;
    while (expected < RING_ITEMS) {
        u64 value;
        if (pop_ring_buffer(&ring, &value)) {
            errors += value != expected;
            expected++;
        }
    }
    pthread_join(producer, NULL);
    CHECK(errors == 0);
    CHECK(get_ring_buffer_count(&ring) == 0);
}

#define FRAME_WORDS 4096
#define FRAMES 20000ull

static void *produce_triple_frames(void *context)
{
    Triple_Buffer *buffer = (Triple_Buffer *)context;
    for (u64 frame = 1; frame <= FRAMES; frame++) {
        u64 *words = (u64 *)get_triple_buffer_write_slot(buffer);
        for (u32 i = 0; i < FRAME_WORDS; i++) {
            words[i] = frame;
        }
        publish_triple_buffer(buffer);
    }
    return NULL;
}

// The consumer never sees a torn frame (every word from the same write), and frame
// numbers only ever increase.
static void test_triple_threaded(Linear_Allocator *allocator)
{
    Triple_Buffer buffer;
    CHECK(create_triple_buffer(&buffer, allocator, FRAME_WORDS * sizeof(u64)));
    CHECK(read_triple_buffer(&buffer) == NULL); // nothing published yet

    pthread_t producer;
    pthread_create(&producer, NULL, produce_triple_frames, &buffer);
    u64 last_frame = 0;
    u64 torn = 0;
    u64 backwards = 0;
    u64 frames_seen = 0;
    while (last_frame < FRAMES) {
        const u64 *words = (const u64 *)read_triple_buffer(&buffer);
        if (!words) {
            continue;
        }
        u64 frame = words[0];
        for (u32 i = 1; i < FRAME_WORDS; i++) {
            torn += words[i] != frame;
        }
        backwards += frame <= last_frame;
        last_frame = frame;
        frames_seen++;
    }
    pthread_join(producer, NULL);
    CHECK(torn == 0);
    CHECK(backwards == 0);
    CHECK(frames_seen > 0);
    CHECK(read_triple_buffer(&buffer) == NULL); // the last frame was already consumed
}

static void test_shared_scalars(void)
{
    Shared_Global_State shared = {};
    CHECK(!is_quit_requested(&shared));
    request_quit(&shared);
    CHECK(is_quit_requested(&shared));
    set_sim_time(&shared, 123456789ull);
    CHECK(get_sim_time(&shared) == 123456789ull);
    CHECK(get_stamp_offset(&shared) == 0); // sim time until set
    set_stamp_offset(&shared, 1790000000ull * NS_PER_S);
    CHECK(get_stamp_offset(&shared) == 1790000000ull * NS_PER_S);
}

int main(void)
{
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 16 * MEGABYTE));
    test_ring_single_thread(&allocator);
    test_ring_threaded(&allocator);
    test_triple_threaded(&allocator);
    test_shared_scalars();
    destroy_allocator(&allocator);
    return report_checks("shared_state");
}
