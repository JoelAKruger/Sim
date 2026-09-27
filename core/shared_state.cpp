#include "core/shared_state.h"

#include "core/sensors/camera_model.h"
#include "core/sensors/lidar.h"

#include <string.h>

#define TRIPLE_FRESH 0x4u
#define TRIPLE_INDEX 0x3u

bool create_ring_buffer(Ring_Buffer *ring, Linear_Allocator *allocator, u32 item_size, u32 capacity)
{
    if (capacity < 2 || (capacity & (capacity - 1)) != 0) {
        log_error("ring: capacity %u is not a power of two", capacity);
        return false;
    }
    *ring = {};
    ring->items = (u8 *)allocate_memory(allocator, (u64)item_size * capacity, 64);
    ring->item_size = item_size;
    ring->mask = capacity - 1;
    return ring->items != NULL;
}

bool push_ring_buffer(Ring_Buffer *ring, const void *item)
{
    u32 head = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
    u32 tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    if (head - tail > ring->mask) {
        return false;
    }
    memcpy(ring->items + (u64)(head & ring->mask) * ring->item_size, item, ring->item_size);
    __atomic_store_n(&ring->head, head + 1, __ATOMIC_RELEASE);
    return true;
}

bool pop_ring_buffer(Ring_Buffer *ring, void *item)
{
    u32 tail = __atomic_load_n(&ring->tail, __ATOMIC_RELAXED);
    u32 head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
    if (head == tail) {
        return false;
    }
    memcpy(item, ring->items + (u64)(tail & ring->mask) * ring->item_size, ring->item_size);
    __atomic_store_n(&ring->tail, tail + 1, __ATOMIC_RELEASE);
    return true;
}

u32 get_ring_buffer_count(Ring_Buffer *ring)
{
    u32 head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
    u32 tail = __atomic_load_n(&ring->tail, __ATOMIC_ACQUIRE);
    return head - tail;
}

// Three slots rotate between the roles write, middle and read. Publishing swaps the
// write slot into the middle; reading swaps the middle into the read slot. Each swap is
// one atomic exchange, so neither side ever waits for the other.
bool create_triple_buffer(Triple_Buffer *buffer, Linear_Allocator *allocator, u64 slot_size)
{
    *buffer = {};
    buffer->slot_size = (slot_size + 63) & ~63ull;
    buffer->slots = (u8 *)allocate_memory(allocator, buffer->slot_size * 3, 64);
    buffer->write_slot = 0;
    buffer->middle = 1;
    buffer->read_slot = 2;
    return buffer->slots != NULL;
}

void *get_triple_buffer_write_slot(Triple_Buffer *buffer)
{
    return buffer->slots + buffer->write_slot * buffer->slot_size;
}

void publish_triple_buffer(Triple_Buffer *buffer)
{
    u32 previous =
        __atomic_exchange_n(&buffer->middle, buffer->write_slot | TRIPLE_FRESH, __ATOMIC_ACQ_REL);
    buffer->write_slot = previous & TRIPLE_INDEX;
}

void *read_triple_buffer(Triple_Buffer *buffer)
{
    if (!(__atomic_load_n(&buffer->middle, __ATOMIC_ACQUIRE) & TRIPLE_FRESH)) {
        return NULL;
    }
    u32 previous = __atomic_exchange_n(&buffer->middle, buffer->read_slot, __ATOMIC_ACQ_REL);
    buffer->read_slot = previous & TRIPLE_INDEX;
    return buffer->slots + buffer->read_slot * buffer->slot_size;
}

void set_sim_time(Shared_Global_State *shared, u64 sim_time_ns)
{
    __atomic_store_n(&shared->sim_time_ns, sim_time_ns, __ATOMIC_RELEASE);
}

u64 get_sim_time(Shared_Global_State *shared)
{
    return __atomic_load_n(&shared->sim_time_ns, __ATOMIC_ACQUIRE);
}

void set_stamp_offset(Shared_Global_State *shared, u64 offset_ns)
{
    __atomic_store_n(&shared->stamp_offset_ns, offset_ns, __ATOMIC_RELEASE);
}

u64 get_stamp_offset(Shared_Global_State *shared)
{
    return __atomic_load_n(&shared->stamp_offset_ns, __ATOMIC_ACQUIRE);
}

void request_quit(Shared_Global_State *shared)
{
    __atomic_store_n(&shared->quit, 1u, __ATOMIC_RELEASE);
}

bool is_quit_requested(Shared_Global_State *shared)
{
    return __atomic_load_n(&shared->quit, __ATOMIC_ACQUIRE) != 0;
}

bool create_shared_state(Shared_Global_State *shared, Linear_Allocator *allocator,
                         const Sim_Config *config)
{
    u64 lidar_size =
        get_lidar_slot_size(config->lidar.enabled ? get_lidar_capacity(&config->lidar) : 0);
    const Camera_Stream_Config *color = &config->camera.color;
    const Camera_Stream_Config *depth = &config->camera.depth;
    u64 color_size = color->enabled
                         ? get_color_frame_size(color->resolution[0], color->resolution[1])
                         : sizeof(Camera_Frame_Header);
    u64 depth_size = depth->enabled
                         ? get_depth_frame_size(depth->resolution[0], depth->resolution[1])
                         : sizeof(Camera_Frame_Header);
    return create_ring_buffer(&shared->actuator_commands, allocator, sizeof(Actuator_Command),
                              1024) &&
           create_triple_buffer(&shared->status_light, allocator, sizeof(Status_Light)) &&
           create_ring_buffer(&shared->imu_samples, allocator, sizeof(Imu_Sample), 1024) &&
           create_triple_buffer(&shared->lidar_frames, allocator, lidar_size) &&
           create_triple_buffer(&shared->color_frames, allocator, color_size) &&
           create_triple_buffer(&shared->depth_frames, allocator, depth_size);
}
