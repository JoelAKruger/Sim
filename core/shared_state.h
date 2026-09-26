#pragma once

#include "core/actuator.h"
#include "core/allocator.h"
#include "core/config.h"
#include "core/sensors/imu.h"
#include "core/status_light.h"

// Everything that crosses a thread boundary goes through this file. There are no locks:
// each structure has exactly one producer thread and one consumer thread.

// Queue of fixed-size items, for streams where every item matters and order matters
// (CAN commands). Indices run freely and wrap at 2^32.
struct Ring_Buffer {
    u8 *items;
    u32 item_size;
    u32 mask; // capacity - 1; capacity is a power of two
    alignas(64) u32 head; // next slot to write; stored only by the producer
    alignas(64) u32 tail; // next slot to read; stored only by the consumer
};

bool create_ring_buffer(Ring_Buffer *ring, Linear_Allocator *allocator, u32 item_size,
                        u32 capacity);
bool push_ring_buffer(Ring_Buffer *ring, const void *item); // false when full
bool pop_ring_buffer(Ring_Buffer *ring, void *item); // false when empty
u32 get_ring_buffer_count(Ring_Buffer *ring);

// Latest-value mailbox for large frames (images, point clouds). The producer never
// waits, and the consumer always receives the newest complete frame, never a torn one.
struct Triple_Buffer {
    u8 *slots;
    u64 slot_size;
    u32 write_slot; // producer-owned
    u32 read_slot; // consumer-owned
    alignas(64) u32 middle; // shared: slot index, plus TRIPLE_FRESH when unread
};

bool create_triple_buffer(Triple_Buffer *buffer, Linear_Allocator *allocator, u64 slot_size);
void *get_triple_buffer_write_slot(Triple_Buffer *buffer); // producer: the slot to fill next
void publish_triple_buffer(Triple_Buffer *buffer); // producer: hand the filled slot over
void *read_triple_buffer(Triple_Buffer *buffer); // consumer: newest unread frame, or NULL

// Shared state between the sim, ROS and CAN threads. Scalars are accessed only through
// the functions below, which use atomics; each ring has one producer and one consumer.
struct Shared_Global_State {
    u64 sim_time_ns;
    u32 quit;
    Ring_Buffer actuator_commands; // Actuator_Command: CAN thread to sim
    Triple_Buffer status_light; // Status_Light: CAN thread to sim, latest wins
    Ring_Buffer imu_samples; // Imu_Sample: sim to ROS
    Triple_Buffer lidar_frames; // Lidar_Frame_Header and its points: sim to ROS
    Triple_Buffer camera_frames; // Camera_Frame_Header and its images: renderer to ROS
};

// Sensor buffers are sized for the config's sensors (and tiny for disabled ones).
bool create_shared_state(Shared_Global_State *shared, Linear_Allocator *allocator,
                         const Sim_Config *config);

void set_sim_time(Shared_Global_State *shared, u64 sim_time_ns);
u64 get_sim_time(Shared_Global_State *shared);
void request_quit(Shared_Global_State *shared);
bool is_quit_requested(Shared_Global_State *shared);
