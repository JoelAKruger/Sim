#pragma once

#include "core/config.h"
#include "core/sensors/mount.h"
#include "core/worker_pool.h"

// A Livox Mid-360 fixed to a robot link, made of Box3D ray casts.
//
// Like the real unit it scans continuously: every step casts the rays whose scheduled
// time fell within it, from the sensor's pose after the step (split across threads, which
// is safe because nothing changes the Box3D world between steps), and a frame is complete every
// 1 / rate seconds. The Mid-360's rosette is proprietary, so the directions follow an R2
// low-discrepancy sequence over azimuth and sin(elevation) instead: it never repeats, one
// frame covers the whole field of view evenly, and coverage keeps filling in over time.
// Rays are dealt to its four lasers in turn (the line field). Misses and hits inside the
// blind zone give no point, as the driver drops them.

#define LIDAR_LINES 4
#define LIDAR_BATCH 4096 // rays cast together

// One return, in the sensor frame (x forward, z up).
struct Lidar_Point {
    f32 x;
    f32 y;
    f32 z;
    f32 intensity; // reflectivity, 0..255
    u8 tag;
    u8 line;
    u64 time_ns; // sim time the ray was scheduled
};

// One ray's cast, before noise.
struct Lidar_Ray {
    v3 direction; // sensor frame
    f32 range; // m, or -1 for no return
    f32 incidence; // cosine between the ray and the surface normal
    u32 line;
};

struct Lidar_Frame {
    u64 start_ns; // sim time of the frame's first ray slot
    u32 count;
    Lidar_Point *points;
};

struct Lidar_Sensor {
    const Lidar_Config *config;
    Sensor_Mount mount;
    Random random;
    u32 capacity; // points a frame can hold
    u64 next_ray; // index of the next ray to cast, counted from time 0
    u64 frame_number;
    Lidar_Frame frames[2];
    u32 building; // index into frames
    bool frame_ready; // frames[building ^ 1] is complete and not yet taken
    u64 rays_cast; // in total, for cost measurements
    Lidar_Ray *rays; // one batch
    Worker_Pool pool; // casts a batch across lidar.threads threads
};

// Points a frame can hold at these settings.
u32 get_lidar_capacity(const Lidar_Config *config);

// A frame as it crosses to the ROS thread: this header, then count points.
struct Lidar_Frame_Header {
    u64 start_ns;
    u32 count;
};

inline u64 get_lidar_slot_size(u32 capacity)
{
    return sizeof(Lidar_Frame_Header) + (u64)capacity * sizeof(Lidar_Point);
}
inline Lidar_Point *get_lidar_slot_points(Lidar_Frame_Header *header)
{
    return (Lidar_Point *)(header + 1);
}

bool create_lidar(Lidar_Sensor *lidar, const Lidar_Config *config, const Robot *robot,
                  Linear_Allocator *allocator, u64 seed, char *error, u32 error_size);

void destroy_lidar(Lidar_Sensor *lidar);

// After a physics step; sim_time_ns is the time after the step.
void update_lidar(Lidar_Sensor *lidar, const Robot *robot, b3WorldId world, u64 sim_time_ns);

// The latest complete frame, or NULL if none has completed since the last call.
const Lidar_Frame *take_lidar_frame(Lidar_Sensor *lidar);

// The last complete frame, taken or not (for drawing). NULL before the first.
const Lidar_Frame *get_last_lidar_frame(const Lidar_Sensor *lidar);

// The unit direction of ray k in the sensor frame, and its line.
v3 get_lidar_direction(const Lidar_Config *config, u64 ray, u32 *line);
