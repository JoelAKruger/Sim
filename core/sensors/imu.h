#pragma once

#include "core/config.h"
#include "core/sensors/mount.h"

// An IMU fixed to a robot link, sampled from the physics: angular velocity of its body and
// the specific force at its own point (the change in that point's velocity, minus
// gravity), both in the sensor frame, plus white noise and random-walk bias.
//
// It gives no orientation, like the Mid-360's. Samples fall on 1 / rate slots of sim time
// and carry the time of the step that produced them.

#define IMU_MAX_PENDING 8
#define STANDARD_GRAVITY 9.80665f // m/s² per g

struct Imu_Sample {
    u64 stamp_ns; // sim time
    v3 angular_velocity; // rad/s
    v3 linear_acceleration; // m/s², or g when the config says so
};

struct Imu_Sensor {
    const Imu_Config *config;
    Sensor_Mount mount;
    Random random;
    v3 gyro_bias;
    v3 accel_bias;
    v3 previous_velocity; // of the sensor's point, world frame
    bool has_previous;
    u64 next_sample; // index of the next sample slot
    Imu_Sample pending[IMU_MAX_PENDING]; // produced but not yet taken; the newest are kept
    u32 pending_count;
};

bool create_imu(Imu_Sensor *imu, const Imu_Config *config, const Robot *robot, u64 seed,
                char *error, u32 error_size);

// After a physics step. gravity is the world's, sim_time_ns the time after the step.
void update_imu(Imu_Sensor *imu, const Robot *robot, v3 gravity, f32 step_seconds, u64 sim_time_ns);

// Copies out the samples produced since the last call; returns how many.
u32 take_imu_samples(Imu_Sensor *imu, Imu_Sample *samples, u32 capacity);

// Forgets the last velocity, so a teleported robot doesn't read as a violent jolt.
void reset_imu(Imu_Sensor *imu);
