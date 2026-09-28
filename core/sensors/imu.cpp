#include "core/sensors/imu.h"

#include <math.h>
#include <string.h>

bool create_imu(Imu_Sensor *imu, const Imu_Config *config, const Robot *robot, u64 seed,
                char *error, u32 error_size)
{
    *imu = {};
    imu->config = config;
    imu->random = create_random(seed, 1);
    return find_sensor_mount(robot, config->frame, "imu.frame", &imu->mount, error, error_size);
}

void reset_imu(Imu_Sensor *imu) { imu->has_previous = false; }

static u64 get_slot_time_ns(u64 slot, f32 rate) { return (u64)((f64)slot * 1e9 / (f64)rate); }

static v3 get_gaussian_v3(Random *random, f32 sigma)
{
    v3 noise;
    noise.x = sigma * get_random_gaussian(random);
    noise.y = sigma * get_random_gaussian(random);
    noise.z = sigma * get_random_gaussian(random);
    return noise;
}

void update_imu(Imu_Sensor *imu, const Robot *robot, v3 gravity, f32 step_seconds, u64 sim_time_ns)
{
    const Imu_Config *config = imu->config;
    u32 body = robot->bodies[imu->mount.body].physics_body;
    Pose pose = get_sensor_pose(robot, &imu->mount);
    v3 velocity = get_point_velocity(robot->physics, body, pose.p);
    bool had_previous = imu->has_previous;
    v3 acceleration = (1.0f / step_seconds) * (velocity - imu->previous_velocity);
    imu->previous_velocity = velocity;
    imu->has_previous = true;
    if (!had_previous) {
        return; // no acceleration to report yet; the slot waits for the next step
    }
    if (sim_time_ns < get_slot_time_ns(imu->next_sample, config->rate)) {
        return;
    }
    while (sim_time_ns >= get_slot_time_ns(imu->next_sample, config->rate)) {
        imu->next_sample++; // slots missed before the first step are skipped, not replayed
    }

    // Discrete noise for one sample period, from the continuous densities.
    f32 period = 1.0f / config->rate;
    imu->gyro_bias =
        imu->gyro_bias + get_gaussian_v3(&imu->random, config->gyro_bias_walk * sqrtf(period));
    imu->accel_bias =
        imu->accel_bias + get_gaussian_v3(&imu->random, config->accel_bias_walk * sqrtf(period));
    v3 gyro_noise = get_gaussian_v3(&imu->random, config->gyro_noise * sqrtf(config->rate));
    v3 accel_noise = get_gaussian_v3(&imu->random, config->accel_noise * sqrtf(config->rate));

    Imu_Sample sample;
    sample.stamp_ns = sim_time_ns;
    sample.angular_velocity =
        inverse_rotate_vector(pose.q, get_body_angular_velocity(robot->physics, body)) +
        imu->gyro_bias + gyro_noise;
    sample.linear_acceleration =
        inverse_rotate_vector(pose.q, acceleration - gravity) + imu->accel_bias + accel_noise;
    if (config->acceleration_in_g) {
        sample.linear_acceleration = (1.0f / STANDARD_GRAVITY) * sample.linear_acceleration;
    }
    if (imu->pending_count == IMU_MAX_PENDING) {
        // Nobody is taking them: keep the newest.
        memmove(imu->pending, imu->pending + 1, (IMU_MAX_PENDING - 1) * sizeof(Imu_Sample));
        imu->pending_count--;
    }
    imu->pending[imu->pending_count++] = sample;
}

u32 take_imu_samples(Imu_Sensor *imu, Imu_Sample *samples, u32 capacity)
{
    u32 count = min(imu->pending_count, capacity);
    for (u32 i = 0; i < count; i++) {
        samples[i] = imu->pending[i];
    }
    imu->pending_count = 0;
    return count;
}
