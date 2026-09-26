#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "core/math.h"
#include "core/sensors/camera_model.h"
#include "core/world.h"
#include "tests/check.h"

static const char *banksia_path;

// A 1 kg box with two sensor links: imu_link rolled 90° about x, lidar_link above the box.
static const char *sensor_robot =
    "<robot name='sensor_box'>"
    "  <link name='base'><inertial><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' "
    "ixy='0' ixz='0' iyz='0'/></inertial>"
    "    <collision><geometry><box size='0.4 0.4 0.4'/></geometry></collision></link>"
    "  <link name='imu_link'/>"
    "  <link name='lidar_link'/>"
    "  <joint name='imu_joint' type='fixed'><parent link='base'/><child link='imu_link'/>"
    "    <origin xyz='0 0 0' rpy='1.5707963 0 0'/></joint>"
    "  <joint name='lidar_joint' type='fixed'><parent link='base'/><child link='lidar_link'/>"
    "    <origin xyz='0 0 0.3'/></joint>"
    "</robot>";

static Sim_Config make_sensor_config(void)
{
    Sim_Config config = get_default_config();
    config.terrain_size[0] = config.terrain_size[1] = 100.0f;
    config.terrain_relief = 0.0f;
    config.crater_count = 0;
    config.boulder_count = 0; // bare ground, so every LiDAR return is on it
    config.imu.enabled = true;
    config.lidar.enabled = true;
    snprintf(config.imu.frame, sizeof(config.imu.frame), "base");
    snprintf(config.lidar.frame, sizeof(config.lidar.frame), "lidar_link");
    config.imu.gyro_noise = config.imu.accel_noise = 0.0f;
    config.imu.gyro_bias_walk = config.imu.accel_bias_walk = 0.0f;
    return config;
}

static bool create_sensor_world(World *world, Linear_Allocator *allocator, const Sim_Config *config,
                                const char *xml)
{
    CHECK(create_allocator(allocator, 64 * MEGABYTE));
    CHECK(create_world(world, allocator, config));
    char error[256];
    bool loaded = load_robot(world, xml, strlen(xml), "", error, sizeof(error));
    CHECK(loaded);
    if (!loaded) {
        fprintf(stderr, "  %s\n", error);
    }
    return loaded;
}

static void finish_world(World *world, Linear_Allocator *allocator)
{
    destroy_world(world);
    destroy_allocator(allocator);
}

static void run_steps(World *world, f32 seconds)
{
    u32 count = (u32)(seconds * (f32)world->config.physics_hz);
    for (u32 i = 0; i < count; i++) {
        step_world(world);
    }
}

// Steps for a while and averages the IMU's samples; returns how many there were.
static u32 average_imu(World *world, f32 seconds, v3 *rate, v3 *acceleration, bool *ordered)
{
    *rate = *acceleration = v3{0.0f, 0.0f, 0.0f};
    *ordered = true;
    take_imu_samples(&world->imu, NULL, 0); // only samples from this window
    u64 last_stamp = 0;
    u32 total = 0;
    u32 count = (u32)(seconds * (f32)world->config.physics_hz);
    for (u32 i = 0; i < count; i++) {
        step_world(world);
        Imu_Sample samples[IMU_MAX_PENDING];
        u32 taken = take_imu_samples(&world->imu, samples, IMU_MAX_PENDING);
        for (u32 s = 0; s < taken; s++) {
            *ordered = *ordered && samples[s].stamp_ns > last_stamp;
            last_stamp = samples[s].stamp_ns;
            *rate = *rate + samples[s].angular_velocity;
            *acceleration = *acceleration + samples[s].linear_acceleration;
            total++;
        }
    }
    if (total) {
        *rate = (1.0f / (f32)total) * *rate;
        *acceleration = (1.0f / (f32)total) * *acceleration;
    }
    return total;
}

// At rest the IMU feels 1 g upwards, in its own axes, and no rotation.
static void test_imu_at_rest(void)
{
    const char *frames[] = {"base", "imu_link"};
    v3 expected[] = {{0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}}; // imu_link's y points up
    for (u32 f = 0; f < 2; f++) {
        Sim_Config config = make_sensor_config();
        snprintf(config.imu.frame, sizeof(config.imu.frame), "%s", frames[f]);
        Linear_Allocator allocator;
        static World world;
        if (create_sensor_world(&world, &allocator, &config, sensor_robot)) {
            run_steps(&world, 1.0f);
            v3 rate, acceleration;
            bool ordered;
            u32 count = average_imu(&world, 1.0f, &rate, &acceleration, &ordered);
            printf("  imu on %s at rest: %u samples/s, (%.4f, %.4f, %.4f) g\n", frames[f], count,
                   (f64)acceleration.x, (f64)acceleration.y, (f64)acceleration.z);
            CHECK(count >= 199 && count <= 201);
            CHECK(ordered);
            CHECK(b3Length(acceleration - expected[f]) < 0.01f);
            CHECK(b3Length(rate) < 1e-3f);
        }
        finish_world(&world, &allocator);
    }

    // In m/s² when asked.
    Sim_Config config = make_sensor_config();
    config.imu.acceleration_in_g = false;
    Linear_Allocator allocator;
    static World world;
    if (create_sensor_world(&world, &allocator, &config, sensor_robot)) {
        run_steps(&world, 1.0f);
        v3 rate, acceleration;
        bool ordered;
        average_imu(&world, 0.5f, &rate, &acceleration, &ordered);
        CHECK_NEAR(acceleration.z, 9.81, 0.05);
    }
    finish_world(&world, &allocator);
}

// Spinning in free fall, the gyro reads the spin in the sensor's axes, and the accelerometer
// reads nothing (it is falling with everything else).
static void test_imu_spin(void)
{
    const char *frames[] = {"base", "imu_link"};
    v3 expected[] = {{0.0f, 0.0f, 0.5f}, {0.0f, 0.5f, 0.0f}};
    for (u32 f = 0; f < 2; f++) {
        Sim_Config config = make_sensor_config();
        snprintf(config.imu.frame, sizeof(config.imu.frame), "%s", frames[f]);
        Linear_Allocator allocator;
        static World world;
        if (create_sensor_world(&world, &allocator, &config, sensor_robot)) {
            b3BodyId body = world.robot.bodies[0].id;
            // Lifted well clear of the ground, so it falls freely.
            b3Body_SetTransform(body, v3{20.0f, 0.0f, 50.0f}, b3Body_GetRotation(body));
            b3Body_SetAngularVelocity(body, v3{0.0f, 0.0f, 0.5f});
            reset_imu(&world.imu);
            v3 rate, acceleration;
            bool ordered;
            average_imu(&world, 0.5f, &rate, &acceleration, &ordered);
            printf("  imu on %s spinning: (%.4f, %.4f, %.4f) rad/s, |a| %.4f g\n", frames[f],
                   (f64)rate.x, (f64)rate.y, (f64)rate.z, (f64)b3Length(acceleration));
            CHECK(b3Length(rate - expected[f]) < 1e-3f);
            CHECK(b3Length(acceleration) < 1e-4f);
        }
        finish_world(&world, &allocator);
    }
}

// With noise on, the same seed gives the same readings and a different seed doesn't.
static u64 hash_imu_run(u32 seed)
{
    Sim_Config config = make_sensor_config();
    config.sensor_seed = seed;
    config.imu.gyro_noise = 6.6e-5f;
    config.imu.accel_noise = 9.8e-4f;
    Linear_Allocator allocator;
    static World world;
    u64 hash = HASH_SEED;
    if (create_sensor_world(&world, &allocator, &config, sensor_robot)) {
        for (u32 i = 0; i < 240; i++) {
            step_world(&world);
            Imu_Sample samples[IMU_MAX_PENDING];
            u32 taken = take_imu_samples(&world.imu, samples, IMU_MAX_PENDING);
            hash = hash_bytes(hash, samples, taken * sizeof(Imu_Sample));
        }
    }
    finish_world(&world, &allocator);
    return hash;
}

// Flat ground seen from 0.5 m up: every return lies on it, inside the field of view.
static void test_lidar_ground(void)
{
    Sim_Config config = make_sensor_config();
    config.lidar.range_noise = 0.0f;
    config.lidar.threads = 3;
    Linear_Allocator allocator;
    static World world;
    if (create_sensor_world(&world, &allocator, &config, sensor_robot)) {
        run_steps(&world, 1.0f);
        CHECK(world.lidar.frame_number == 10);
        CHECK(take_lidar_frame(&world.lidar) != NULL);
        u64 rays_before = world.lidar.rays_cast;
        const Lidar_Frame *frame = NULL;
        u64 start_ns = get_time_ns();
        u32 steps = 0;
        while (!frame) {
            step_world(&world);
            steps++;
            frame = take_lidar_frame(&world.lidar);
        }
        f64 step_ms = (f64)(get_time_ns() - start_ns) * 1e-6 / steps;
        u64 rays = world.lidar.rays_cast - rays_before;
        printf("  lidar: %llu rays over %u steps (%.3f ms a step with the rays), %u returns\n",
               (unsigned long long)rays, steps, step_ms, frame->count);
        CHECK(rays >= 19900 && rays <= 20100);
        CHECK(frame->count > 2000 && frame->count < 4000); // the ground band is about 13%

        b3Transform pose = get_sensor_pose(&world.robot, &world.lidar.mount);
        f32 worst = 0.0f;
        bool in_fov = true, in_range = true, in_time = true;
        u32 lines_seen = 0;
        for (u32 i = 0; i < frame->count; i++) {
            const Lidar_Point *point = &frame->points[i];
            v3 local = {point->x, point->y, point->z};
            v3 world_point = b3TransformPoint(pose, local);
            f32 ground = get_terrain_height(&world.terrain, world_point.x, world_point.y);
            worst = max(worst, absolute(world_point.z - ground));
            f32 range = b3Length(local);
            f32 elevation = asinf(point->z / range) * 180.0f / PI_F32;
            in_fov = in_fov && elevation >= -7.01f && elevation <= 52.01f;
            in_range = in_range && range >= 0.1f && range <= 40.0f;
            in_time = in_time && point->time_ns >= frame->start_ns &&
                      point->time_ns < frame->start_ns + 100000000ull;
            lines_seen |= 1u << point->line;
            CHECK(point->intensity >= 0.0f && point->intensity <= 60.0f);
        }
        printf("  lidar: worst distance from the ground %.4f m\n", (f64)worst);
        CHECK(worst < 0.03f);
        CHECK(in_fov && in_range && in_time);
        CHECK(lines_seen == 0xf);
    }
    finish_world(&world, &allocator);
}

static u64 hash_lidar_frame(u32 seed, u32 threads)
{
    Sim_Config config = make_sensor_config();
    config.sensor_seed = seed;
    config.lidar.threads = threads;
    Linear_Allocator allocator;
    static World world;
    u64 hash = HASH_SEED;
    if (create_sensor_world(&world, &allocator, &config, sensor_robot)) {
        run_steps(&world, 0.5f);
        const Lidar_Frame *frame = get_last_lidar_frame(&world.lidar);
        CHECK(frame != NULL);
        if (frame) {
            hash = hash_bytes(hash, frame->points, frame->count * sizeof(Lidar_Point));
        }
    }
    finish_world(&world, &allocator);
    return hash;
}

// A sensor on a link that doesn't exist, or that has no body, is refused by name.
static void test_bad_frames(void)
{
    struct Bad_Frame {
        const char *frame;
        const char *expected;
    };
    Bad_Frame bad[] = {
        {"mast", "lidar.frame: the robot has no link named \"mast\""},
        {"tl_ball_link_x", "lidar.frame: \"tl_ball_link_x\" is a dummy link"},
    };
    u64 size = 0;
    char *xml = read_file(banksia_path, &size);
    CHECK(xml != NULL);
    for (u32 i = 0; i < ARRAY_COUNT(bad) && xml; i++) {
        Sim_Config config = make_sensor_config();
        config.imu.enabled = false;
        snprintf(config.lidar.frame, sizeof(config.lidar.frame), "%s", bad[i].frame);
        Linear_Allocator allocator;
        static World world;
        CHECK(create_allocator(&allocator, 64 * MEGABYTE));
        CHECK(create_world(&world, &allocator, &config));
        char error[256];
        CHECK(!load_robot(&world, xml, size, "", error, sizeof(error)));
        if (strstr(error, bad[i].expected) == NULL) {
            fprintf(stderr, "  got \"%s\", expected \"%s\"\n", error, bad[i].expected);
            check_failures++;
        }
        finish_world(&world, &allocator);
    }
    free(xml);
}

// Banksia with both sensors on its gps link (on the chassis) reads 1 g once settled.
static void test_banksia(void)
{
    Sim_Config config = make_sensor_config();
    snprintf(config.imu.frame, sizeof(config.imu.frame), "gps");
    snprintf(config.lidar.frame, sizeof(config.lidar.frame), "gps");
    u64 size = 0;
    char *xml = read_file(banksia_path, &size);
    CHECK(xml != NULL);
    Linear_Allocator allocator;
    static World world;
    if (xml && create_sensor_world(&world, &allocator, &config, xml)) {
        run_steps(&world, 1.5f);
        v3 rate, acceleration;
        bool ordered;
        average_imu(&world, 0.5f, &rate, &acceleration, &ordered);
        printf("  banksia imu: (%.4f, %.4f, %.4f) g\n", (f64)acceleration.x, (f64)acceleration.y,
               (f64)acceleration.z);
        CHECK(b3Length(acceleration - v3{0.0f, 0.0f, 1.0f}) < 0.02f);
        const Lidar_Frame *frame = get_last_lidar_frame(&world.lidar);
        CHECK(frame && frame->count > 1000);
    }
    finish_world(&world, &allocator);
    free(xml);
}

static void test_camera_model(void)
{
    // The optical frame looks along the body's x, with its x to the body's right (-y) and its
    // y down (-z), as realsense2_camera's optical frames do.
    b3Quat optical = get_optical_rotation();
    v3 look = b3RotateVector(optical, v3{0.0f, 0.0f, 1.0f});
    v3 right = b3RotateVector(optical, v3{1.0f, 0.0f, 0.0f});
    v3 down = b3RotateVector(optical, v3{0.0f, 1.0f, 0.0f});
    CHECK(b3Length(look - v3{1.0f, 0.0f, 0.0f}) < 1e-6f);
    CHECK(b3Length(right - v3{0.0f, -1.0f, 0.0f}) < 1e-6f);
    CHECK(b3Length(down - v3{0.0f, 0.0f, -1.0f}) < 1e-6f);
    char name[64];
    make_camera_frame_name("d415", "_color_optical_frame", name, sizeof(name));
    CHECK(strcmp(name, "d415_color_optical_frame") == 0);

    Camera_Intrinsics camera = make_camera_intrinsics(640, 480, 69.0f);
    CHECK_NEAR(camera.fx, 465.6, 0.1);
    CHECK(camera.fx == camera.fy);
    CHECK_NEAR(camera.cx, 319.5, 1e-6);
    CHECK_NEAR(camera.cy, 239.5, 1e-6);
    CHECK_NEAR(get_vertical_fov_deg(&camera), 54.5, 0.1);

    CHECK(encode_depth_mm(1.5f, 0.1f, 10.0f) == 1500);
    CHECK(encode_depth_mm(0.05f, 0.1f, 10.0f) == 0);
    CHECK(encode_depth_mm(11.0f, 0.1f, 10.0f) == 0);
    CHECK(encode_depth_mm(65.0f, 0.1f, 70.0f) == 65000);
    CHECK(unpack_depth_mm(1500 >> 8, 1500 & 0xff) == 1500);

    // Frames are a 16-byte header, then pixels, so depth stays aligned whatever the size.
    static u64 storage[(64 + 17 * 17 * 3) / 8 + 1];
    Camera_Frame_Header *frame = (Camera_Frame_Header *)storage;
    CHECK((u8 *)get_depth_pixels(frame) - (u8 *)storage == 16);
    CHECK(get_color_frame_size(17, 17) == 16 + 17 * 17 * 3);
    CHECK(get_depth_frame_size(17, 17) == 16 + 17 * 17 * 2);
}

int main(int argc, char **argv)
{
    test_camera_model();
    test_imu_at_rest();
    test_imu_spin();
    CHECK(hash_imu_run(1) == hash_imu_run(1));
    CHECK(hash_imu_run(1) != hash_imu_run(2));
    test_lidar_ground();
    // The same frame whatever the number of threads; a different seed gives different noise.
    CHECK(hash_lidar_frame(7, 1) == hash_lidar_frame(7, 1));
    CHECK(hash_lidar_frame(7, 1) == hash_lidar_frame(7, 5));
    CHECK(hash_lidar_frame(7, 1) != hash_lidar_frame(8, 1));
    if (argc > 1) {
        banksia_path = argv[1];
        test_bad_frames();
        test_banksia();
    }
    return report_checks("sensors");
}
