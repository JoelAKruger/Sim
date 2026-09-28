#pragma once

#include "core/actuator.h"
#include "core/allocator.h"
#include "core/boulders.h"
#include "core/config.h"
#include "core/physics.h"
#include "core/raycast.h"
#include "core/robot.h"
#include "core/sensors/imu.h"
#include "core/sensors/lidar.h"
#include "core/soil.h"
#include "core/status_light.h"
#include "core/terrain.h"
#include "core/urdf.h"

#define WORLD_MAX_PROPS 256

// Who drives the robot's joints. Never both: switching hands over cleanly.
enum Control_Mode {
    CONTROL_CAN, // commands from the CAN bus, through the actuators
    CONTROL_KEYBOARD, // the arrow keys, through drive_robot
};

// A free rigid body that is not part of the robot: test boxes now, rocks later.
struct Prop {
    u32 body; // in the physics
    v3 half_extents;
    u32 color; // 0xRRGGBB, for drawing
    Pose previous; // pose before the most recent step, for render interpolation
    Pose current;
};

// The simulation. Owns the physics; everything outside core/ reads the world but never
// calls the physics directly.
struct World {
    Physics *physics;
    Sim_Config config;
    Terrain terrain;
    Soil soil; // deformable soil set into the terrain, when has_soil
    bool has_soil;
    Boulder_Field boulders; // fixed rocks on the terrain
    Ray_Scene ray_scene; // what the sensors' rays can hit
    Prop props[WORLD_MAX_PROPS];
    u32 prop_count;
    u64 step_count;
    f32 step_seconds;

    // The robot, loaded once. Its memory comes from the world's allocator.
    Robot robot;
    bool has_robot;
    Linear_Allocator *allocator; // given to create_world; outlives the world

    // The robot's motor controllers, and who drives them. Exactly one source has control.
    Actuator_Set actuators;
    Control_Mode control_mode;
    Status_Light status_light; // what the LED strip shows; off until commanded

    // Sensors on the robot, when the config enables them. They read the physics but never
    // change it.
    Imu_Sensor imu;
    bool has_imu;
    Lidar_Sensor lidar;
    bool has_lidar;

    // A digger, if the robot has the joints teleop.boom_joint and teleop.bucket_joint name.
    i32 boom_joint; // Robot_Joint index, or -1
    i32 bucket_joint;
    f32 bucket_soil_force; // N, the soil's push on the bucket over the last step
};

// The world must not move once created: the ray scene points into it.
bool create_world(World *world, Linear_Allocator *allocator, const Sim_Config *config);
void destroy_world(World *world);

// Advances exactly one fixed step of 1 / physics_hz seconds.
void step_world(World *world);

// Exact: derived from the step count, so it never drifts.
u64 get_sim_time_ns(const World *world);

// Builds the robot from a URDF, dropped onto the terrain at robot.spawn. Done once, before
// the first step. resource_dir resolves relative mesh paths ("" if unknown). On failure,
// error says why.
bool load_robot(World *world, const char *xml, u64 xml_size, const char *resource_dir, char *error,
                u32 error_size);

// A dynamic box centred at position, drawn in color (0xRRGGBB). False when the prop table
// is full.
bool add_box(World *world, v3 position, v3 half_extents, f32 density, u32 color);

// The ground's height at (x, y): the soil's, as it is now, where there is soil.
f32 get_world_ground_height(const World *world, f32 x, f32 y);

// Distance to the first hit along a unit direction, or -1 for a miss.
f32 cast_ray(const World *world, v3 origin, v3 direction, f32 max_distance);

// Hands control to mode. Either way the joints hold until the new source commands them:
// CAN commands from before the switch are forgotten, and keyboard control starts at rest.
void set_control_mode(World *world, Control_Mode mode);

// Puts the robot back at its spawn pose, at rest, and has it hold there until the controlling
// source commands it again (CAN commands from before the reset are forgotten).
void reset_robot(World *world);

// Hash of every body's pose and velocity (props and robot), for determinism checks.
u64 hash_world_state(const World *world);
