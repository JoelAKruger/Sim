#include "core/world.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/math.h"

#define SPAWN_CLEARANCE 0.05f // m between the robot's lowest point and the ground at spawn
#define ROBOT_SHAPES 1024 // ray scene room for the robot's collision shapes
#define PLANES_PER_SHAPE 64 // ray scene room for hull faces
#define ROBOT_COLLISION_GROUP 1 // the robot's shapes don't meet each other, unless self_collide
#define GROUND_TILE_CELLS 16 // terrain cells along each side of one ground mesh

// The rigid ground: the terrain's triangles, less the soil's. The soil is its own thing in
// the physics.
static bool create_ground_body(World *world)
{
    const Terrain *terrain = &world->terrain;
    const Soil *soil = world->has_soil ? &world->soil : NULL;
    u32 vertex_count = terrain->rows * terrain->cols;
    v3 *vertices = (v3 *)malloc(vertex_count * sizeof(v3));
    u32 *indices = (u32 *)malloc(GROUND_TILE_CELLS * GROUND_TILE_CELLS * 6 * sizeof(u32));
    if (!vertices || !indices) {
        free(vertices);
        free(indices);
        log_error("world: out of memory for the ground's triangles");
        return false;
    }
    for (u32 row = 0; row < terrain->rows; row++) {
        for (u32 col = 0; col < terrain->cols; col++) {
            vertices[row * terrain->cols + col] =
                v3{terrain->origin_x + (f32)col * terrain->spacing,
                   terrain->origin_y - (f32)row * terrain->spacing,
                   terrain->heights[row * terrain->cols + col]};
        }
    }
    Shape_Material material = {.friction = world->config.terrain_friction, .group = 0};
    // A mesh per tile, as Chrono's mesh setup slows with the square of a mesh's size, and a
    // body per mesh, as Chrono mistakes a triangle for a shape in a body with several.
    for (u32 tile_row = 0; tile_row + 1 < terrain->rows; tile_row += GROUND_TILE_CELLS) {
        for (u32 tile_col = 0; tile_col + 1 < terrain->cols; tile_col += GROUND_TILE_CELLS) {
            // Two triangles per cell, split along the south-west to north-east diagonal as
            // get_terrain_height assumes, counter-clockwise seen from above.
            u32 triangle_count = 0;
            u32 last_row = min(tile_row + GROUND_TILE_CELLS, terrain->rows - 1);
            u32 last_col = min(tile_col + GROUND_TILE_CELLS, terrain->cols - 1);
            for (u32 row = tile_row; row < last_row; row++) {
                for (u32 col = tile_col; col < last_col; col++) {
                    if (soil && is_soil_cell(soil, row, col)) {
                        continue;
                    }
                    u32 v11 = row * terrain->cols + col;
                    u32 v12 = v11 + 1;
                    u32 v21 = v11 + terrain->cols;
                    u32 v22 = v21 + 1;
                    u32 cell[6] = {v11, v21, v12, v22, v12, v21};
                    memcpy(&indices[triangle_count * 3], cell, sizeof(cell));
                    triangle_count += 2;
                }
            }
            if (triangle_count > 0) {
                u32 body = add_body(world->physics, identity_pose, true);
                add_mesh_shape(world->physics, body, vertices, vertex_count, indices,
                               triangle_count, &material);
            }
        }
    }
    free(vertices);
    free(indices);
    return true;
}

static bool create_world_soil(World *world)
{
    const Sim_Config *config = &world->config;
    char error[256];
    if (!create_soil(&world->soil, &world->terrain, config, world->allocator, error,
                     sizeof(error))) {
        log_error("%s", error);
        return false;
    }
    Soil_Settings settings = {
        .bekker_kphi = config->soil_bekker_kphi,
        .bekker_kc = config->soil_bekker_kc,
        .bekker_n = config->soil_bekker_n,
        .cohesion = config->soil_cohesion,
        .friction_angle = config->soil_friction_angle,
        .janosi_shear = config->soil_janosi_shear,
        .elastic_stiffness = config->soil_elastic_stiffness,
        .damping = config->soil_damping,
        .bulldozing = config->soil_bulldozing,
        .erosion_angle = config->soil_erosion_angle,
    };
    if (!create_physics_soil(world->physics, &world->soil, &world->terrain, &settings, error,
                             sizeof(error))) {
        log_error("%s", error);
        return false;
    }
    world->has_soil = true;
    return true;
}

bool create_world(World *world, Linear_Allocator *allocator, const Sim_Config *config)
{
    *world = {};
    world->config = *config;
    world->allocator = allocator;
    world->step_seconds = 1.0f / (f32)config->physics_hz;
    world->boom_joint = -1;
    world->bucket_joint = -1;

    Physics_Settings settings = {
        .gravity = config->gravity,
        .threads = config->workers,
        .solver_iterations = config->solver_iterations,
    };
    world->physics = create_physics(&settings);

    if (config->terrain_heightmap[0]) {
        char error[256];
        if (!load_heightmap(&world->terrain, allocator, config, error, sizeof(error))) {
            log_error("terrain: %s", error);
            return false;
        }
    } else if (!generate_terrain(&world->terrain, allocator, config)) {
        return false;
    }
    if (config->soil_enabled && !create_world_soil(world)) {
        return false;
    }
    const Soil *soil = world->has_soil ? &world->soil : NULL;
    u32 solids = config->boulder_count + WORLD_MAX_PROPS + ROBOT_SHAPES;
    if (!create_ray_scene(&world->ray_scene, &world->terrain, soil, solids,
                          solids * PLANES_PER_SHAPE, allocator)) {
        log_error("world: out of memory for the ray scene");
        return false;
    }
    return create_ground_body(world) &&
           create_boulders(&world->boulders, world->physics, &world->ray_scene, &world->terrain,
                           soil, config, allocator);
}

void destroy_world(World *world)
{
    if (world->has_lidar) {
        destroy_lidar(&world->lidar);
        world->has_lidar = false;
    }
    world->has_imu = false;
    if (world->physics) {
        destroy_physics(world->physics); // and every body in it, the robot's too
    }
    world->physics = NULL;
    world->prop_count = 0;
    world->has_robot = false;
}

void step_world(World *world)
{
    for (u32 i = 0; i < world->prop_count; i++) {
        world->props[i].previous = world->props[i].current;
    }
    if (world->has_robot) {
        if (world->control_mode == CONTROL_CAN) {
            u64 timeout_steps =
                (u64)(world->config.command_timeout_ms * 1e-3f * (f32)world->config.physics_hz);
            update_actuators(&world->actuators, &world->robot, world->step_count, timeout_steps);
        }
        apply_robot_commands(&world->robot);
    }
    step_physics(world->physics, world->step_seconds, world->config.substeps);
    world->step_count++;
    for (u32 i = 0; i < world->prop_count; i++) {
        world->props[i].current = get_body_pose(world->physics, world->props[i].body);
    }
    if (world->has_robot) {
        read_robot_state(&world->robot);
        if (world->bucket_joint >= 0) {
            const Robot_Joint *bucket = &world->robot.joints[world->bucket_joint];
            u32 body = world->robot.bodies[bucket->body_b].physics_body;
            world->bucket_soil_force = get_length(get_soil_force(world->physics, body));
        }
        u64 now = get_sim_time_ns(world);
        if (world->has_imu) {
            update_imu(&world->imu, &world->robot, world->config.gravity, world->step_seconds, now);
        }
        if (world->has_lidar) {
            update_lidar(&world->lidar, &world->robot, &world->ray_scene, now);
        }
    }
}

f32 get_world_ground_height(const World *world, f32 x, f32 y)
{
    return get_ground_height(&world->terrain, world->has_soil ? &world->soil : NULL, x, y);
}

// Where the robot's root link goes: at robot.spawn, low enough to settle quickly but
// clear of the highest ground anywhere under it.
static Pose get_spawn_pose(const World *world, const Urdf_Model *model)
{
    Spawn_Pose spawn = world->config.robot_spawn;
    v3 lower, upper;
    get_rest_bounds(model, &lower, &upper);
    f32 reach_x = max(absolute(lower.x), absolute(upper.x));
    f32 reach_y = max(absolute(lower.y), absolute(upper.y));
    f32 radius = sqrtf(reach_x * reach_x + reach_y * reach_y);

    f32 ground = -INFINITY;
    f32 step = max(0.5f * world->terrain.spacing, 0.05f);
    for (f32 dx = -radius; dx <= radius; dx += step) {
        for (f32 dy = -radius; dy <= radius; dy += step) {
            if (dx * dx + dy * dy <= radius * radius) {
                ground = max(ground, get_world_ground_height(world, spawn.x + dx, spawn.y + dy));
            }
        }
    }
    ground = max(ground, get_world_ground_height(world, spawn.x, spawn.y));

    Pose pose;
    pose.p = v3{spawn.x, spawn.y, ground - lower.z + SPAWN_CLEARANCE};
    pose.q = make_quat_from_axis_angle(v3{0.0f, 0.0f, 1.0f}, spawn.yaw);
    return pose;
}

bool load_robot(World *world, const char *xml, u64 xml_size, const char *resource_dir, char *error,
                u32 error_size)
{
    if (world->has_robot) {
        snprintf(error, error_size, "the world already has a robot");
        return false;
    }
    Linear_Allocator *allocator = world->allocator;
    Urdf_Model model;
    if (!parse_urdf(&model, allocator, xml, xml_size, error, error_size)) {
        return false;
    }
    for (u32 j = 0; j < model.joint_count; j++) {
        Urdf_Joint_Type type = model.joints[j].type;
        if (type == URDF_JOINT_FLOATING || type == URDF_JOINT_PLANAR) {
            snprintf(error, error_size, "joint \"%s\": %s joints are not supported",
                     model.joints[j].name, type == URDF_JOINT_FLOATING ? "floating" : "planar");
            return false;
        }
    }

    // resource_dir must outlive this call, so keep a copy with the robot.
    u64 dir_length = strlen(resource_dir ? resource_dir : "") + 1;
    char *dir = (char *)allocate_memory(allocator, dir_length, 1);
    if (!dir) {
        snprintf(error, error_size, "out of memory for the robot");
        return false;
    }
    memcpy(dir, resource_dir ? resource_dir : "", dir_length);

    Robot_Settings settings = {
        .friction = world->config.robot_friction,
        .motor_torque = world->config.motor_torque,
        .servo_speed = world->config.servo_speed,
        .collision_group = ROBOT_COLLISION_GROUP,
        .resource_dir = dir,
    };
    Pose spawn = get_spawn_pose(world, &model);
    if (!create_robot(&world->robot, world->physics, &world->ray_scene, allocator, &model,
                      &settings, spawn, world->step_seconds, error, error_size)) {
        return false;
    }
    if (!create_actuators(&world->actuators, &world->robot, allocator, error, error_size)) {
        destroy_robot(&world->robot);
        return false;
    }
    disable_unmounted_sensors(&world->config, &world->robot);
    const Sim_Config *config = &world->config;
    world->has_imu = config->imu.enabled;
    world->has_lidar = config->lidar.enabled;
    if ((world->has_imu && !create_imu(&world->imu, &config->imu, &world->robot,
                                       config->sensor_seed, error, error_size)) ||
        (world->has_lidar &&
         !create_lidar(&world->lidar, &config->lidar, &world->robot, &world->ray_scene, allocator,
                       config->sensor_seed, error, error_size))) {
        world->has_imu = false;
        world->has_lidar = false;
        destroy_robot(&world->robot);
        return false;
    }
    world->has_robot = true;
    world->boom_joint = find_robot_joint(&world->robot, config->teleop_boom_joint);
    world->bucket_joint = find_robot_joint(&world->robot, config->teleop_bucket_joint);
    if (world->boom_joint >= 0 || world->bucket_joint >= 0) {
        log_info("digger: %s and %s (I/K and U/O in keyboard mode)",
                 world->boom_joint >= 0 ? config->teleop_boom_joint : "no boom",
                 world->bucket_joint >= 0 ? config->teleop_bucket_joint : "no bucket");
    }
    const Robot *robot = &world->robot;
    f32 mass = 0.0f;
    for (u32 b = 0; b < robot->body_count; b++) {
        mass += robot->bodies[b].mass;
    }
    log_info("robot %s: %u links as %u bodies, %u joints (%u ball joints), %u drive wheels, "
             "%u actuators, %.2f kg",
             model.name, model.link_count, robot->body_count, robot->joint_count, robot->ball_count,
             robot->wheel_count, world->actuators.count, (f64)mass);
    if (world->has_lidar) {
        log_info("lidar on %s: %.0f rays/s, %.0f Hz frames", config->lidar.frame,
                 (f64)config->lidar.points_per_second, (f64)config->lidar.rate);
    }
    if (world->has_imu) {
        log_info("imu on %s: %.0f Hz", config->imu.frame, (f64)config->imu.rate);
    }
    return true;
}

u64 get_sim_time_ns(const World *world)
{
    return world->step_count * NS_PER_S / world->config.physics_hz;
}

bool add_box(World *world, v3 position, v3 half_extents, f32 density, u32 color)
{
    if (world->prop_count == WORLD_MAX_PROPS) {
        log_warning("world: prop table full (%u)", WORLD_MAX_PROPS);
        return false;
    }
    Prop *prop = &world->props[world->prop_count];
    Pose pose = {position, identity_quat};
    prop->body = add_body(world->physics, pose, false);
    v3 h = half_extents;
    f32 mass = density * 8.0f * h.x * h.y * h.z;
    Mat3 inertia = {};
    inertia.cx.x = mass / 3.0f * (h.y * h.y + h.z * h.z);
    inertia.cy.y = mass / 3.0f * (h.x * h.x + h.z * h.z);
    inertia.cz.z = mass / 3.0f * (h.x * h.x + h.y * h.y);
    set_body_mass(world->physics, prop->body, mass, v3{0.0f, 0.0f, 0.0f}, inertia);
    Shape_Material material = {.friction = 0.7f, .group = 0};
    add_box_shape(world->physics, prop->body, identity_pose, half_extents, &material);
    add_ray_box(&world->ray_scene, &prop->current, identity_pose, half_extents);
    if (world->has_soil) {
        add_soil_domain(world->physics, prop->body, v3{0.0f, 0.0f, 0.0f},
                        2.0f * half_extents + v3{0.1f, 0.1f, 0.1f});
    }

    prop->half_extents = half_extents;
    prop->color = color;
    prop->current = pose;
    prop->previous = pose;
    world->prop_count++;
    return true;
}

f32 cast_ray(const World *world, v3 origin, v3 direction, f32 max_distance)
{
    return cast_scene_ray(&world->ray_scene, origin, direction, max_distance, NULL, 0).distance;
}

static u64 hash_body(u64 hash, const Physics *physics, u32 body)
{
    Pose pose = get_body_pose(physics, body);
    v3 linear = get_body_velocity(physics, body);
    v3 angular = get_body_angular_velocity(physics, body);
    hash = hash_bytes(hash, &pose, sizeof(pose));
    hash = hash_bytes(hash, &linear, sizeof(linear));
    return hash_bytes(hash, &angular, sizeof(angular));
}

u64 hash_world_state(const World *world)
{
    u64 hash = HASH_SEED;
    for (u32 i = 0; i < world->prop_count; i++) {
        hash = hash_body(hash, world->physics, world->props[i].body);
    }
    if (world->has_robot) {
        for (u32 b = 0; b < world->robot.body_count; b++) {
            hash = hash_body(hash, world->physics, world->robot.bodies[b].physics_body);
        }
    }
    return hash;
}

void set_control_mode(World *world, Control_Mode mode)
{
    world->control_mode = mode;
    if (!world->has_robot) {
        return;
    }
    reset_actuators(&world->actuators);
    if (mode == CONTROL_KEYBOARD) {
        drive_robot(&world->robot, 0.0f, 0.0f);
    }
}

void reset_robot(World *world)
{
    if (!world->has_robot) {
        return;
    }
    reset_robot(&world->robot);
    if (world->has_imu) {
        reset_imu(&world->imu);
    }
    set_control_mode(world, world->control_mode);
}
