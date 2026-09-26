#include "core/world.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "core/math.h"

#define SPAWN_CLEARANCE 0.05f // m between the robot's lowest point and the ground at spawn

static bool create_terrain_body(World *world)
{
    Terrain *terrain = &world->terrain;

    b3HeightFieldDef field_def = {};
    field_def.heights = terrain->heights;
    field_def.scale = {terrain->spacing, 1.0f, terrain->spacing};
    field_def.countX = (int)terrain->cols;
    field_def.countZ = (int)terrain->rows;
    field_def.globalMinimumHeight = terrain->min_height;
    field_def.globalMaximumHeight = terrain->max_height;
    world->height_field = b3CreateHeightField(&field_def);
    if (!world->height_field) {
        log_error("world: Box3D rejected the terrain height field");
        return false;
    }

    // Box3D height fields are y-up in their own frame, with columns along local x and
    // rows along local z. Rotating +90 degrees about x takes local y to world z and
    // local z to world -y, which is exactly the row direction Terrain uses.
    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.position = {terrain->origin_x, terrain->origin_y, 0.0f};
    body_def.rotation = make_quat_from_axis_angle({1.0f, 0.0f, 0.0f}, 0.5f * B3_PI);
    body_def.name = "terrain";
    world->terrain_body = b3CreateBody(world->id, &body_def);

    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.baseMaterial.friction = world->config.terrain_friction;
    b3CreateHeightFieldShape(world->terrain_body, &shape_def, world->height_field);

    // Box3D stores heights quantised to 16 bits. Copy the quantised values back so the
    // renderer and get_terrain_height describe exactly the surface physics collides with.
    const u16 *quantised = b3GetHeightFieldCompressedHeights(world->height_field);
    if (quantised) {
        for (u32 i = 0; i < terrain->rows * terrain->cols; i++) {
            terrain->heights[i] = world->height_field->minHeight +
                                  world->height_field->heightScale * (f32)quantised[i];
        }
    }
    return true;
}

bool create_world(World *world, Linear_Allocator *allocator, const Sim_Config *config)
{
    *world = {};
    world->config = *config;
    world->allocator = allocator;
    world->step_seconds = 1.0f / (f32)config->physics_hz;

    b3WorldDef world_def = b3DefaultWorldDef();
    world_def.gravity = config->gravity;
    world_def.workerCount = config->workers;
    world->id = b3CreateWorld(&world_def);

    if (config->terrain_heightmap[0]) {
        char error[256];
        if (!load_heightmap(&world->terrain, allocator, config, error, sizeof(error))) {
            log_error("terrain: %s", error);
            return false;
        }
    } else if (!generate_terrain(&world->terrain, allocator, config)) {
        return false;
    }
    return create_terrain_body(world) &&
           create_boulders(&world->boulders, world->id, &world->terrain, config, allocator);
}

void destroy_world(World *world)
{
    if (world->has_lidar) {
        destroy_lidar(&world->lidar);
        world->has_lidar = false;
    }
    world->has_imu = false;
    if (b3World_IsValid(world->id)) {
        b3DestroyWorld(world->id); // also destroys the robot's bodies
    }
    // The shape referenced the height field, so it can only go after the world.
    if (world->height_field) {
        b3DestroyHeightField(world->height_field);
    }
    world->id = b3_nullWorldId;
    world->height_field = NULL;
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
    b3World_Step(world->id, world->step_seconds, (int)world->config.substeps);
    world->step_count++;
    for (u32 i = 0; i < world->prop_count; i++) {
        world->props[i].current = b3Body_GetTransform(world->props[i].body);
    }
    if (world->has_robot) {
        read_robot_state(&world->robot);
        u64 now = get_sim_time_ns(world);
        if (world->has_imu) {
            update_imu(&world->imu, &world->robot, world->config.gravity, world->step_seconds, now);
        }
        if (world->has_lidar) {
            update_lidar(&world->lidar, &world->robot, world->id, now);
        }
    }
}

// Where the robot's root link goes: at robot.spawn, low enough to settle quickly but
// clear of the highest ground anywhere under it.
static b3Transform get_spawn_pose(const World *world, const Urdf_Model *model)
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
                ground =
                    max(ground, get_terrain_height(&world->terrain, spawn.x + dx, spawn.y + dy));
            }
        }
    }
    ground = max(ground, get_terrain_height(&world->terrain, spawn.x, spawn.y));

    b3Transform pose;
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
        .collision_group = -1,
        .joint_hertz = 0.25f * (f32)(world->config.physics_hz * world->config.substeps),
        .resource_dir = dir,
    };
    b3Transform spawn = get_spawn_pose(world, &model);
    if (!create_robot(&world->robot, world->id, allocator, &model, &settings, spawn,
                      world->step_seconds, error, error_size)) {
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
        (world->has_lidar && !create_lidar(&world->lidar, &config->lidar, &world->robot, allocator,
                                           config->sensor_seed, error, error_size))) {
        world->has_imu = false;
        world->has_lidar = false;
        destroy_robot(&world->robot);
        return false;
    }
    world->has_robot = true;
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

    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = b3_dynamicBody;
    body_def.position = position;
    b3BodyId body = b3CreateBody(world->id, &body_def);

    b3BoxHull hull = b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z);
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.density = density;
    shape_def.baseMaterial.friction = 0.7f;
    b3CreateHullShape(body, &shape_def, &hull.base);

    Prop *prop = &world->props[world->prop_count++];
    prop->body = body;
    prop->half_extents = half_extents;
    prop->color = color;
    prop->current = b3Body_GetTransform(body);
    prop->previous = prop->current;
    return true;
}

f32 cast_ray(const World *world, v3 origin, v3 direction, f32 max_distance)
{
    b3RayResult result =
        b3World_CastRayClosest(world->id, origin, max_distance * direction, b3DefaultQueryFilter());
    return result.hit ? result.fraction * max_distance : -1.0f;
}

static u64 hash_body(u64 hash, b3BodyId body)
{
    b3Transform transform = b3Body_GetTransform(body);
    v3 linear = b3Body_GetLinearVelocity(body);
    v3 angular = b3Body_GetAngularVelocity(body);
    hash = hash_bytes(hash, &transform, sizeof(transform));
    hash = hash_bytes(hash, &linear, sizeof(linear));
    return hash_bytes(hash, &angular, sizeof(angular));
}

u64 hash_world_state(const World *world)
{
    u64 hash = HASH_SEED;
    for (u32 i = 0; i < world->prop_count; i++) {
        hash = hash_body(hash, world->props[i].body);
    }
    if (world->has_robot) {
        for (u32 b = 0; b < world->robot.body_count; b++) {
            hash = hash_body(hash, world->robot.bodies[b].id);
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
