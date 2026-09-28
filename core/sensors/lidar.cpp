#include "core/sensors/lidar.h"

#include <math.h>
#include <stdio.h>

// The R2 sequence's steps: 1/φ₂ and 1/φ₂², where φ₂ is the plastic number.
#define R2_STEP_U 0.7548776662466927
#define R2_STEP_V 0.5698402909980532
#define DEGREES_TO_RADIANS (PI_F32 / 180.0f)

bool create_lidar(Lidar_Sensor *lidar, const Lidar_Config *config, const Robot *robot,
                  const Ray_Scene *scene, Linear_Allocator *allocator, u64 seed, char *error,
                  u32 error_size)
{
    *lidar = {};
    lidar->config = config;
    lidar->random = create_random(seed, 2);
    if (!find_sensor_mount(robot, config->frame, "lidar.frame", &lidar->mount, error, error_size)) {
        return false;
    }
    lidar->capacity = get_lidar_capacity(config);
    for (u32 i = 0; i < 2; i++) {
        lidar->frames[i].points = ALLOCATE_ARRAY(allocator, Lidar_Point, lidar->capacity);
        if (!lidar->frames[i].points) {
            snprintf(error, error_size, "lidar: out of memory for %u points", lidar->capacity);
            return false;
        }
    }
    lidar->rays = ALLOCATE_ARRAY(allocator, Lidar_Ray, LIDAR_BATCH);
    lidar->nearby_capacity = scene->solid_capacity;
    lidar->nearby = ALLOCATE_ARRAY(allocator, u32, max(lidar->nearby_capacity, 1u));
    if (!lidar->rays || !lidar->nearby) {
        snprintf(error, error_size, "lidar: out of memory for its rays");
        return false;
    }
    return create_worker_pool(&lidar->pool, config->threads);
}

u32 get_lidar_capacity(const Lidar_Config *config)
{
    return (u32)ceil((f64)config->points_per_second / (f64)config->rate) + 2;
}

static f64 get_fraction(f64 x) { return x - floor(x); }

v3 get_lidar_direction(const Lidar_Config *config, u64 ray, u32 *line)
{
    f64 u = get_fraction(0.5 + (f64)ray * R2_STEP_U);
    f64 v = get_fraction(0.5 + (f64)ray * R2_STEP_V);
    f32 low = sinf(config->vertical_fov[0] * DEGREES_TO_RADIANS);
    f32 high = sinf(config->vertical_fov[1] * DEGREES_TO_RADIANS);
    f32 sin_elevation = low + (high - low) * (f32)v; // even over the solid angle
    f32 cos_elevation = sqrtf(max(1.0f - sin_elevation * sin_elevation, 0.0f));
    f32 azimuth = 2.0f * PI_F32 * (f32)u;
    *line = (u32)(ray % LIDAR_LINES);
    return v3{cos_elevation * cosf(azimuth), cos_elevation * sinf(azimuth), sin_elevation};
}

static u64 get_ray_time_ns(const Lidar_Config *config, u64 ray)
{
    return (u64)((f64)ray * 1e9 / (f64)config->points_per_second);
}

static u64 get_frame_start_ns(const Lidar_Config *config, u64 frame)
{
    return (u64)((f64)frame * 1e9 / (f64)config->rate);
}

static void finish_frame(Lidar_Sensor *lidar)
{
    lidar->building ^= 1;
    lidar->frame_ready = true;
    lidar->frame_number++;
    Lidar_Frame *next = &lidar->frames[lidar->building];
    next->start_ns = get_frame_start_ns(lidar->config, lidar->frame_number);
    next->count = 0;
}

void destroy_lidar(Lidar_Sensor *lidar) { destroy_worker_pool(&lidar->pool); }

struct Lidar_Cast {
    const Lidar_Config *config;
    const Ray_Scene *scene;
    const u32 *nearby;
    u32 nearby_count;
    Pose pose;
    u64 first_ray;
    Lidar_Ray *rays;
};

// Casts rays [first, end) of the batch. Runs on several threads at once: it only reads.
static void cast_rays(void *context, u32 first, u32 end)
{
    const Lidar_Cast *cast = (const Lidar_Cast *)context;
    f32 max_range = cast->config->range[1];
    for (u32 i = first; i < end; i++) {
        Lidar_Ray *ray = &cast->rays[i];
        ray->direction = get_lidar_direction(cast->config, cast->first_ray + i, &ray->line);
        v3 world_direction = rotate_vector(cast->pose.q, ray->direction);
        Ray_Hit hit = cast_scene_ray(cast->scene, cast->pose.p, world_direction, max_range,
                                     cast->nearby, cast->nearby_count);
        ray->range = hit.distance;
        ray->incidence = hit.distance >= 0.0f ? absolute(dot(hit.normal, world_direction)) : 0.0f;
    }
}

void update_lidar(Lidar_Sensor *lidar, const Robot *robot, const Ray_Scene *scene, u64 sim_time_ns)
{
    const Lidar_Config *config = lidar->config;
    Lidar_Cast cast = {.config = config,
                       .scene = scene,
                       .nearby = lidar->nearby,
                       .pose = get_sensor_pose(robot, &lidar->mount),
                       .rays = lidar->rays};
    // Only solids within range can be hit.
    for (u32 i = 0; i < scene->solid_count && cast.nearby_count < lidar->nearby_capacity; i++) {
        const Solid *solid = &scene->solids[i];
        f32 reach = config->range[1] + solid->bound;
        v3 offset = get_solid_center(solid) - cast.pose.p;
        if (dot(offset, offset) <= reach * reach) {
            lidar->nearby[cast.nearby_count++] = i;
        }
    }
    u64 due = (u64)((f64)sim_time_ns * 1e-9 * (f64)config->points_per_second);
    while (lidar->next_ray < due) {
        u32 count = (u32)min(due - lidar->next_ray, (u64)LIDAR_BATCH);
        cast.first_ray = lidar->next_ray;
        run_parallel(&lidar->pool, cast_rays, &cast, count);
        lidar->rays_cast += count;

        // In ray order on this thread, so the noise and the frames are the same however
        // the casting was split.
        for (u32 i = 0; i < count; i++) {
            u64 ray_index = lidar->next_ray + i;
            u64 time_ns = get_ray_time_ns(config, ray_index);
            while (time_ns >= get_frame_start_ns(config, lidar->frame_number + 1)) {
                finish_frame(lidar);
            }
            const Lidar_Ray *ray = &lidar->rays[i];
            Lidar_Frame *frame = &lidar->frames[lidar->building];
            if (ray->range < config->range[0] || frame->count == lidar->capacity) {
                continue; // no return, or inside the blind zone
            }
            f32 range =
                max(ray->range + config->range_noise * get_random_gaussian(&lidar->random), 0.0f);
            Lidar_Point *point = &frame->points[frame->count++];
            point->x = range * ray->direction.x;
            point->y = range * ray->direction.y;
            point->z = range * ray->direction.z;
            point->intensity = min(config->reflectivity * ray->incidence, 255.0f);
            point->tag = 0;
            point->line = (u8)ray->line;
            point->time_ns = time_ns;
        }
        lidar->next_ray += count;
    }
    if (sim_time_ns >= get_frame_start_ns(config, lidar->frame_number + 1)) {
        finish_frame(lidar);
    }
}

const Lidar_Frame *take_lidar_frame(Lidar_Sensor *lidar)
{
    if (!lidar->frame_ready) {
        return NULL;
    }
    lidar->frame_ready = false;
    return &lidar->frames[lidar->building ^ 1];
}

const Lidar_Frame *get_last_lidar_frame(const Lidar_Sensor *lidar)
{
    return lidar->frame_number > 0 ? &lidar->frames[lidar->building ^ 1] : NULL;
}
