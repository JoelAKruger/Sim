#include "core/boulders.h"

#include <math.h>

#include "core/math.h"

#define BOULDER_POINTS 24 // points on each rock before the hull is taken
#define BOULDER_MAX_TRIANGLES 64 // per rock: a 24-point hull has at most 44
#define BOULDER_ATTEMPTS 40 // tries to find a clear spot for each rock

// Directions spread evenly over a sphere (a Fibonacci lattice).
static v3 get_sphere_direction(u32 index, u32 count)
{
    f32 z = 1.0f - 2.0f * ((f32)index + 0.5f) / (f32)count;
    f32 ring = sqrtf(max(1.0f - z * z, 0.0f));
    f32 angle = (f32)index * 2.39996323f; // the golden angle
    return v3{ring * cosf(angle), ring * sinf(angle), z};
}

// The lowest and highest ground under a rock's footprint.
static void get_ground_under(const Terrain *terrain, f32 x, f32 y, f32 radius, f32 *lowest,
                             f32 *highest)
{
    *lowest = *highest = get_terrain_height(terrain, x, y);
    for (u32 i = 0; i < 8; i++) {
        f32 angle = (f32)i * 0.25f * PI_F32;
        f32 ground =
            get_terrain_height(terrain, x + radius * cosf(angle), y + radius * sinf(angle));
        *lowest = min(*lowest, ground);
        *highest = max(*highest, ground);
    }
}

// True if a rock of this radius at (x, y) would reach onto the soil.
static bool is_near_soil(const Soil *soil, f32 x, f32 y, f32 radius)
{
    if (!soil) {
        return false;
    }
    const Terrain *grid = &soil->grid;
    f32 east = grid->origin_x + (f32)(grid->cols - 1) * grid->spacing;
    f32 south = grid->origin_y - (f32)(grid->rows - 1) * grid->spacing;
    return x + radius >= grid->origin_x && x - radius <= east && y - radius <= grid->origin_y &&
           y + radius >= south;
}

bool create_boulders(Boulder_Field *field, Physics *physics, Ray_Scene *scene,
                     const Terrain *terrain, const Soil *soil, const Sim_Config *config,
                     Linear_Allocator *allocator)
{
    *field = {};
    u32 count = config->boulder_count;
    if (count == 0) {
        return true;
    }
    field->boulders = ALLOCATE_ARRAY(allocator, Boulder, count);
    u32 capacity = count * BOULDER_MAX_TRIANGLES * 3;
    field->vertices = ALLOCATE_ARRAY(allocator, v3, capacity);
    field->normals = ALLOCATE_ARRAY(allocator, v3, capacity);
    if (!field->boulders || !field->vertices || !field->normals) {
        log_error("terrain: out of memory for %u boulders", count);
        return false;
    }

    Shape_Material material = {.friction = config->terrain_friction, .group = 0};

    Random random = create_random(config->terrain_seed, 4);
    f32 width = (f32)(terrain->cols - 1) * terrain->spacing;
    f32 depth = (f32)(terrain->rows - 1) * terrain->spacing;
    f32 smallest = config->boulder_size[0];
    f32 largest = config->boulder_size[1];
    f32 spawn_x = config->robot_spawn.x;
    f32 spawn_y = config->robot_spawn.y;
    for (u32 i = 0; i < count; i++) {
        // Mostly small rocks, a few big ones.
        f32 u = get_random_f32(&random);
        f32 size = smallest + (largest - smallest) * u * u;
        f32 half = 0.5f * size;
        f32 axes[3] = {half * (0.8f + 0.4f * get_random_f32(&random)),
                       half * (0.8f + 0.4f * get_random_f32(&random)),
                       half * (0.4f + 0.3f * get_random_f32(&random))}; // flatter on top
        f32 radius = max(axes[0], axes[1]) * 1.25f;

        bool placed = false;
        f32 x = 0.0f, y = 0.0f, lowest = 0.0f, highest = 0.0f;
        for (u32 attempt = 0; attempt < BOULDER_ATTEMPTS && !placed; attempt++) {
            x = terrain->origin_x + radius + (width - 2.0f * radius) * get_random_f32(&random);
            y = terrain->origin_y - radius - (depth - 2.0f * radius) * get_random_f32(&random);
            f32 dx = x - spawn_x, dy = y - spawn_y;
            f32 clear = BOULDER_SPAWN_CLEARANCE + radius;
            placed = dx * dx + dy * dy >= clear * clear && !is_near_soil(soil, x, y, radius);
            // Only on ground level enough for the rock: on a slope steeper than this it would
            // be buried on the uphill side and float on the downhill side.
            if (placed) {
                get_ground_under(terrain, x, y, radius, &lowest, &highest);
                placed = highest - lowest <= 0.6f * axes[2];
            }
            for (u32 j = 0; j < field->count && placed; j++) {
                const Boulder *other = &field->boulders[j];
                f32 ox = x - other->center.x, oy = y - other->center.y;
                f32 gap = radius + other->radius;
                placed = ox * ox + oy * oy >= gap * gap;
            }
        }
        if (!placed) {
            continue; // no room left: fewer rocks rather than overlapping ones
        }

        // Sunk about 30% of its height into the lowest ground under it.
        v3 center = {x, y, lowest + 0.4f * axes[2]};
        Quat yaw = make_quat_from_axis_angle(v3{0.0f, 0.0f, 1.0f},
                                             2.0f * PI_F32 * get_random_f32(&random));
        v3 points[BOULDER_POINTS];
        for (u32 p = 0; p < BOULDER_POINTS; p++) {
            v3 direction = get_sphere_direction(p, BOULDER_POINTS);
            f32 jitter = 0.75f + 0.5f * get_random_f32(&random);
            v3 local = {direction.x * axes[0] * jitter, direction.y * axes[1] * jitter,
                        direction.z * axes[2] * jitter};
            points[p] = center + rotate_vector(yaw, local);
        }
        u32 first_vertex = field->vertex_count;
        u32 written = make_convex_hull(points, BOULDER_POINTS, field->vertices + first_vertex,
                                       capacity - first_vertex);
        if (written == 0) {
            continue;
        }
        const v3 *triangles = field->vertices + first_vertex;
        for (u32 v = 0; v < written; v += 3) {
            v3 normal =
                normalize(cross(triangles[v + 1] - triangles[v], triangles[v + 2] - triangles[v]));
            field->normals[first_vertex + v] = normal;
            field->normals[first_vertex + v + 1] = normal;
            field->normals[first_vertex + v + 2] = normal;
        }
        // A body each, so the broadphase can pass over rocks nowhere near anything.
        u32 body = add_body(physics, identity_pose, true);
        add_hull_shape(physics, body, points, BOULDER_POINTS, &material);
        add_ray_hull(scene, NULL, triangles, written);
        field->vertex_count += written;
        field->boulders[field->count++] = Boulder{center, radius, first_vertex, written};
    }
    log_info("terrain: %u boulders, %.2f to %.2f m", field->count, (f64)smallest, (f64)largest);
    return true;
}
