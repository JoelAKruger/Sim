#include "core/raycast.h"

#include <math.h>

#define HIT_EPSILON 1e-6f // slack at triangle edges, so rays can't slip between two cells

bool create_ray_scene(Ray_Scene *scene, const Terrain *terrain, const Soil *soil,
                      u32 solid_capacity, u32 plane_capacity, Linear_Allocator *allocator)
{
    *scene = {};
    scene->terrain = terrain;
    scene->soil = soil;
    scene->solids = ALLOCATE_ARRAY(allocator, Solid, max(solid_capacity, 1u));
    scene->planes = ALLOCATE_ARRAY(allocator, Plane, max(plane_capacity, 1u));
    scene->solid_capacity = solid_capacity;
    scene->plane_capacity = plane_capacity;
    return scene->solids && scene->planes;
}

static Solid *add_solid(Ray_Scene *scene, Solid_Kind kind, const Pose *frame, Pose local)
{
    if (scene->solid_count == scene->solid_capacity) {
        return NULL;
    }
    Solid *solid = &scene->solids[scene->solid_count++];
    *solid = {};
    solid->kind = kind;
    solid->frame = frame;
    solid->local = local;
    return solid;
}

bool add_ray_box(Ray_Scene *scene, const Pose *frame, Pose local, v3 half_extents)
{
    Solid *solid = add_solid(scene, SOLID_BOX, frame, local);
    if (solid) {
        solid->size = half_extents;
        solid->bound = get_length(half_extents);
    }
    return solid != NULL;
}

bool add_ray_sphere(Ray_Scene *scene, const Pose *frame, v3 center, f32 radius)
{
    Solid *solid = add_solid(scene, SOLID_SPHERE, frame, Pose{center, identity_quat});
    if (solid) {
        solid->size = v3{radius, radius, radius};
        solid->bound = radius;
    }
    return solid != NULL;
}

bool add_ray_cylinder(Ray_Scene *scene, const Pose *frame, Pose local, f32 radius, f32 length)
{
    Solid *solid = add_solid(scene, SOLID_CYLINDER, frame, local);
    if (solid) {
        solid->size = v3{radius, radius, 0.5f * length};
        solid->bound = sqrtf(radius * radius + 0.25f * length * length);
    }
    return solid != NULL;
}

bool add_ray_hull(Ray_Scene *scene, const Pose *frame, const v3 *triangles, u32 vertex_count)
{
    if (vertex_count < 12) {
        return false;
    }
    v3 centroid = {0.0f, 0.0f, 0.0f};
    for (u32 i = 0; i < vertex_count; i++) {
        centroid += (1.0f / (f32)vertex_count) * triangles[i];
    }
    Solid *solid = add_solid(scene, SOLID_HULL, frame, Pose{centroid, identity_quat});
    if (!solid) {
        return false;
    }
    solid->first_plane = scene->plane_count;
    for (u32 i = 0; i < vertex_count; i += 3) {
        v3 a = triangles[i] - centroid;
        v3 normal =
            normalize(cross(triangles[i + 1] - triangles[i], triangles[i + 2] - triangles[i]));
        Plane plane = {normal, dot(normal, a)};
        solid->bound = max(solid->bound, get_length(a));
        bool seen = get_length(normal) == 0.0f;
        for (u32 p = solid->first_plane; p < scene->plane_count && !seen; p++) {
            const Plane *other = &scene->planes[p];
            seen = dot(other->normal, normal) > 1.0f - 1e-5f &&
                   absolute(other->offset - plane.offset) < 1e-5f;
        }
        if (!seen) {
            if (scene->plane_count == scene->plane_capacity) {
                scene->solid_count--;
                scene->plane_count = solid->first_plane;
                return false;
            }
            scene->planes[scene->plane_count++] = plane;
        }
    }
    solid->plane_count = scene->plane_count - solid->first_plane;
    return true;
}

static Pose get_solid_pose(const Solid *solid)
{
    return solid->frame ? multiply_poses(*solid->frame, solid->local) : solid->local;
}

v3 get_solid_center(const Solid *solid)
{
    return solid->frame ? transform_point(*solid->frame, solid->local.p) : solid->local.p;
}

// Möller-Trumbore, both sides. Updates *best if the triangle is hit nearer than it.
static bool hit_triangle(v3 a, v3 b, v3 c, v3 origin, v3 direction, f32 t_min, f32 *best,
                         v3 *normal)
{
    v3 edge_1 = b - a;
    v3 edge_2 = c - a;
    v3 p = cross(direction, edge_2);
    f32 det = dot(edge_1, p);
    if (absolute(det) < 1e-12f) {
        return false;
    }
    f32 inverse = 1.0f / det;
    v3 s = origin - a;
    f32 u = dot(s, p) * inverse;
    if (u < -HIT_EPSILON || u > 1.0f + HIT_EPSILON) {
        return false;
    }
    v3 q = cross(s, edge_1);
    f32 v = dot(direction, q) * inverse;
    if (v < -HIT_EPSILON || u + v > 1.0f + HIT_EPSILON) {
        return false;
    }
    f32 t = dot(edge_2, q) * inverse;
    if (t < t_min || t >= *best) {
        return false;
    }
    *best = t;
    *normal = normalize(cross(edge_1, edge_2));
    return true;
}

// Called for each cell a ray crosses, in order, with the part of the ray inside it; true
// stops the walk.
typedef bool Cell_Visitor(void *context, u32 row, u32 col, f32 t_in, f32 t_out);

// Walks the cells of a grid of rows x cols square cells, each size across, whose north-west
// corner is at (west, north) (rows run south), along the ray from t_start to t_end.
static bool walk_cells(f32 west, f32 north, f32 size, u32 rows, u32 cols, v3 origin, v3 direction,
                       f32 t_start, f32 t_end, Cell_Visitor *visit, void *context)
{
    // In cell units: u along columns (east), w along rows (south).
    f32 u0 = (origin.x - west) / size;
    f32 w0 = (north - origin.y) / size;
    f32 du = direction.x / size;
    f32 dw = -direction.y / size;
    f32 extents[2] = {(f32)cols, (f32)rows};
    f32 starts[2] = {u0, w0};
    f32 steps[2] = {du, dw};
    for (u32 axis = 0; axis < 2; axis++) {
        if (absolute(steps[axis]) < 1e-12f) {
            if (starts[axis] < 0.0f || starts[axis] > extents[axis]) {
                return false;
            }
            continue;
        }
        f32 a = (0.0f - starts[axis]) / steps[axis];
        f32 b = (extents[axis] - starts[axis]) / steps[axis];
        t_start = max(t_start, min(a, b));
        t_end = min(t_end, max(a, b));
    }
    if (t_start > t_end) {
        return false;
    }
    i32 col = (i32)floorf(u0 + du * t_start);
    i32 row = (i32)floorf(w0 + dw * t_start);
    col = min(max(col, 0), (i32)cols - 1);
    row = min(max(row, 0), (i32)rows - 1);
    i32 step_col = du > 0.0f ? 1 : -1;
    i32 step_row = dw > 0.0f ? 1 : -1;
    f32 next_col = absolute(du) < 1e-12f ? INFINITY : ((f32)(col + (du > 0.0f ? 1 : 0)) - u0) / du;
    f32 next_row = absolute(dw) < 1e-12f ? INFINITY : ((f32)(row + (dw > 0.0f ? 1 : 0)) - w0) / dw;
    f32 t = t_start;
    for (;;) {
        f32 t_out = min(min(next_col, next_row), t_end);
        if (visit(context, (u32)row, (u32)col, t, t_out)) {
            return true;
        }
        if (t_out >= t_end) {
            return false;
        }
        if (next_col < next_row) {
            col += step_col;
            t = next_col;
            next_col = ((f32)(col + (du > 0.0f ? 1 : 0)) - u0) / du;
        } else {
            row += step_row;
            t = next_row;
            next_row = ((f32)(row + (dw > 0.0f ? 1 : 0)) - w0) / dw;
        }
        if (col < 0 || row < 0 || col >= (i32)cols || row >= (i32)rows) {
            return false;
        }
    }
}

struct Ground_Cast {
    const Terrain *grid;
    const Soil *hole; // skip the terrain's cells under this soil (terrain casts only)
    v3 origin;
    v3 direction;
    f32 best;
    v3 normal;
    u32 block_row; // soil casts: the block being walked
    u32 block_col;
};

// The ray against one cell's two triangles, split as the terrain is.
static bool visit_ground_cell(void *context, u32 row, u32 col, f32 t_in, f32 t_out)
{
    Ground_Cast *cast = (Ground_Cast *)context;
    const Terrain *grid = cast->grid;
    if (cast->hole && is_soil_cell(cast->hole, row, col)) {
        return false;
    }
    u32 n = grid->cols;
    const f32 *h = grid->heights;
    f32 h11 = h[row * n + col], h12 = h[row * n + col + 1];
    f32 h21 = h[(row + 1) * n + col], h22 = h[(row + 1) * n + col + 1];
    f32 z_in = cast->origin.z + cast->direction.z * t_in;
    f32 z_out = cast->origin.z + cast->direction.z * t_out;
    f32 top = max(max(h11, h12), max(h21, h22));
    if (min(z_in, z_out) > top + HIT_EPSILON) {
        return false; // passes over the cell
    }
    f32 s = grid->spacing;
    f32 x = grid->origin_x + (f32)col * s;
    f32 y = grid->origin_y - (f32)row * s;
    v3 p11 = {x, y, h11}, p12 = {x + s, y, h12};
    v3 p21 = {x, y - s, h21}, p22 = {x + s, y - s, h22};
    f32 t_min = max(t_in - 1e-4f, 0.0f);
    bool hit = hit_triangle(p11, p21, p12, cast->origin, cast->direction, t_min, &cast->best,
                            &cast->normal);
    hit = hit_triangle(p22, p12, p21, cast->origin, cast->direction, t_min, &cast->best,
                       &cast->normal) ||
          hit;
    // Cells are visited in order, so the first hit is the nearest.
    return hit;
}

static bool visit_soil_cell(void *context, u32 row, u32 col, f32 t_in, f32 t_out)
{
    Ground_Cast *cast = (Ground_Cast *)context;
    return visit_ground_cell(context, cast->block_row * SOIL_BLOCK_CELLS + row,
                             cast->block_col * SOIL_BLOCK_CELLS + col, t_in, t_out);
}

// The soil's blocks first, then the cells of any block the ray dips into.
static bool visit_soil_block(void *context, u32 row, u32 col, f32 t_in, f32 t_out)
{
    Ground_Cast *cast = (Ground_Cast *)context;
    const Soil *soil = cast->hole;
    f32 z_in = cast->origin.z + cast->direction.z * t_in;
    f32 z_out = cast->origin.z + cast->direction.z * t_out;
    if (min(z_in, z_out) > soil->block_high[row * soil->block_cols + col] + HIT_EPSILON) {
        return false;
    }
    const Terrain *grid = &soil->grid;
    u32 rows = min((u32)SOIL_BLOCK_CELLS, grid->rows - 1 - row * SOIL_BLOCK_CELLS);
    u32 cols = min((u32)SOIL_BLOCK_CELLS, grid->cols - 1 - col * SOIL_BLOCK_CELLS);
    f32 block_size = (f32)SOIL_BLOCK_CELLS * grid->spacing;
    Ground_Cast cells = *cast;
    cells.hole = NULL;
    cells.block_row = row;
    cells.block_col = col;
    bool hit =
        walk_cells(grid->origin_x + (f32)col * block_size, grid->origin_y - (f32)row * block_size,
                   grid->spacing, rows, cols, cast->origin, cast->direction,
                   max(t_in - 1e-4f, 0.0f), t_out + 1e-4f, visit_soil_cell, &cells);
    if (hit) {
        cast->best = cells.best;
        cast->normal = cells.normal;
    }
    return hit;
}

static void cast_ground(const Ray_Scene *scene, v3 origin, v3 direction, f32 max_distance,
                        Ray_Hit *hit)
{
    const Terrain *terrain = scene->terrain;
    Ground_Cast cast = {.grid = terrain,
                        .hole = scene->soil,
                        .origin = origin,
                        .direction = direction,
                        .best = max_distance};
    if (walk_cells(terrain->origin_x, terrain->origin_y, terrain->spacing, terrain->rows - 1,
                   terrain->cols - 1, origin, direction, 0.0f, max_distance, visit_ground_cell,
                   &cast)) {
        hit->distance = cast.best;
        hit->normal = cast.normal;
    }
    const Soil *soil = scene->soil;
    if (!soil) {
        return;
    }
    const Terrain *grid = &soil->grid;
    f32 limit = hit->distance >= 0.0f ? hit->distance : max_distance;
    Ground_Cast soil_cast = {
        .grid = grid, .hole = soil, .origin = origin, .direction = direction, .best = limit};
    f32 block_size = (f32)SOIL_BLOCK_CELLS * grid->spacing;
    if (walk_cells(grid->origin_x, grid->origin_y, block_size, soil->block_rows, soil->block_cols,
                   origin, direction, 0.0f, limit, visit_soil_block, &soil_cast)) {
        hit->distance = soil_cast.best;
        hit->normal = soil_cast.normal;
    }
}

// Clips the ray to the inside of one slab or plane: entering and leaving bounds on t, and
// the normal of the plane that was entered last.
static bool clip_to_plane(v3 normal, f32 offset, v3 origin, v3 direction, f32 *t_enter, f32 *t_exit,
                          v3 *enter_normal)
{
    f32 denominator = dot(normal, direction);
    f32 distance = dot(normal, origin) - offset;
    if (absolute(denominator) < 1e-12f) {
        return distance <= 0.0f;
    }
    f32 t = -distance / denominator;
    if (denominator < 0.0f) {
        if (t > *t_enter) {
            *t_enter = t;
            *enter_normal = normal;
        }
    } else {
        *t_exit = min(*t_exit, t);
    }
    return *t_enter <= *t_exit;
}

// The ray in the solid's own frame against it. t_enter starts negative, so an origin
// inside leaves it negative and counts as no hit.
static bool hit_solid(const Ray_Scene *scene, const Solid *solid, v3 origin, v3 direction,
                      f32 *best, v3 *normal)
{
    Pose pose = get_solid_pose(solid);
    v3 o = inverse_transform_point(pose, origin);
    v3 d = inverse_rotate_vector(pose.q, direction);
    f32 t_enter = -INFINITY;
    f32 t_exit = *best;
    v3 local_normal = {0.0f, 0.0f, 1.0f};
    switch (solid->kind) {
    case SOLID_BOX: {
        const v3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        for (u32 axis = 0; axis < 3; axis++) {
            f32 half = dot(solid->size, axes[axis]);
            if (!clip_to_plane(axes[axis], half, o, d, &t_enter, &t_exit, &local_normal) ||
                !clip_to_plane(-axes[axis], half, o, d, &t_enter, &t_exit, &local_normal)) {
                return false;
            }
        }
        break;
    }
    case SOLID_SPHERE: {
        f32 r = solid->size.x;
        f32 b = dot(o, d);
        f32 c = dot(o, o) - r * r;
        f32 discriminant = b * b - c;
        if (c <= 0.0f || discriminant < 0.0f) {
            return false; // inside, or a miss
        }
        t_enter = -b - sqrtf(discriminant);
        local_normal = (1.0f / r) * (o + t_enter * d);
        break;
    }
    case SOLID_CYLINDER: {
        f32 r = solid->size.x;
        f32 half = solid->size.z;
        if (!clip_to_plane(v3{0.0f, 0.0f, 1.0f}, half, o, d, &t_enter, &t_exit, &local_normal) ||
            !clip_to_plane(v3{0.0f, 0.0f, -1.0f}, half, o, d, &t_enter, &t_exit, &local_normal)) {
            return false;
        }
        f32 a = d.x * d.x + d.y * d.y;
        f32 b = o.x * d.x + o.y * d.y;
        f32 c = o.x * o.x + o.y * o.y - r * r;
        if (a < 1e-12f) {
            if (c > 0.0f) {
                return false; // parallel to the axis, outside it
            }
        } else {
            f32 discriminant = b * b - a * c;
            if (discriminant < 0.0f) {
                return false;
            }
            f32 root = sqrtf(discriminant);
            f32 side_enter = (-b - root) / a;
            f32 side_exit = (-b + root) / a;
            if (side_enter > t_enter) {
                t_enter = side_enter;
                v3 at = o + side_enter * d;
                local_normal = normalize(v3{at.x, at.y, 0.0f});
            }
            t_exit = min(t_exit, side_exit);
        }
        if (t_enter > t_exit) {
            return false;
        }
        break;
    }
    case SOLID_HULL:
        for (u32 p = 0; p < solid->plane_count; p++) {
            const Plane *plane = &scene->planes[solid->first_plane + p];
            if (!clip_to_plane(plane->normal, plane->offset, o, d, &t_enter, &t_exit,
                               &local_normal)) {
                return false;
            }
        }
        break;
    }
    if (t_enter < 0.0f || t_enter >= *best) {
        return false;
    }
    *best = t_enter;
    *normal = rotate_vector(pose.q, local_normal);
    return true;
}

Ray_Hit cast_scene_ray(const Ray_Scene *scene, v3 origin, v3 direction, f32 max_distance,
                       const u32 *solids, u32 solid_count)
{
    Ray_Hit hit = {-1.0f, {0.0f, 0.0f, 1.0f}};
    cast_ground(scene, origin, direction, max_distance, &hit);
    f32 best = hit.distance >= 0.0f ? hit.distance : max_distance;
    u32 count = solids ? solid_count : scene->solid_count;
    for (u32 i = 0; i < count; i++) {
        const Solid *solid = &scene->solids[solids ? solids[i] : i];
        // The bounding sphere first: most solids are nowhere near the ray.
        v3 offset = get_solid_center(solid) - origin;
        f32 along = dot(offset, direction);
        f32 across_squared = dot(offset, offset) - along * along;
        if (across_squared > solid->bound * solid->bound || along + solid->bound < 0.0f ||
            along - solid->bound > best) {
            continue;
        }
        if (hit_solid(scene, solid, origin, direction, &best, &hit.normal)) {
            hit.distance = best;
        }
    }
    return hit;
}
