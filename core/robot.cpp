#include "core/robot.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/math.h"
#include "core/stl.h"

#define DUMMY_MASS 1e-3f // kg; lighter links with no geometry are kinematic dummies
#define DEFAULT_MASS 0.1f // kg for a body with no inertial at all
#define SOIL_DOMAIN_MARGIN 0.01f // m around a body's shapes where soil can touch it

static void set_error(char *error, u32 error_size, const char *format, ...)
    __attribute__((format(printf, 3, 4)));

static void set_error(char *error, u32 error_size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(error, error_size, format, args);
    va_end(args);
}

static f32 wrap_pi(f32 angle)
{
    return angle - 2.0f * PI_F32 * floorf((angle + PI_F32) / (2.0f * PI_F32));
}

static bool is_revolute(Urdf_Joint_Type type)
{
    return type == URDF_JOINT_REVOLUTE || type == URDF_JOINT_CONTINUOUS;
}

// Every link's pose in the root link's frame with all joints at zero.
static Pose get_rest_pose(const Urdf_Model *model, Pose *poses, bool *done, u32 link)
{
    if (!done[link]) {
        Pose pose = identity_pose;
        i32 joint = model->links[link].parent_joint;
        if (joint >= 0) {
            const Urdf_Joint *parent_joint = &model->joints[joint];
            pose = multiply_poses(get_rest_pose(model, poses, done, parent_joint->parent),
                                  parent_joint->origin);
        }
        poses[link] = pose;
        done[link] = true;
    }
    return poses[link];
}

static void compute_rest_poses(const Urdf_Model *model, Pose *poses, bool *done)
{
    memset(done, 0, model->link_count * sizeof(bool));
    for (u32 i = 0; i < model->link_count; i++) {
        get_rest_pose(model, poses, done, i);
    }
}

static u32 get_link_depth(const Urdf_Model *model, u32 link)
{
    u32 depth = 0;
    while (model->links[link].parent_joint >= 0) {
        link = model->joints[model->links[link].parent_joint].parent;
        depth++;
    }
    return depth;
}

static void grow_bounds(v3 *lower, v3 *upper, v3 point)
{
    lower->x = min(lower->x, point.x);
    lower->y = min(lower->y, point.y);
    lower->z = min(lower->z, point.z);
    upper->x = max(upper->x, point.x);
    upper->y = max(upper->y, point.y);
    upper->z = max(upper->z, point.z);
}

// Bounds of one collision shape placed at pose, in the pose's parent frame.
static void get_shape_bounds(const Urdf_Geometry *geometry, Pose pose, v3 *lower, v3 *upper)
{
    switch (geometry->type) {
    case URDF_GEOMETRY_BOX:
        for (u32 corner = 0; corner < 8; corner++) {
            v3 local = {(corner & 1 ? 0.5f : -0.5f) * geometry->size.x,
                        (corner & 2 ? 0.5f : -0.5f) * geometry->size.y,
                        (corner & 4 ? 0.5f : -0.5f) * geometry->size.z};
            grow_bounds(lower, upper, transform_point(pose, local));
        }
        break;
    case URDF_GEOMETRY_CYLINDER: {
        v3 axis = rotate_vector(pose.q, v3{0.0f, 0.0f, 1.0f});
        v3 extent = {geometry->radius * sqrtf(max(0.0f, 1.0f - axis.x * axis.x)),
                     geometry->radius * sqrtf(max(0.0f, 1.0f - axis.y * axis.y)),
                     geometry->radius * sqrtf(max(0.0f, 1.0f - axis.z * axis.z))};
        for (i32 end = -1; end <= 1; end += 2) {
            v3 centre = pose.p + (0.5f * (f32)end * geometry->length) * axis;
            grow_bounds(lower, upper, centre - extent);
            grow_bounds(lower, upper, centre + extent);
        }
        break;
    }
    case URDF_GEOMETRY_SPHERE: {
        v3 extent = {geometry->radius, geometry->radius, geometry->radius};
        grow_bounds(lower, upper, pose.p - extent);
        grow_bounds(lower, upper, pose.p + extent);
        break;
    }
    default:
        grow_bounds(lower, upper, pose.p); // meshes: their origin, without loading them
        break;
    }
}

void get_rest_bounds(const Urdf_Model *model, v3 *lower, v3 *upper)
{
    static Pose poses[4096];
    static bool done[4096];
    *lower = v3{INFINITY, INFINITY, INFINITY};
    *upper = v3{-INFINITY, -INFINITY, -INFINITY};
    if (model->link_count > ARRAY_COUNT(poses)) {
        *lower = *upper = v3{0.0f, 0.0f, 0.0f};
        return;
    }
    compute_rest_poses(model, poses, done);
    for (u32 i = 0; i < model->collision_count; i++) {
        const Urdf_Shape *shape = &model->collisions[i];
        get_shape_bounds(&shape->geometry, multiply_poses(poses[shape->link], shape->origin), lower,
                         upper);
    }
    if (lower->x > upper->x) {
        *lower = *upper = v3{0.0f, 0.0f, 0.0f};
    }
}

static u32 find_root(u32 *parent, u32 x)
{
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

static void join_sets(u32 *parent, u32 a, u32 b)
{
    a = find_root(parent, a);
    b = find_root(parent, b);
    if (a != b) {
        parent[b] = a;
    }
}

static i32 get_only_child_joint(const Urdf_Model *model, u32 link)
{
    i32 found = -1;
    for (u32 i = 0; i < model->joint_count; i++) {
        if (model->joints[i].parent == link) {
            if (found >= 0) {
                return -1;
            }
            found = (i32)i;
        }
    }
    return found;
}

// No geometry, next to no mass, one child joint, not welded to anything: a link that exists
// only so URDF can chain joints.
static bool is_dummy_link(const Urdf_Model *model, u32 link, const bool *closure_link)
{
    const Urdf_Link *l = &model->links[link];
    return l->visual_count == 0 && l->collision_count == 0 && !closure_link[link] &&
           (!l->inertial.present || l->inertial.mass < DUMMY_MASS) &&
           get_only_child_joint(model, link) >= 0;
}

static bool is_pure_twist(Pose origin)
{
    return get_length(origin.p) < 1e-5f && get_length(origin.q.v) < 1e-4f;
}

// Joint j1 starts a ball joint if j1, j2 and j3 are revolute, chained through two dummy
// links, share one pivot, and have mutually perpendicular axes.
static bool find_ball_chain(const Urdf_Model *model, u32 j1, const bool *closure_link, u32 *chain)
{
    const Urdf_Joint *joints = model->joints;
    if (!is_revolute(joints[j1].type) || !is_dummy_link(model, joints[j1].child, closure_link)) {
        return false;
    }
    i32 j2 = get_only_child_joint(model, joints[j1].child);
    if (!is_revolute(joints[j2].type) || !is_pure_twist(joints[j2].origin) ||
        !is_dummy_link(model, joints[j2].child, closure_link)) {
        return false;
    }
    i32 j3 = get_only_child_joint(model, joints[j2].child);
    if (!is_revolute(joints[j3].type) || !is_pure_twist(joints[j3].origin)) {
        return false;
    }
    v3 u1 = joints[j1].axis;
    v3 u2 = joints[j2].axis;
    v3 u3 = joints[j3].axis;
    if (absolute(dot(u1, u2)) > 1e-3f || absolute(dot(u1, u3)) > 1e-3f ||
        absolute(dot(u2, u3)) > 1e-3f) {
        return false;
    }
    chain[0] = j1;
    chain[1] = (u32)j2;
    chain[2] = (u32)j3;
    return true;
}

// Physically possible inertia: positive definite, and each principal moment no larger
// than the sum of the other two (true of the diagonal in any frame).
static bool is_inertia_plausible(Mat3 inertia)
{
    f32 ixx = inertia.cx.x, iyy = inertia.cy.y, izz = inertia.cz.z;
    f32 ixy = inertia.cy.x;
    f32 slack = 1e-6f * (ixx + iyy + izz);
    bool positive = ixx > 0.0f && ixx * iyy - ixy * ixy > 0.0f && get_determinant(inertia) > 0.0f;
    bool triangle =
        ixx + iyy + slack >= izz && ixx + izz + slack >= iyy && iyy + izz + slack >= ixx;
    return positive && triangle;
}

static Mat3 make_diagonal(f32 value)
{
    Mat3 m = {};
    m.cx.x = m.cy.y = m.cz.z = value;
    return m;
}

struct Mass_Data {
    f32 mass;
    v3 center; // in the body frame
    Mat3 inertia; // about the center, in the body's axes
};

static void set_robot_body_mass(Robot *robot, u32 body, const u32 *members, u32 member_count)
{
    const Urdf_Model *model = &robot->model;
    f32 mass = 0.0f;
    v3 weighted = {0.0f, 0.0f, 0.0f};
    for (u32 m = 0; m < member_count; m++) {
        const Urdf_Inertial *inertial = &model->links[members[m]].inertial;
        if (!inertial->present || inertial->mass <= 0.0f) {
            continue;
        }
        v3 centre = transform_point(robot->link_in_body[members[m]], inertial->origin.p);
        mass += inertial->mass;
        weighted += inertial->mass * centre;
    }

    const char *name = model->links[robot->bodies[body].frame_link].name;
    Mass_Data data = {};
    if (mass <= 0.0f) {
        log_warning("robot: %s has no mass in the URDF; using %.1f kg", name, (f64)DEFAULT_MASS);
        data.mass = DEFAULT_MASS;
        data.inertia = make_diagonal(1e-3f);
    } else {
        data.mass = mass;
        data.center = (1.0f / mass) * weighted;
        for (u32 m = 0; m < member_count; m++) {
            const Urdf_Inertial *inertial = &model->links[members[m]].inertial;
            if (!inertial->present || inertial->mass <= 0.0f) {
                continue;
            }
            Pose frame = multiply_poses(robot->link_in_body[members[m]], inertial->origin);
            Mat3 rotation = make_matrix_from_quat(frame.q);
            Mat3 rotated = multiply_matrices(multiply_matrices(rotation, inertial->inertia),
                                             transpose_matrix(rotation));
            v3 offset = frame.p - data.center;
            data.inertia = add_matrices(
                data.inertia, add_matrices(rotated, get_offset_inertia(inertial->mass, offset)));
        }
        if (!is_inertia_plausible(data.inertia)) {
            f32 mean = (absolute(data.inertia.cx.x) + absolute(data.inertia.cy.y) +
                        absolute(data.inertia.cz.z)) /
                       3.0f;
            log_warning(
                "robot: %s's inertia is not physically possible; using %g kg·m² on each axis", name,
                (f64)max(mean, 1e-6f));
            data.inertia = make_diagonal(max(mean, 1e-6f));
        }
    }
    robot->bodies[body].mass = data.mass;
    set_body_mass(robot->physics, robot->bodies[body].physics_body, data.mass, data.center,
                  data.inertia);
}

// A collision shape's corners and extremes in its body's frame, for sizing the body's soil
// domain.
static void grow_body_bounds(v3 *lower, v3 *upper, Pose pose, const Urdf_Geometry *geometry,
                             const v3 *points, u32 point_count)
{
    if (geometry->type == URDF_GEOMETRY_MESH) {
        for (u32 i = 0; i < point_count; i++) {
            grow_bounds(lower, upper, points[i]);
        }
    } else {
        get_shape_bounds(geometry, pose, lower, upper);
    }
}

// Adds a collision shape to the physics and the ray scene, and grows the body's bounds.
static void add_collision_shape(Robot *robot, Ray_Scene *scene, u32 body, const Urdf_Shape *shape,
                                v3 *lower, v3 *upper)
{
    const Urdf_Geometry *geometry = &shape->geometry;
    Pose pose = multiply_poses(robot->link_in_body[shape->link], shape->origin);
    Robot_Body *b = &robot->bodies[body];
    u32 id = b->physics_body;
    Shape_Material material = {
        .friction = robot->settings.friction,
        .group = robot->model.self_collide ? 0 : robot->settings.collision_group,
    };

    switch (geometry->type) {
    case URDF_GEOMETRY_BOX: {
        v3 half = 0.5f * geometry->size;
        add_box_shape(robot->physics, id, pose, half, &material);
        add_ray_box(scene, &b->current, pose, half);
        break;
    }
    case URDF_GEOMETRY_CYLINDER:
        // URDF's cylinder is centred on its origin along z, as the physics' is.
        add_cylinder_shape(robot->physics, id, pose, geometry->radius, geometry->length, &material);
        add_ray_cylinder(scene, &b->current, pose, geometry->radius, geometry->length);
        break;
    case URDF_GEOMETRY_SPHERE:
        add_sphere_shape(robot->physics, id, pose.p, geometry->radius, &material);
        add_ray_sphere(scene, &b->current, pose.p, geometry->radius);
        break;
    case URDF_GEOMETRY_MESH: {
        char path[URDF_PATH_SIZE];
        char message[256];
        Stl_Mesh mesh;
        if (!resolve_resource_path(geometry->mesh, robot->settings.resource_dir, path,
                                   sizeof(path))) {
            log_warning("robot: collision mesh %s not found; shape skipped", geometry->mesh);
            return;
        }
        if (!load_stl(&mesh, path, message, sizeof(message))) {
            log_warning("robot: %s; shape skipped", message);
            return;
        }
        u32 count = mesh.triangle_count * 3;
        v3 *points = (v3 *)mesh.normals; // reuse: same size, no longer needed
        for (u32 i = 0; i < count; i++) {
            v3 local = {mesh.positions[i * 3] * geometry->scale.x,
                        mesh.positions[i * 3 + 1] * geometry->scale.y,
                        mesh.positions[i * 3 + 2] * geometry->scale.z};
            points[i] = transform_point(pose, local);
        }
        // A hull of n points has at most 2n - 4 triangles.
        u32 capacity = 6 * count;
        v3 *hull = (v3 *)malloc(capacity * sizeof(v3));
        u32 hull_count = hull ? make_convex_hull(points, count, hull, capacity) : 0;
        if (hull_count > 0) {
            // The hull's corners are all the physics needs of the mesh.
            add_hull_shape(robot->physics, id, hull, hull_count, &material);
            add_ray_hull(scene, &b->current, hull, hull_count);
            grow_body_bounds(lower, upper, pose, geometry, hull, hull_count);
        } else {
            log_warning("robot: could not make a convex hull from %s", path);
        }
        free(hull);
        free_stl(&mesh);
        return;
    }
    default:
        return;
    }
    grow_body_bounds(lower, upper, pose, geometry, NULL, 0);
}

// The most a joint's motor applies in a drive mode: the full motor for velocity and position
// control, and otherwise just the URDF's joint friction, as a brake.
static f32 get_motor_limit(const Robot *robot, const Robot_Joint *joint, Joint_Drive drive)
{
    if (drive == DRIVE_VELOCITY || drive == DRIVE_POSITION) {
        return robot->settings.motor_torque;
    }
    return robot->model.joints[joint->urdf_joint].friction;
}

static bool create_joint(Robot *robot, Robot_Joint *joint, char *error, u32 error_size)
{
    const Urdf_Model *model = &robot->model;
    const Urdf_Joint *urdf = &model->joints[joint->urdf_joint];
    if (urdf->type == URDF_JOINT_FLOATING || urdf->type == URDF_JOINT_PLANAR) {
        set_error(error, error_size, "joint \"%s\": %s joints are not supported", urdf->name,
                  urdf->type == URDF_JOINT_FLOATING ? "floating" : "planar");
        return false;
    }
    joint->body_a = (u32)robot->link_body[urdf->parent];
    joint->body_b = (u32)robot->link_body[urdf->child];
    if (joint->body_a == joint->body_b) {
        log_warning(
            "robot: joint %s joins two links of one rigid body (after loop closure); ignored",
            urdf->name);
        return true;
    }

    // Revolute joints turn about the frame's z; prismatic joints slide along its x.
    v3 reference = urdf->type == URDF_JOINT_PRISMATIC ? v3{1.0f, 0.0f, 0.0f} : v3{0.0f, 0.0f, 1.0f};
    Pose axis_frame = {{0.0f, 0.0f, 0.0f}, make_quat_between(reference, urdf->axis)};
    joint->frame_a =
        multiply_poses(robot->link_in_body[urdf->parent], multiply_poses(urdf->origin, axis_frame));
    joint->frame_b = multiply_poses(robot->link_in_body[urdf->child], axis_frame);

    Joint_Def def = {
        .kind = urdf->type == URDF_JOINT_PRISMATIC ? JOINT_PRISMATIC : JOINT_REVOLUTE,
        .body_a = robot->bodies[joint->body_a].physics_body,
        .body_b = robot->bodies[joint->body_b].physics_body,
        .frame_a = joint->frame_a,
        .frame_b = joint->frame_b,
        .limited = false,
        .lower = urdf->lower,
        .upper = urdf->upper,
    };
    if (urdf->type == URDF_JOINT_PRISMATIC) {
        def.limited = urdf->has_limits && urdf->lower < urdf->upper;
    } else {
        // A range of a full turn or more is no limit at all, and a stop at ±π would jam.
        def.limited = urdf->type == URDF_JOINT_REVOLUTE && urdf->has_limits &&
                      urdf->upper - urdf->lower < 2.0f * PI_F32 - 0.01f;
    }
    joint->physics_joint = (i32)add_joint(robot->physics, &def);
    return true;
}

static void create_ball(Robot *robot, Robot_Ball *ball, const u32 *chain, const Pose *rest)
{
    const Urdf_Model *model = &robot->model;
    const Urdf_Joint *first = &model->joints[chain[0]];
    u32 parent = first->parent;
    u32 child = model->joints[chain[2]].child;
    ball->body_a = (u32)robot->link_body[parent];
    ball->body_b = (u32)robot->link_body[child];
    ball->frame_a = multiply_poses(robot->link_in_body[parent], first->origin);
    Pose pivot = multiply_poses(rest[parent], first->origin);
    Pose body_b_rest = rest[robot->bodies[ball->body_b].frame_link];
    ball->frame_b = inverse_multiply_poses(body_b_rest, pivot);

    v3 u1 = first->axis;
    v3 u2 = model->joints[chain[1]].axis;
    v3 u3 = cross(u1, u2);
    ball->basis.cx = u1;
    ball->basis.cy = u2;
    ball->basis.cz = u3;
    ball->third_sign = dot(u3, model->joints[chain[2]].axis) >= 0.0f ? 1.0f : -1.0f;

    Joint_Def def = {
        .kind = JOINT_SPHERICAL,
        .body_a = robot->bodies[ball->body_a].physics_body,
        .body_b = robot->bodies[ball->body_b].physics_body,
        .frame_a = ball->frame_a,
        .frame_b = ball->frame_b,
    };
    ball->physics_joint = add_joint(robot->physics, &def);
}

static f32 get_wheel_radius(const Urdf_Model *model, u32 link)
{
    f32 radius = 0.0f;
    const Urdf_Link *l = &model->links[link];
    for (u32 i = 0; i < l->collision_count; i++) {
        const Urdf_Geometry *geometry = &model->collisions[l->first_collision + i].geometry;
        if (geometry->type == URDF_GEOMETRY_CYLINDER || geometry->type == URDF_GEOMETRY_SPHERE) {
            radius = max(radius, geometry->radius);
        }
    }
    return radius;
}

static void find_wheels(Robot *robot, const Pose *rest)
{
    const Urdf_Model *model = &robot->model;
    for (u32 i = 0; i < robot->joint_count; i++) {
        Robot_Joint *joint = &robot->joints[i];
        const Urdf_Joint *urdf = &model->joints[joint->urdf_joint];
        if (joint->drive != DRIVE_VELOCITY || urdf->type != URDF_JOINT_CONTINUOUS) {
            continue;
        }
        f32 radius = get_wheel_radius(model, urdf->child);
        if (radius <= 0.0f) {
            log_warning("robot: drive wheel %s has no cylinder or sphere collision; assuming 0.1 m",
                        urdf->name);
            radius = 0.1f;
        }
        // The joint axis is in the child frame. Positive spin rolls forward (+x) when the
        // contact point below the axle moves backwards: (axis × z) · x > 0.
        v3 axis = rotate_vector(rest[urdf->child].q, urdf->axis);
        f32 forward = cross(axis, v3{0.0f, 0.0f, 1.0f}).x;
        Robot_Wheel wheel = {
            .joint = i,
            .forward = rest[urdf->child].p.x,
            .lateral = rest[urdf->child].p.y,
            .radius = radius,
            .forward_sign = forward >= 0.0f ? 1.0f : -1.0f,
            .steer_joint = -1,
            .steer_sign = 1.0f,
        };
        // A steered wheel hangs from a position-controlled joint about a roughly vertical
        // axis, one link up (wheel -> knuckle -> steering joint).
        i32 steer = model->links[urdf->parent].parent_joint;
        if (steer >= 0 && robot->urdf_joint_index[steer] >= 0) {
            const Urdf_Joint *steer_urdf = &model->joints[steer];
            v3 steer_axis = rotate_vector(rest[steer_urdf->child].q, steer_urdf->axis);
            if (robot->joints[robot->urdf_joint_index[steer]].drive == DRIVE_POSITION &&
                absolute(steer_axis.z) > 0.7f) {
                wheel.steer_joint = robot->urdf_joint_index[steer];
                wheel.steer_sign = steer_axis.z > 0.0f ? 1.0f : -1.0f;
            }
        }
        robot->wheels[robot->wheel_count++] = wheel;
    }
}

bool create_robot(Robot *robot, Physics *physics, Ray_Scene *scene, Linear_Allocator *allocator,
                  const Urdf_Model *model, const Robot_Settings *settings, Pose spawn,
                  f32 step_seconds, char *error, u32 error_size)
{
    *robot = {};
    robot->model = *model;
    robot->settings = *settings;
    robot->physics = physics;
    robot->step_seconds = step_seconds;
    u32 links = model->link_count;
    u32 joints = model->joint_count;

    Pose *rest = ALLOCATE_ARRAY(allocator, Pose, links);
    bool *done = ALLOCATE_ARRAY(allocator, bool, links);
    bool *closure_link = ALLOCATE_ARRAY(allocator, bool, links);
    bool *dummy = ALLOCATE_ARRAY(allocator, bool, links);
    u32 *set = ALLOCATE_ARRAY(allocator, u32, links);
    u32 *members = ALLOCATE_ARRAY(allocator, u32, links);
    i32 *ball_of_joint = ALLOCATE_ARRAY(allocator, i32, max(joints, 1u));
    u32 *chains = ALLOCATE_ARRAY(allocator, u32, max(joints, 1u) * 3);
    robot->bodies = ALLOCATE_ARRAY(allocator, Robot_Body, links);
    robot->link_body = ALLOCATE_ARRAY(allocator, i32, links);
    robot->link_in_body = ALLOCATE_ARRAY(allocator, Pose, links);
    robot->joints = ALLOCATE_ARRAY(allocator, Robot_Joint, max(joints, 1u));
    robot->urdf_joint_index = ALLOCATE_ARRAY(allocator, i32, max(joints, 1u));
    robot->balls = ALLOCATE_ARRAY(allocator, Robot_Ball, max(joints / 3, 1u));
    robot->wheels = ALLOCATE_ARRAY(allocator, Robot_Wheel, max(joints, 1u));
    if (!rest || !done || !closure_link || !dummy || !set || !members || !ball_of_joint ||
        !chains || !robot->bodies || !robot->link_body || !robot->link_in_body || !robot->joints ||
        !robot->urdf_joint_index || !robot->balls || !robot->wheels) {
        set_error(error, error_size, "out of memory building the robot");
        return false;
    }
    compute_rest_poses(model, rest, done);

    // Ball joints: find the chains and retire their dummy links.
    for (u32 i = 0; i < model->closure_count; i++) {
        closure_link[model->closures[i].parent_link] = true;
        closure_link[model->closures[i].child_link] = true;
    }
    for (u32 i = 0; i < joints; i++) {
        ball_of_joint[i] = -1;
    }
    for (u32 j = 0; j < joints; j++) {
        u32 *chain = &chains[robot->ball_count * 3];
        if (ball_of_joint[j] >= 0 || !find_ball_chain(model, j, closure_link, chain)) {
            continue;
        }
        for (u32 k = 0; k < 3; k++) {
            ball_of_joint[chain[k]] = (i32)robot->ball_count;
        }
        dummy[model->joints[chain[0]].child] = true;
        dummy[model->joints[chain[1]].child] = true;
        robot->ball_count++;
    }

    // Bodies: links joined by fixed joints or loop closures move as one.
    for (u32 i = 0; i < links; i++) {
        set[i] = i;
        robot->link_body[i] = -1;
    }
    for (u32 j = 0; j < joints; j++) {
        const Urdf_Joint *joint = &model->joints[j];
        if (joint->type == URDF_JOINT_FIXED && !dummy[joint->parent] && !dummy[joint->child]) {
            join_sets(set, joint->parent, joint->child);
        }
    }
    for (u32 i = 0; i < model->closure_count; i++) {
        join_sets(set, model->closures[i].parent_link, model->closures[i].child_link);
    }
    for (u32 i = 0; i < links; i++) {
        if (dummy[i] || robot->link_body[find_root(set, i)] >= 0) {
            continue;
        }
        // A new set: its frame is the member nearest the root.
        u32 root = find_root(set, i);
        u32 frame = i;
        for (u32 k = 0; k < links; k++) {
            if (!dummy[k] && find_root(set, k) == root &&
                get_link_depth(model, k) < get_link_depth(model, frame)) {
                frame = k;
            }
        }
        u32 body = robot->body_count++;
        robot->link_body[root] = (i32)body;
        robot->bodies[body].frame_link = frame;
    }
    for (u32 i = 0; i < links; i++) {
        if (!dummy[i]) {
            robot->link_body[i] = robot->link_body[find_root(set, i)];
            u32 frame = robot->bodies[robot->link_body[i]].frame_link;
            robot->link_in_body[i] = inverse_multiply_poses(rest[frame], rest[i]);
        }
    }

    for (u32 b = 0; b < robot->body_count; b++) {
        Robot_Body *body = &robot->bodies[b];
        Pose pose = multiply_poses(spawn, rest[body->frame_link]);
        body->physics_body = add_body(robot->physics, pose, false);
        body->current = body->previous = body->start = pose;

        u32 member_count = 0;
        for (u32 i = 0; i < links; i++) {
            if (robot->link_body[i] == (i32)b) {
                members[member_count++] = i;
            }
        }
        v3 lower = {INFINITY, INFINITY, INFINITY};
        v3 upper = {-INFINITY, -INFINITY, -INFINITY};
        for (u32 m = 0; m < member_count; m++) {
            const Urdf_Link *link = &model->links[members[m]];
            for (u32 c = 0; c < link->collision_count; c++) {
                add_collision_shape(robot, scene, b, &model->collisions[link->first_collision + c],
                                    &lower, &upper);
            }
        }
        // Mass before joints: the physics attaches joints relative to the centre of mass.
        set_robot_body_mass(robot, b, members, member_count);
        if (lower.x <= upper.x) {
            // The soil only feels a body within this box around its shapes.
            v3 margin = {SOIL_DOMAIN_MARGIN, SOIL_DOMAIN_MARGIN, SOIL_DOMAIN_MARGIN};
            add_soil_domain(robot->physics, body->physics_body, 0.5f * (lower + upper),
                            upper - lower + 2.0f * margin);
        }
    }

    // Joints: every movable URDF joint gets a Robot_Joint; balls get one spherical joint.
    for (u32 j = 0; j < joints; j++) {
        const Urdf_Joint *urdf = &model->joints[j];
        robot->urdf_joint_index[j] = -1;
        if (urdf->type == URDF_JOINT_FIXED) {
            continue;
        }
        u32 index = robot->joint_count++;
        robot->urdf_joint_index[j] = (i32)index;
        Robot_Joint *joint = &robot->joints[index];
        *joint = {};
        joint->physics_joint = -1;
        joint->urdf_joint = j;
        joint->ball = ball_of_joint[j];
        if (urdf->actuated) {
            joint->drive = urdf->position_command ? DRIVE_POSITION : DRIVE_VELOCITY;
        }
        if (joint->ball >= 0) {
            if (joint->drive != DRIVE_NONE) {
                log_warning("robot: %s is part of a ball joint and can't be driven", urdf->name);
            }
            joint->drive = DRIVE_NONE;
            continue;
        }
        joint->default_drive = joint->drive;
        if (!create_joint(robot, joint, error, error_size)) {
            destroy_robot(robot);
            return false;
        }
    }
    for (u32 b = 0; b < robot->ball_count; b++) {
        const u32 *chain = &chains[b * 3];
        Robot_Ball *ball = &robot->balls[b];
        create_ball(robot, ball, chain, rest);
        for (u32 k = 0; k < 3; k++) {
            ball->joints[k] = (u32)robot->urdf_joint_index[chain[k]];
            robot->joints[ball->joints[k]].ball_axis = k;
        }
    }
    find_wheels(robot, rest);
    return true;
}

void destroy_robot(Robot *robot)
{
    // The physics has no way to take bodies out; they go with the world.
    robot->body_count = 0;
    robot->joint_count = 0;
    robot->ball_count = 0;
    robot->wheel_count = 0;
}

void apply_robot_commands(Robot *robot)
{
    for (u32 i = 0; i < robot->joint_count; i++) {
        Robot_Joint *joint = &robot->joints[i];
        if (joint->physics_joint < 0) {
            continue;
        }
        const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
        u32 id = (u32)joint->physics_joint;
        f32 limit = get_motor_limit(robot, joint, joint->drive);
        switch (joint->drive) {
        case DRIVE_NONE:
            // Coasting still feels the URDF's joint friction, as an undriven motor does.
            drive_joint(robot->physics, id, limit > 0.0f ? MOTOR_SPEED : MOTOR_OFF, 0.0f, limit);
            break;
        case DRIVE_EFFORT:
            drive_joint(robot->physics, id, MOTOR_EFFORT, joint->command, 0.0f);
            break;
        case DRIVE_VELOCITY:
            drive_joint(robot->physics, id, MOTOR_SPEED, joint->command, limit);
            break;
        case DRIVE_POSITION: {
            // A servo that heads for its target, within the joint's limits, at servo_speed
            // (and arrives exactly, without overshoot).
            f32 target = joint->command;
            if (urdf->has_limits && urdf->lower < urdf->upper) {
                target = min(max(target, urdf->lower), urdf->upper);
            }
            f32 speed = (target - joint->position) / robot->step_seconds;
            f32 servo = robot->settings.servo_speed;
            if (servo > 0.0f) {
                speed = min(max(speed, -servo), servo);
            }
            drive_joint(robot->physics, id, MOTOR_SPEED, speed, limit);
            break;
        }
        }
    }
}

void set_joint_drive(Robot *robot, u32 joint_index, Joint_Drive drive)
{
    Robot_Joint *joint = &robot->joints[joint_index];
    if (joint->drive == drive || joint->physics_joint < 0) {
        return;
    }
    joint->drive = drive;
    if (drive == DRIVE_NONE) {
        joint->command = 0.0f;
    }
}

// Angles (a, b, c) with m = Rx(a) Ry(b) Rz(c).
static v3 get_angles_xyz(Mat3 m)
{
    f32 sin_b = min(max(m.cz.x, -1.0f), 1.0f);
    return v3{atan2f(-m.cz.y, m.cz.z), asinf(sin_b), atan2f(-m.cy.x, m.cx.x)};
}

// A joint's angle from its bodies' poses. The position keeps counting past a turn, and
// the velocity comes from the change since the last step.
static void update_joint_angle(Robot *robot, u32 joint_index, f32 angle)
{
    Robot_Joint *joint = &robot->joints[joint_index];
    f32 delta = wrap_pi(angle - wrap_pi(joint->position));
    joint->velocity = delta / robot->step_seconds;
    joint->position += delta;
}

static void read_ball(Robot *robot, Robot_Ball *ball)
{
    Pose body_a = robot->bodies[ball->body_a].current;
    Pose body_b = robot->bodies[ball->body_b].current;
    Quat frame_a = multiply_quats(body_a.q, ball->frame_a.q);
    Quat frame_b = multiply_quats(body_b.q, ball->frame_b.q);
    Mat3 relative = make_matrix_from_quat(inverse_multiply_quats(frame_a, frame_b));
    Mat3 in_basis =
        multiply_matrices(multiply_matrices(transpose_matrix(ball->basis), relative), ball->basis);
    v3 angles = get_angles_xyz(in_basis);
    update_joint_angle(robot, ball->joints[0], angles.x);
    update_joint_angle(robot, ball->joints[1], angles.y);
    update_joint_angle(robot, ball->joints[2], ball->third_sign * angles.z);
}

// A revolute or prismatic joint's position and rate, from its bodies' poses and velocities.
static void read_joint(Robot *robot, Robot_Joint *joint)
{
    const Robot_Body *body_a = &robot->bodies[joint->body_a];
    const Robot_Body *body_b = &robot->bodies[joint->body_b];
    Pose frame_a = multiply_poses(body_a->current, joint->frame_a);
    Pose frame_b = multiply_poses(body_b->current, joint->frame_b);
    const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
    if (urdf->type == URDF_JOINT_PRISMATIC) {
        v3 axis = rotate_vector(frame_a.q, v3{1.0f, 0.0f, 0.0f});
        joint->position = inverse_transform_point(frame_a, frame_b.p).x;
        v3 relative = get_point_velocity(robot->physics, body_b->physics_body, frame_b.p) -
                      get_point_velocity(robot->physics, body_a->physics_body, frame_b.p);
        joint->velocity = dot(relative, axis);
        return;
    }
    // Keep counting past ±π, as joint states do for continuous joints.
    f32 angle = get_twist_angle(inverse_multiply_quats(frame_a.q, frame_b.q));
    joint->position += wrap_pi(angle - wrap_pi(joint->position));
    v3 axis = rotate_vector(frame_a.q, v3{0.0f, 0.0f, 1.0f});
    v3 relative = get_body_angular_velocity(robot->physics, body_b->physics_body) -
                  get_body_angular_velocity(robot->physics, body_a->physics_body);
    joint->velocity = dot(relative, axis);
}

void read_robot_state(Robot *robot)
{
    for (u32 b = 0; b < robot->body_count; b++) {
        Robot_Body *body = &robot->bodies[b];
        body->previous = body->current;
        body->current = get_body_pose(robot->physics, body->physics_body);
    }
    for (u32 i = 0; i < robot->joint_count; i++) {
        Robot_Joint *joint = &robot->joints[i];
        if (joint->physics_joint < 0) {
            continue;
        }
        read_joint(robot, joint);
        joint->effort = joint->drive == DRIVE_EFFORT
                            ? joint->command
                            : get_joint_effort(robot->physics, (u32)joint->physics_joint);
    }
    for (u32 b = 0; b < robot->ball_count; b++) {
        read_ball(robot, &robot->balls[b]);
    }
}

void reset_robot(Robot *robot)
{
    for (u32 b = 0; b < robot->body_count; b++) {
        Robot_Body *body = &robot->bodies[b];
        set_body_pose(robot->physics, body->physics_body, body->start);
        set_body_velocity(robot->physics, body->physics_body, v3{0.0f, 0.0f, 0.0f},
                          v3{0.0f, 0.0f, 0.0f});
        body->current = body->previous = body->start;
    }
    for (u32 i = 0; i < robot->joint_count; i++) {
        Robot_Joint *joint = &robot->joints[i];
        joint->position = 0.0f;
        joint->velocity = 0.0f;
        joint->effort = 0.0f;
        joint->command = 0.0f;
        if (joint->ball < 0) {
            set_joint_drive(robot, i, joint->default_drive);
        }
    }
}

void steer_robot(Robot *robot, f32 drive, f32 turn, f32 max_speed, f32 turn_rate)
{
    f32 forward = drive * max_speed;
    f32 yaw_rate = (drive < 0.0f ? -turn : turn) * turn_rate;
    // The fastest wheel over the ground, for this forward speed and yaw rate.
    f32 fastest = 0.0f;
    for (u32 w = 0; w < robot->wheel_count; w++) {
        const Robot_Wheel *wheel = &robot->wheels[w];
        f32 along = forward - yaw_rate * wheel->lateral;
        f32 across = wheel->steer_joint >= 0 ? yaw_rate * wheel->forward : 0.0f;
        fastest = max(fastest, sqrtf(along * along + across * across));
    }
    f32 scale = fastest > max_speed ? max_speed / fastest : 1.0f;
    drive_robot(robot, scale * forward, scale * yaw_rate);
}

void drive_robot(Robot *robot, f32 forward_mps, f32 turn_radps)
{
    for (u32 w = 0; w < robot->wheel_count; w++) {
        const Robot_Wheel *wheel = &robot->wheels[w];
        // Take the joints back from whatever mode a controller left them in.
        set_joint_drive(robot, wheel->joint, robot->joints[wheel->joint].default_drive);
        if (wheel->steer_joint >= 0) {
            u32 steer = (u32)wheel->steer_joint;
            set_joint_drive(robot, steer, robot->joints[steer].default_drive);
        }
        // The ground velocity this wheel needs for the robot to move as asked.
        f32 along = forward_mps - turn_radps * wheel->lateral;
        f32 across = turn_radps * wheel->forward;
        f32 speed = along;
        if (wheel->steer_joint >= 0) {
            speed = sqrtf(along * along + across * across);
            if (speed > 1e-4f) {
                // Point the wheel along it; beyond ±90° drive backwards instead.
                f32 angle = atan2f(across, along);
                if (angle > 0.5f * PI_F32) {
                    angle -= PI_F32;
                    speed = -speed;
                } else if (angle < -0.5f * PI_F32) {
                    angle += PI_F32;
                    speed = -speed;
                }
                robot->joints[wheel->steer_joint].command = wheel->steer_sign * angle;
                // Until the pivot gets there, drive only the part of the speed along the
                // way the wheel points now, so it doesn't scrub sideways mid-swing.
                f32 pointing = wheel->steer_sign * robot->joints[wheel->steer_joint].position;
                speed *= max(cosf(angle - pointing), 0.0f);
            }
        }
        robot->joints[wheel->joint].command = wheel->forward_sign * speed / wheel->radius;
    }
}

Pose get_link_pose(const Robot *robot, u32 link, f32 alpha)
{
    i32 body = robot->link_body[link];
    if (body < 0) {
        return identity_pose;
    }
    const Robot_Body *b = &robot->bodies[body];
    Pose pose;
    pose.p = lerp(b->previous.p, b->current.p, alpha);
    pose.q = nlerp_quats(b->previous.q, b->current.q, alpha);
    return multiply_poses(pose, robot->link_in_body[link]);
}

i32 find_robot_joint(const Robot *robot, const char *name)
{
    i32 urdf = find_urdf_joint(&robot->model, name);
    return urdf >= 0 ? robot->urdf_joint_index[urdf] : -1;
}

void nudge_robot_joint(Robot *robot, u32 joint_index, f32 direction, f32 rate, f32 seconds)
{
    Robot_Joint *joint = &robot->joints[joint_index];
    if (joint->default_drive != DRIVE_POSITION) {
        return;
    }
    set_joint_drive(robot, joint_index, DRIVE_POSITION);
    f32 target = joint->command + direction * rate * seconds;
    const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
    if (urdf->has_limits && urdf->lower < urdf->upper) {
        target = min(max(target, urdf->lower), urdf->upper);
    }
    joint->command = target;
}
