#include "core/robot.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/math.h"
#include "core/stl.h"

#define DUMMY_MASS 1e-3f // kg; lighter links with no geometry are kinematic dummies
#define CYLINDER_SIDES 32 // Box3D's maximum for a cylinder hull
#define MESH_HULL_VERTICES 64
#define DEFAULT_MASS 0.1f // kg for a body with no inertial at all

static const b3Transform identity_transform = {{0.0f, 0.0f, 0.0f}, {{0.0f, 0.0f, 0.0f}, 1.0f}};

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
static b3Transform get_rest_pose(const Urdf_Model *model, b3Transform *poses, bool *done, u32 link)
{
    if (!done[link]) {
        b3Transform pose = identity_transform;
        i32 joint = model->links[link].parent_joint;
        if (joint >= 0) {
            const Urdf_Joint *parent_joint = &model->joints[joint];
            pose = b3MulTransforms(get_rest_pose(model, poses, done, parent_joint->parent),
                                   parent_joint->origin);
        }
        poses[link] = pose;
        done[link] = true;
    }
    return poses[link];
}

static void compute_rest_poses(const Urdf_Model *model, b3Transform *poses, bool *done)
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
static void get_shape_bounds(const Urdf_Geometry *geometry, b3Transform pose, v3 *lower, v3 *upper)
{
    switch (geometry->type) {
    case URDF_GEOMETRY_BOX:
        for (u32 corner = 0; corner < 8; corner++) {
            v3 local = {(corner & 1 ? 0.5f : -0.5f) * geometry->size.x,
                        (corner & 2 ? 0.5f : -0.5f) * geometry->size.y,
                        (corner & 4 ? 0.5f : -0.5f) * geometry->size.z};
            grow_bounds(lower, upper, b3TransformPoint(pose, local));
        }
        break;
    case URDF_GEOMETRY_CYLINDER: {
        v3 axis = b3RotateVector(pose.q, v3{0.0f, 0.0f, 1.0f});
        v3 extent = {geometry->radius * sqrtf(max(0.0f, 1.0f - axis.x * axis.x)),
                     geometry->radius * sqrtf(max(0.0f, 1.0f - axis.y * axis.y)),
                     geometry->radius * sqrtf(max(0.0f, 1.0f - axis.z * axis.z))};
        for (i32 end = -1; end <= 1; end += 2) {
            v3 centre = b3MulAdd(pose.p, 0.5f * (f32)end * geometry->length, axis);
            grow_bounds(lower, upper, b3Sub(centre, extent));
            grow_bounds(lower, upper, b3Add(centre, extent));
        }
        break;
    }
    case URDF_GEOMETRY_SPHERE: {
        v3 extent = {geometry->radius, geometry->radius, geometry->radius};
        grow_bounds(lower, upper, b3Sub(pose.p, extent));
        grow_bounds(lower, upper, b3Add(pose.p, extent));
        break;
    }
    default:
        grow_bounds(lower, upper, pose.p); // meshes: their origin, without loading them
        break;
    }
}

void get_rest_bounds(const Urdf_Model *model, v3 *lower, v3 *upper)
{
    static b3Transform poses[4096];
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
        get_shape_bounds(&shape->geometry, b3MulTransforms(poses[shape->link], shape->origin),
                         lower, upper);
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

static bool is_pure_twist(b3Transform origin)
{
    return b3Length(origin.p) < 1e-5f && b3Length(origin.q.v) < 1e-4f;
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
    if (absolute(b3Dot(u1, u2)) > 1e-3f || absolute(b3Dot(u1, u3)) > 1e-3f ||
        absolute(b3Dot(u2, u3)) > 1e-3f) {
        return false;
    }
    chain[0] = j1;
    chain[1] = (u32)j2;
    chain[2] = (u32)j3;
    return true;
}

// Physically possible inertia: positive definite, and each principal moment no larger
// than the sum of the other two (true of the diagonal in any frame).
static bool is_inertia_plausible(b3Matrix3 inertia)
{
    f32 ixx = inertia.cx.x, iyy = inertia.cy.y, izz = inertia.cz.z;
    f32 ixy = inertia.cy.x;
    f32 slack = 1e-6f * (ixx + iyy + izz);
    bool positive = ixx > 0.0f && ixx * iyy - ixy * ixy > 0.0f && b3Det(inertia) > 0.0f;
    bool triangle =
        ixx + iyy + slack >= izz && ixx + izz + slack >= iyy && iyy + izz + slack >= ixx;
    return positive && triangle;
}

static b3Matrix3 make_diagonal(f32 value)
{
    b3Matrix3 m = {};
    m.cx.x = m.cy.y = m.cz.z = value;
    return m;
}

static void set_body_mass(Robot *robot, u32 body, const u32 *members, u32 member_count)
{
    const Urdf_Model *model = &robot->model;
    f32 mass = 0.0f;
    v3 weighted = {0.0f, 0.0f, 0.0f};
    for (u32 m = 0; m < member_count; m++) {
        const Urdf_Inertial *inertial = &model->links[members[m]].inertial;
        if (!inertial->present || inertial->mass <= 0.0f) {
            continue;
        }
        v3 centre = b3TransformPoint(robot->link_in_body[members[m]], inertial->origin.p);
        mass += inertial->mass;
        weighted = b3MulAdd(weighted, inertial->mass, centre);
    }

    const char *name = model->links[robot->bodies[body].frame_link].name;
    b3MassData data = {};
    if (mass <= 0.0f) {
        log_warning("robot: %s has no mass in the URDF; using %.1f kg", name, (f64)DEFAULT_MASS);
        data.mass = DEFAULT_MASS;
        data.inertia = make_diagonal(1e-3f);
    } else {
        data.mass = mass;
        data.center = b3MulSV(1.0f / mass, weighted);
        for (u32 m = 0; m < member_count; m++) {
            const Urdf_Inertial *inertial = &model->links[members[m]].inertial;
            if (!inertial->present || inertial->mass <= 0.0f) {
                continue;
            }
            b3Transform frame = b3MulTransforms(robot->link_in_body[members[m]], inertial->origin);
            b3Matrix3 rotation = b3MakeMatrixFromQuat(frame.q);
            b3Matrix3 rotated =
                b3MulMM(b3MulMM(rotation, inertial->inertia), b3Transpose(rotation));
            v3 offset = b3Sub(frame.p, data.center);
            data.inertia =
                b3AddMM(data.inertia, b3AddMM(rotated, b3Steiner(inertial->mass, offset)));
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
    b3Body_SetMassData(robot->bodies[body].id, data);
}

static void add_collision_shape(Robot *robot, u32 body, const Urdf_Shape *shape)
{
    const Urdf_Geometry *geometry = &shape->geometry;
    b3Transform pose = b3MulTransforms(robot->link_in_body[shape->link], shape->origin);
    b3BodyId id = robot->bodies[body].id;
    b3ShapeDef def = b3DefaultShapeDef();
    def.density = 0.0f; // mass comes from the URDF inertials
    def.updateBodyMass = false;
    def.baseMaterial.friction = robot->settings.friction;
    if (!robot->model.self_collide) {
        def.filter.groupIndex = robot->settings.collision_group;
    }

    switch (geometry->type) {
    case URDF_GEOMETRY_BOX: {
        b3BoxHull hull = b3MakeTransformedBoxHull(0.5f * geometry->size.x, 0.5f * geometry->size.y,
                                                  0.5f * geometry->size.z, pose);
        b3CreateHullShape(id, &def, &hull.base);
        break;
    }
    case URDF_GEOMETRY_CYLINDER: {
        // Box3D's cylinder runs along y; URDF's along z.
        b3HullData *along_y = b3CreateCylinder(geometry->length, geometry->radius,
                                               -0.5f * geometry->length, CYLINDER_SIDES);
        b3Transform y_to_z = {{0.0f, 0.0f, 0.0f},
                              make_quat_from_axis_angle(v3{1.0f, 0.0f, 0.0f}, 0.5f * B3_PI)};
        b3HullData *placed =
            b3CloneAndTransformHull(along_y, b3MulTransforms(pose, y_to_z), v3{1.0f, 1.0f, 1.0f});
        b3CreateHullShape(id, &def, placed);
        b3DestroyHull(placed);
        b3DestroyHull(along_y);
        break;
    }
    case URDF_GEOMETRY_SPHERE: {
        b3Sphere sphere = {pose.p, geometry->radius};
        b3CreateSphereShape(id, &def, &sphere);
        break;
    }
    case URDF_GEOMETRY_MESH: {
        char path[URDF_PATH_SIZE];
        char message[256];
        Stl_Mesh mesh;
        if (!resolve_resource_path(geometry->mesh, robot->settings.resource_dir, path,
                                   sizeof(path))) {
            log_warning("robot: collision mesh %s not found; shape skipped", geometry->mesh);
            break;
        }
        if (!load_stl(&mesh, path, message, sizeof(message))) {
            log_warning("robot: %s; shape skipped", message);
            break;
        }
        u32 count = mesh.triangle_count * 3;
        v3 *points = (v3 *)mesh.normals; // reuse: same size, no longer needed
        for (u32 i = 0; i < count; i++) {
            v3 local = {mesh.positions[i * 3] * geometry->scale.x,
                        mesh.positions[i * 3 + 1] * geometry->scale.y,
                        mesh.positions[i * 3 + 2] * geometry->scale.z};
            points[i] = b3TransformPoint(pose, local);
        }
        b3HullData *hull = b3CreateHull(points, (int)count, MESH_HULL_VERTICES);
        if (hull) {
            b3CreateHullShape(id, &def, hull);
            b3DestroyHull(hull);
        } else {
            log_warning("robot: could not make a convex hull from %s", path);
        }
        free_stl(&mesh);
        break;
    }
    default:
        break;
    }
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

static bool create_joint(Robot *robot, b3WorldId world, Robot_Joint *joint, char *error,
                         u32 error_size)
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

    // Box3D turns revolute joints about the frame's z and slides prismatic joints along x.
    v3 reference = urdf->type == URDF_JOINT_PRISMATIC ? v3{1.0f, 0.0f, 0.0f} : v3{0.0f, 0.0f, 1.0f};
    b3Transform axis_frame = {{0.0f, 0.0f, 0.0f},
                              b3ComputeQuatBetweenUnitVectors(reference, urdf->axis)};
    joint->frame_a = b3MulTransforms(robot->link_in_body[urdf->parent],
                                     b3MulTransforms(urdf->origin, axis_frame));
    b3Transform frame_b = b3MulTransforms(robot->link_in_body[urdf->child], axis_frame);

    f32 max_effort = get_motor_limit(robot, joint, joint->drive);
    bool motor = max_effort > 0.0f;
    if (urdf->type == URDF_JOINT_PRISMATIC) {
        b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
        def.base.bodyIdA = robot->bodies[joint->body_a].id;
        def.base.bodyIdB = robot->bodies[joint->body_b].id;
        def.base.localFrameA = joint->frame_a;
        def.base.localFrameB = frame_b;
        def.base.constraintHertz = robot->settings.joint_hertz;
        def.enableLimit = urdf->has_limits && urdf->lower < urdf->upper;
        def.lowerTranslation = urdf->lower;
        def.upperTranslation = urdf->upper;
        def.enableMotor = motor;
        def.maxMotorForce = max_effort;
        joint->id = b3CreatePrismaticJoint(world, &def);
    } else {
        b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
        def.base.bodyIdA = robot->bodies[joint->body_a].id;
        def.base.bodyIdB = robot->bodies[joint->body_b].id;
        def.base.localFrameA = joint->frame_a;
        def.base.localFrameB = frame_b;
        def.base.constraintHertz = robot->settings.joint_hertz;
        // A range of a full turn or more is no limit at all, and a stop at ±π would jam.
        def.enableLimit = urdf->type == URDF_JOINT_REVOLUTE && urdf->has_limits &&
                          urdf->upper - urdf->lower < 2.0f * PI_F32 - 0.01f;
        def.lowerAngle = urdf->lower;
        def.upperAngle = urdf->upper;
        def.enableMotor = motor;
        def.maxMotorTorque = max_effort;
        joint->id = b3CreateRevoluteJoint(world, &def);
    }
    return true;
}

static void create_ball(Robot *robot, b3WorldId world, Robot_Ball *ball, const u32 *chain,
                        const b3Transform *rest)
{
    const Urdf_Model *model = &robot->model;
    const Urdf_Joint *first = &model->joints[chain[0]];
    u32 parent = first->parent;
    u32 child = model->joints[chain[2]].child;
    ball->body_a = (u32)robot->link_body[parent];
    ball->body_b = (u32)robot->link_body[child];
    ball->frame_a = b3MulTransforms(robot->link_in_body[parent], first->origin);
    b3Transform pivot = b3MulTransforms(rest[parent], first->origin);
    b3Transform body_b_rest = rest[robot->bodies[ball->body_b].frame_link];
    ball->frame_b = b3InvMulTransforms(body_b_rest, pivot);

    v3 u1 = first->axis;
    v3 u2 = model->joints[chain[1]].axis;
    v3 u3 = b3Cross(u1, u2);
    ball->basis.cx = u1;
    ball->basis.cy = u2;
    ball->basis.cz = u3;
    ball->third_sign = b3Dot(u3, model->joints[chain[2]].axis) >= 0.0f ? 1.0f : -1.0f;

    b3SphericalJointDef def = b3DefaultSphericalJointDef();
    def.base.bodyIdA = robot->bodies[ball->body_a].id;
    def.base.bodyIdB = robot->bodies[ball->body_b].id;
    def.base.localFrameA = ball->frame_a;
    def.base.localFrameB = ball->frame_b;
    def.base.constraintHertz = robot->settings.joint_hertz;
    ball->id = b3CreateSphericalJoint(world, &def);
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

static void find_wheels(Robot *robot, const b3Transform *rest)
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
        v3 axis = b3RotateVector(rest[urdf->child].q, urdf->axis);
        f32 forward = b3Cross(axis, v3{0.0f, 0.0f, 1.0f}).x;
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
            v3 steer_axis = b3RotateVector(rest[steer_urdf->child].q, steer_urdf->axis);
            if (robot->joints[robot->urdf_joint_index[steer]].drive == DRIVE_POSITION &&
                absolute(steer_axis.z) > 0.7f) {
                wheel.steer_joint = robot->urdf_joint_index[steer];
                wheel.steer_sign = steer_axis.z > 0.0f ? 1.0f : -1.0f;
            }
        }
        robot->wheels[robot->wheel_count++] = wheel;
    }
}

bool create_robot(Robot *robot, b3WorldId world, Linear_Allocator *allocator,
                  const Urdf_Model *model, const Robot_Settings *settings, b3Transform spawn,
                  f32 step_seconds, char *error, u32 error_size)
{
    *robot = {};
    robot->model = *model;
    robot->settings = *settings;
    robot->step_seconds = step_seconds;
    u32 links = model->link_count;
    u32 joints = model->joint_count;

    b3Transform *rest = ALLOCATE_ARRAY(allocator, b3Transform, links);
    bool *done = ALLOCATE_ARRAY(allocator, bool, links);
    bool *closure_link = ALLOCATE_ARRAY(allocator, bool, links);
    bool *dummy = ALLOCATE_ARRAY(allocator, bool, links);
    u32 *set = ALLOCATE_ARRAY(allocator, u32, links);
    u32 *members = ALLOCATE_ARRAY(allocator, u32, links);
    i32 *ball_of_joint = ALLOCATE_ARRAY(allocator, i32, max(joints, 1u));
    u32 *chains = ALLOCATE_ARRAY(allocator, u32, max(joints, 1u) * 3);
    robot->bodies = ALLOCATE_ARRAY(allocator, Robot_Body, links);
    robot->link_body = ALLOCATE_ARRAY(allocator, i32, links);
    robot->link_in_body = ALLOCATE_ARRAY(allocator, b3Transform, links);
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
            robot->link_in_body[i] = b3InvMulTransforms(rest[frame], rest[i]);
        }
    }

    for (u32 b = 0; b < robot->body_count; b++) {
        Robot_Body *body = &robot->bodies[b];
        b3Transform pose = b3MulTransforms(spawn, rest[body->frame_link]);
        bool spins = false;
        i32 parent_joint = model->links[body->frame_link].parent_joint;
        if (parent_joint >= 0 && model->joints[parent_joint].type == URDF_JOINT_CONTINUOUS) {
            spins = true;
        }
        b3BodyDef def = b3DefaultBodyDef();
        def.type = b3_dynamicBody;
        def.position = pose.p;
        def.rotation = pose.q;
        def.name = model->links[body->frame_link].name;
        def.enableSleep = false; // commands must always take effect
        def.allowFastRotation = spins;
        body->id = b3CreateBody(world, &def);
        body->current = body->previous = body->start = pose;

        u32 member_count = 0;
        for (u32 i = 0; i < links; i++) {
            if (robot->link_body[i] == (i32)b) {
                members[member_count++] = i;
            }
        }
        for (u32 m = 0; m < member_count; m++) {
            const Urdf_Link *link = &model->links[members[m]];
            for (u32 c = 0; c < link->collision_count; c++) {
                add_collision_shape(robot, b, &model->collisions[link->first_collision + c]);
            }
        }
        set_body_mass(robot, b, members, member_count);
    }

    // Joints: every movable URDF joint gets a Robot_Joint; balls get one Box3D joint.
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
        if (!create_joint(robot, world, joint, error, error_size)) {
            destroy_robot(robot);
            return false;
        }
    }
    for (u32 b = 0; b < robot->ball_count; b++) {
        const u32 *chain = &chains[b * 3];
        Robot_Ball *ball = &robot->balls[b];
        create_ball(robot, world, ball, chain, rest);
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
    for (u32 b = 0; b < robot->body_count; b++) {
        if (b3Body_IsValid(robot->bodies[b].id)) {
            b3DestroyBody(robot->bodies[b].id); // also destroys its shapes and joints
        }
    }
    robot->body_count = 0;
    robot->joint_count = 0;
    robot->ball_count = 0;
    robot->wheel_count = 0;
}

// Effort control: the commanded torque (or force) acts on both bodies, equal and opposite,
// about (or along) the joint axis.
static void apply_joint_effort(Robot *robot, Robot_Joint *joint)
{
    const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
    b3BodyId body_a = robot->bodies[joint->body_a].id;
    b3BodyId body_b = robot->bodies[joint->body_b].id;
    b3Transform frame = b3MulTransforms(b3Body_GetTransform(body_a), joint->frame_a);
    if (urdf->type == URDF_JOINT_PRISMATIC) {
        v3 force = joint->command * b3RotateVector(frame.q, v3{1.0f, 0.0f, 0.0f});
        b3Body_ApplyForce(body_b, force, frame.p, true);
        b3Body_ApplyForce(body_a, -force, frame.p, true);
    } else {
        v3 torque = joint->command * b3RotateVector(frame.q, v3{0.0f, 0.0f, 1.0f});
        b3Body_ApplyTorque(body_b, torque, true);
        b3Body_ApplyTorque(body_a, -torque, true);
    }
}

void apply_robot_commands(Robot *robot)
{
    for (u32 i = 0; i < robot->joint_count; i++) {
        Robot_Joint *joint = &robot->joints[i];
        if (joint->drive == DRIVE_NONE || B3_IS_NULL(joint->id)) {
            continue;
        }
        if (joint->drive == DRIVE_EFFORT) {
            apply_joint_effort(robot, joint);
            continue;
        }
        const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
        f32 speed = joint->command;
        if (joint->drive == DRIVE_POSITION) {
            // A servo that heads for its target, within the joint's limits, at servo_speed
            // (and arrives exactly, without overshoot).
            f32 target = joint->command;
            if (urdf->has_limits && urdf->lower < urdf->upper) {
                target = min(max(target, urdf->lower), urdf->upper);
            }
            speed = (target - joint->position) / robot->step_seconds;
            f32 limit = robot->settings.servo_speed;
            if (limit > 0.0f) {
                speed = min(max(speed, -limit), limit);
            }
        }
        if (urdf->type == URDF_JOINT_PRISMATIC) {
            b3PrismaticJoint_SetMotorSpeed(joint->id, speed);
        } else {
            b3RevoluteJoint_SetMotorSpeed(joint->id, speed);
        }
    }
}

void set_joint_drive(Robot *robot, u32 joint_index, Joint_Drive drive)
{
    Robot_Joint *joint = &robot->joints[joint_index];
    if (joint->drive == drive || B3_IS_NULL(joint->id)) {
        return;
    }
    joint->drive = drive;
    // Coasting, or effort control, still feels the URDF's joint friction, as an undriven
    // motor does.
    f32 limit = get_motor_limit(robot, joint, drive);
    if (drive == DRIVE_NONE) {
        joint->command = 0.0f;
    }
    if (robot->model.joints[joint->urdf_joint].type == URDF_JOINT_PRISMATIC) {
        b3PrismaticJoint_EnableMotor(joint->id, limit > 0.0f);
        b3PrismaticJoint_SetMaxMotorForce(joint->id, limit);
        b3PrismaticJoint_SetMotorSpeed(joint->id, 0.0f);
    } else {
        b3RevoluteJoint_EnableMotor(joint->id, limit > 0.0f);
        b3RevoluteJoint_SetMaxMotorTorque(joint->id, limit);
        b3RevoluteJoint_SetMotorSpeed(joint->id, 0.0f);
    }
}

// Angles (a, b, c) with m = Rx(a) Ry(b) Rz(c).
static v3 get_angles_xyz(b3Matrix3 m)
{
    f32 sin_b = min(max(m.cz.x, -1.0f), 1.0f);
    return v3{atan2f(-m.cz.y, m.cz.z), asinf(sin_b), atan2f(-m.cy.x, m.cx.x)};
}

// A ball joint's angle, as one of its three URDF joints. The position keeps counting
// past a turn, and the velocity comes from the change since the last step.
static void update_joint_angle(Robot *robot, u32 joint_index, f32 angle)
{
    Robot_Joint *joint = &robot->joints[joint_index];
    f32 delta = wrap_pi(angle - wrap_pi(joint->position));
    joint->velocity = delta / robot->step_seconds;
    joint->position += delta;
}

static void read_ball(Robot *robot, Robot_Ball *ball)
{
    b3Transform body_a = robot->bodies[ball->body_a].current;
    b3Transform body_b = robot->bodies[ball->body_b].current;
    b3Quat frame_a = b3MulQuat(body_a.q, ball->frame_a.q);
    b3Quat frame_b = b3MulQuat(body_b.q, ball->frame_b.q);
    b3Matrix3 relative = b3MakeMatrixFromQuat(b3InvMulQuat(frame_a, frame_b));
    b3Matrix3 in_basis = b3MulMM(b3MulMM(b3Transpose(ball->basis), relative), ball->basis);
    v3 angles = get_angles_xyz(in_basis);
    update_joint_angle(robot, ball->joints[0], angles.x);
    update_joint_angle(robot, ball->joints[1], angles.y);
    update_joint_angle(robot, ball->joints[2], ball->third_sign * angles.z);
}

void read_robot_state(Robot *robot)
{
    for (u32 b = 0; b < robot->body_count; b++) {
        Robot_Body *body = &robot->bodies[b];
        body->previous = body->current;
        body->current = b3Body_GetTransform(body->id);
    }
    for (u32 i = 0; i < robot->joint_count; i++) {
        Robot_Joint *joint = &robot->joints[i];
        if (B3_IS_NULL(joint->id)) {
            continue;
        }
        const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
        if (urdf->type == URDF_JOINT_PRISMATIC) {
            joint->position = b3PrismaticJoint_GetTranslation(joint->id);
            joint->velocity = b3PrismaticJoint_GetSpeed(joint->id);
            joint->effort = joint->drive == DRIVE_EFFORT
                                ? joint->command
                                : b3PrismaticJoint_GetMotorForce(joint->id);
            continue;
        }
        // Keep counting past ±π, as joint states do for continuous joints.
        f32 angle = b3RevoluteJoint_GetAngle(joint->id);
        joint->position += wrap_pi(angle - wrap_pi(joint->position));
        b3BodyId a = robot->bodies[joint->body_a].id;
        b3BodyId b = robot->bodies[joint->body_b].id;
        v3 axis =
            b3RotateVector(b3MulQuat(robot->bodies[joint->body_a].current.q, joint->frame_a.q),
                           v3{0.0f, 0.0f, 1.0f});
        v3 relative = b3Sub(b3Body_GetAngularVelocity(b), b3Body_GetAngularVelocity(a));
        joint->velocity = b3Dot(relative, axis);
        joint->effort = joint->drive == DRIVE_EFFORT ? joint->command
                                                     : b3RevoluteJoint_GetMotorTorque(joint->id);
    }
    for (u32 b = 0; b < robot->ball_count; b++) {
        read_ball(robot, &robot->balls[b]);
    }
}

void reset_robot(Robot *robot)
{
    for (u32 b = 0; b < robot->body_count; b++) {
        Robot_Body *body = &robot->bodies[b];
        b3Body_SetTransform(body->id, body->start.p, body->start.q);
        b3Body_SetLinearVelocity(body->id, v3{0.0f, 0.0f, 0.0f});
        b3Body_SetAngularVelocity(body->id, v3{0.0f, 0.0f, 0.0f});
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

b3Transform get_link_pose(const Robot *robot, u32 link, f32 alpha)
{
    i32 body = robot->link_body[link];
    if (body < 0) {
        return identity_transform;
    }
    const Robot_Body *b = &robot->bodies[body];
    b3Transform pose;
    pose.p = b3Lerp(b->previous.p, b->current.p, alpha);
    pose.q = b3NLerp(b->previous.q, b->current.q, alpha);
    return b3MulTransforms(pose, robot->link_in_body[link]);
}

i32 find_robot_joint(const Robot *robot, const char *name)
{
    i32 urdf = find_urdf_joint(&robot->model, name);
    return urdf >= 0 ? robot->urdf_joint_index[urdf] : -1;
}
