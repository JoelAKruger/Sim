// The one file that includes Chrono. It builds with exceptions and RTTI, which Chrono needs;
// everything it hands out is a plain index or a plain struct from core/physics.h.

#include "core/physics.h"

#include <math.h>
#include <stdio.h>

#include <memory>
#include <vector>

#include <chrono/collision/ChCollisionShapeBox.h>
#include <chrono/collision/ChCollisionShapeConvexHull.h>
#include <chrono/collision/ChCollisionShapeCylinder.h>
#include <chrono/collision/ChCollisionShapeSphere.h>
#include <chrono/collision/ChCollisionShapeTriangleMesh.h>
#include <chrono/collision/bullet/ChCollisionUtilsBullet.h>
#include <chrono/functions/ChFunctionConst.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/geometry/ChTriangleMeshSoup.h>
#include <chrono/physics/ChBodyAuxRef.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMotorLinearForce.h>
#include <chrono/physics/ChLinkMotorLinearSpeed.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/solver/ChIterativeSolverVI.h>
#include <chrono_vehicle/terrain/SCMTerrain.h>

using namespace chrono;

#define COLLISION_ENVELOPE 0.01 // m around shapes where contacts start to be tracked
#define COLLISION_MARGIN 0.005 // m of each shape Bullet rounds off
#define FIXED_FAMILY 14 // the collision family of fixed bodies; shape groups are 1..13

// A revolute or prismatic joint's motors. Chrono's speed motor has no torque limit, so a
// joint driven at a speed switches to its torque motor, at the limit, whenever the speed
// motor would need more; it switches back once it has caught up.
struct Physics_Joint {
    Joint_Kind kind;
    u32 body_a;
    u32 body_b;
    Pose frame_a;
    Pose frame_b;
    std::shared_ptr<ChLinkLock> link;
    std::shared_ptr<ChLinkMotor> speed_motor; // ChLinkMotorRotationSpeed or ...LinearSpeed
    std::shared_ptr<ChLinkMotor> effort_motor; // ChLinkMotorRotationTorque or ...LinearForce
    std::shared_ptr<ChFunctionConst> speed;
    std::shared_ptr<ChFunctionConst> effort;
    Joint_Motor motor;
    f32 value;
    f32 max_effort;
    bool saturated; // the speed motor hit max_effort; the effort motor holds it there
    f32 last_effort;
};

struct Physics {
    ChSystemNSC system;
    std::vector<std::shared_ptr<ChBodyAuxRef>> bodies;
    std::vector<unsigned int> accumulators; // per body: its force accumulator
    std::vector<Physics_Joint> joints;

    std::shared_ptr<vehicle::SCMTerrain> scm;
    Soil *soil;
    f64 soil_base; // the SCM frame's height; SCM levels are above it
    i32 soil_half_cols; // SCM node (i, j) is soil sample (half_rows - j, i + half_cols)
    i32 soil_half_rows;
};

static ChVector3d to_chrono(v3 v) { return ChVector3d(v.x, v.y, v.z); }

static v3 from_chrono(const ChVector3d &v) { return v3{(f32)v.x(), (f32)v.y(), (f32)v.z()}; }

static ChQuaterniond to_chrono(Quat q) { return ChQuaterniond(q.s, q.v.x, q.v.y, q.v.z); }

static ChFrame<> to_chrono(Pose pose) { return ChFrame<>(to_chrono(pose.p), to_chrono(pose.q)); }

static Pose from_chrono(const ChFrame<> &frame)
{
    const ChQuaterniond &q = frame.GetRot();
    return Pose{from_chrono(frame.GetPos()),
                Quat{{(f32)q.e1(), (f32)q.e2(), (f32)q.e3()}, (f32)q.e0()}};
}

Physics *create_physics(const Physics_Settings *settings)
{
    Physics *physics = new Physics();
    ChSystemNSC *system = &physics->system;
    system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    system->SetGravitationalAcceleration(to_chrono(settings->gravity));
    i32 threads = (i32)max(settings->threads, 1u);
    system->SetNumThreads(threads, threads, 1);
    ChCollisionModel::SetDefaultSuggestedEnvelope(COLLISION_ENVELOPE);
    ChCollisionModel::SetDefaultSuggestedMargin(COLLISION_MARGIN);

    // Barzilai-Borwein: an iterative solver for hard contacts and joint limits, fast enough
    // for real time. A direct one (ADMM over a factorisation) holds joints tighter but
    // costs forty times as much.
    system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    auto solver = system->GetSolver()->AsIterative();
    solver->SetMaxIterations((int)settings->solver_iterations);
    solver->EnableWarmStart(true);
    system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    return physics;
}

void destroy_physics(Physics *physics) { delete physics; }

// The joint's rate: rad/s about its axis, or m/s along it.
static f32 get_joint_rate(const Physics *physics, const Physics_Joint *joint)
{
    Pose pose_a = get_body_pose(physics, joint->body_a);
    Pose frame = multiply_poses(pose_a, joint->frame_a);
    if (joint->kind == JOINT_PRISMATIC) {
        v3 axis = rotate_vector(frame.q, v3{1.0f, 0.0f, 0.0f});
        v3 point = transform_point(get_body_pose(physics, joint->body_b), joint->frame_b.p);
        return dot(get_point_velocity(physics, joint->body_b, point) -
                       get_point_velocity(physics, joint->body_a, point),
                   axis);
    }
    v3 axis = rotate_vector(frame.q, v3{0.0f, 0.0f, 1.0f});
    return dot(get_body_angular_velocity(physics, joint->body_b) -
                   get_body_angular_velocity(physics, joint->body_a),
               axis);
}

static f32 get_motor_effort(const Physics_Joint *joint, const std::shared_ptr<ChLinkMotor> &motor)
{
    if (joint->kind == JOINT_PRISMATIC) {
        return (f32)std::static_pointer_cast<ChLinkMotorLinear>(motor)->GetMotorForce();
    }
    return (f32)std::static_pointer_cast<ChLinkMotorRotation>(motor)->GetMotorTorque();
}

// Before a substep: which motor acts, and with what.
static void set_joint_motors(Physics_Joint *joint)
{
    if (!joint->speed_motor) {
        return;
    }
    bool speed = joint->motor == MOTOR_SPEED && joint->max_effort > 0.0f && !joint->saturated;
    bool effort = joint->motor == MOTOR_EFFORT ||
                  (joint->motor == MOTOR_SPEED && joint->max_effort > 0.0f && joint->saturated);
    joint->speed_motor->SetDisabled(!speed);
    joint->effort_motor->SetDisabled(!effort);
    joint->speed->SetConstant(joint->value);
    if (joint->motor == MOTOR_EFFORT) {
        joint->effort->SetConstant(joint->value);
    }
}

// After a substep: a speed-driven joint saturates when it needs more than max_effort, and
// recovers when it has reached its speed.
static void update_joint_saturation(Physics *physics, Physics_Joint *joint)
{
    if (!joint->speed_motor) {
        return;
    }
    if (joint->motor != MOTOR_SPEED || joint->max_effort <= 0.0f) {
        joint->saturated = false;
        joint->last_effort = joint->motor == MOTOR_EFFORT ? joint->value : 0.0f;
        return;
    }
    if (!joint->saturated) {
        f32 effort = get_motor_effort(joint, joint->speed_motor);
        if (absolute(effort) > joint->max_effort) {
            joint->saturated = true;
            f32 held = effort > 0.0f ? joint->max_effort : -joint->max_effort;
            joint->effort->SetConstant(held);
            effort = held;
        }
        joint->last_effort = effort;
        return;
    }
    f32 held = (f32)joint->effort->GetConstant();
    f32 rate = get_joint_rate(physics, joint);
    joint->last_effort = held;
    if ((held > 0.0f && rate >= joint->value) || (held < 0.0f && rate <= joint->value)) {
        joint->saturated = false;
    }
}

// The soil samples SCM moved in the last substep.
static void read_soil_changes(Physics *physics)
{
    Soil *soil = physics->soil;
    for (const auto &node : physics->scm->GetModifiedNodes(false)) {
        i32 col = node.first.x() + physics->soil_half_cols;
        i32 row = physics->soil_half_rows - node.first.y();
        if (col >= 0 && row >= 0 && (u32)col < soil->grid.cols && (u32)row < soil->grid.rows) {
            set_soil_height(soil, (u32)row, (u32)col, (f32)(physics->soil_base + node.second));
        }
    }
}

void step_physics(Physics *physics, f32 seconds, u32 substeps)
{
    substeps = max(substeps, 1u);
    f64 step = (f64)seconds / (f64)substeps;
    for (u32 i = 0; i < substeps; i++) {
        for (Physics_Joint &joint : physics->joints) {
            set_joint_motors(&joint);
        }
        physics->system.DoStepDynamics(step);
        for (Physics_Joint &joint : physics->joints) {
            update_joint_saturation(physics, &joint);
        }
        if (physics->scm) {
            read_soil_changes(physics);
        }
    }
    for (u32 b = 0; b < physics->bodies.size(); b++) {
        physics->bodies[b]->EmptyAccumulator(physics->accumulators[b]);
    }
}

u32 add_body(Physics *physics, Pose pose, bool fixed)
{
    auto body = chrono_types::make_shared<ChBodyAuxRef>();
    body->SetFrameRefToAbs(to_chrono(pose));
    body->SetFixed(fixed);
    body->SetSleepingAllowed(false); // commands must always take effect
    physics->system.AddBody(body);
    physics->bodies.push_back(body);
    physics->accumulators.push_back(body->AddAccumulator());
    return (u32)physics->bodies.size() - 1;
}

void set_body_mass(Physics *physics, u32 body, f32 mass, v3 center, Mat3 inertia)
{
    ChBodyAuxRef *b = physics->bodies[body].get();
    b->SetFrameCOMToRef(ChFrame<>(to_chrono(center), QUNIT));
    b->SetMass(mass);
    b->SetInertiaXX(ChVector3d(inertia.cx.x, inertia.cy.y, inertia.cz.z));
    b->SetInertiaXY(ChVector3d(inertia.cy.x, inertia.cz.x, inertia.cz.y));
}

static std::shared_ptr<ChContactMaterialNSC> make_material(const Shape_Material *material)
{
    auto contact = chrono_types::make_shared<ChContactMaterialNSC>();
    contact->SetFriction(material->friction);
    contact->SetRestitution(0.0f);
    return contact;
}

static void add_shape(Physics *physics, u32 body, std::shared_ptr<ChCollisionShape> shape,
                      const ChFrame<> &frame, const Shape_Material *material)
{
    ChBodyAuxRef *b = physics->bodies[body].get();
    b->AddCollisionShape(shape, frame);
    b->EnableCollision(true);
    // Fixed bodies (the ground, the rocks) share a family that ignores itself, or Bullet
    // would test every rock against the ground under it every step.
    int family = b->IsFixed() ? FIXED_FAMILY : (int)material->group;
    if (family > 0) {
        b->GetCollisionModel()->SetFamily(family);
        b->GetCollisionModel()->DisallowCollisionsWith(family);
    }
}

void add_box_shape(Physics *physics, u32 body, Pose pose, v3 half_extents,
                   const Shape_Material *material)
{
    auto shape = chrono_types::make_shared<ChCollisionShapeBox>(
        make_material(material), 2.0 * half_extents.x, 2.0 * half_extents.y, 2.0 * half_extents.z);
    add_shape(physics, body, shape, to_chrono(pose), material);
}

void add_sphere_shape(Physics *physics, u32 body, v3 center, f32 radius,
                      const Shape_Material *material)
{
    auto shape = chrono_types::make_shared<ChCollisionShapeSphere>(make_material(material), radius);
    add_shape(physics, body, shape, ChFrame<>(to_chrono(center), QUNIT), material);
}

void add_cylinder_shape(Physics *physics, u32 body, Pose pose, f32 radius, f32 length,
                        const Shape_Material *material)
{
    auto shape = chrono_types::make_shared<ChCollisionShapeCylinder>(make_material(material),
                                                                     radius, length);
    add_shape(physics, body, shape, to_chrono(pose), material);
}

void add_hull_shape(Physics *physics, u32 body, const v3 *points, u32 count,
                    const Shape_Material *material)
{
    std::vector<ChVector3d> hull_points(count);
    for (u32 i = 0; i < count; i++) {
        hull_points[i] = to_chrono(points[i]);
    }
    auto shape =
        chrono_types::make_shared<ChCollisionShapeConvexHull>(make_material(material), hull_points);
    add_shape(physics, body, shape, ChFrame<>(), material);
}

void add_mesh_shape(Physics *physics, u32 body, const v3 *vertices, u32 vertex_count,
                    const u32 *indices, u32 triangle_count, const Shape_Material *material)
{
    // A soup, not a connected mesh: Chrono gives a connected one a collision shape per
    // triangle, but a fixed soup one bounding-volume tree, which is far faster to query.
    (void)vertex_count;
    auto mesh = chrono_types::make_shared<ChTriangleMeshSoup>();
    for (u32 t = 0; t < triangle_count; t++) {
        mesh->AddTriangle(to_chrono(vertices[indices[t * 3]]),
                          to_chrono(vertices[indices[t * 3 + 1]]),
                          to_chrono(vertices[indices[t * 3 + 2]]));
    }
    auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(make_material(material),
                                                                         mesh, true, false, 0.0);
    add_shape(physics, body, shape, ChFrame<>(), material);
    // Chrono pads a fixed mesh by its margin only, but counts its whole envelope when it
    // reports a contact, so things would sink the difference into it.
    physics->bodies[body]->GetCollisionModel()->SetEnvelope(COLLISION_MARGIN);
}

u32 make_convex_hull(const v3 *points, u32 count, v3 *triangles, u32 capacity)
{
    if (count < 4) {
        return 0;
    }
    std::vector<ChVector3d> hull_points(count);
    v3 centroid = {0.0f, 0.0f, 0.0f};
    for (u32 i = 0; i < count; i++) {
        hull_points[i] = to_chrono(points[i]);
        centroid += (1.0f / (f32)count) * points[i];
    }
    ChTriangleMeshConnected hull;
    bt_utils::ChConvexHullLibraryWrapper library;
    library.ComputeHull(hull_points, hull);
    const std::vector<ChVector3d> &vertices = hull.GetCoordsVertices();
    const std::vector<ChVector3i> &faces = hull.GetIndicesVertexes();
    if (faces.size() < 4 || faces.size() * 3 > capacity) {
        return 0;
    }
    u32 written = 0;
    for (const ChVector3i &face : faces) {
        v3 a = from_chrono(vertices[face.x()]);
        v3 b = from_chrono(vertices[face.y()]);
        v3 c = from_chrono(vertices[face.z()]);
        // Outward and counter-clockwise, whichever way the library wound it.
        if (dot(cross(b - a, c - a), a - centroid) < 0.0f) {
            v3 swap = b;
            b = c;
            c = swap;
        }
        triangles[written++] = a;
        triangles[written++] = b;
        triangles[written++] = c;
    }
    return written;
}

Pose get_body_pose(const Physics *physics, u32 body)
{
    return from_chrono(physics->bodies[body]->GetFrameRefToAbs());
}

v3 get_body_velocity(const Physics *physics, u32 body)
{
    return from_chrono(physics->bodies[body]->GetLinVel());
}

v3 get_body_angular_velocity(const Physics *physics, u32 body)
{
    return from_chrono(physics->bodies[body]->GetAngVelParent());
}

v3 get_point_velocity(const Physics *physics, u32 body, v3 world_point)
{
    // The body is its centre-of-mass frame.
    const ChBodyAuxRef *b = physics->bodies[body].get();
    return from_chrono(
        b->PointSpeedLocalToParent(b->TransformPointParentToLocal(to_chrono(world_point))));
}

void set_body_pose(Physics *physics, u32 body, Pose pose)
{
    physics->bodies[body]->SetFrameRefToAbs(to_chrono(pose));
}

void set_body_velocity(Physics *physics, u32 body, v3 linear, v3 angular)
{
    ChBodyAuxRef *b = physics->bodies[body].get();
    b->SetLinVel(to_chrono(linear));
    b->SetAngVelParent(to_chrono(angular));
}

void apply_body_force(Physics *physics, u32 body, v3 force, v3 world_point)
{
    physics->bodies[body]->AccumulateForce(physics->accumulators[body], to_chrono(force),
                                           to_chrono(world_point), false);
}

void apply_body_torque(Physics *physics, u32 body, v3 torque)
{
    physics->bodies[body]->AccumulateTorque(physics->accumulators[body], to_chrono(torque), false);
}

// Chrono slides prismatic joints along z; ours slide along x.
static ChFrame<> make_slide_frame(Pose pose)
{
    Quat x_to_z = make_quat_from_axis_angle(v3{0.0f, 1.0f, 0.0f}, 0.5f * PI_F32);
    return to_chrono(Pose{pose.p, multiply_quats(pose.q, x_to_z)});
}

u32 add_joint(Physics *physics, const Joint_Def *def)
{
    Physics_Joint joint = {};
    joint.kind = def->kind;
    joint.body_a = def->body_a;
    joint.body_b = def->body_b;
    joint.frame_a = def->frame_a;
    joint.frame_b = def->frame_b;
    std::shared_ptr<ChBodyAuxRef> parent = physics->bodies[def->body_a];
    std::shared_ptr<ChBodyAuxRef> child = physics->bodies[def->body_b];
    // In the world: Chrono's body-relative frames are relative to the centre of mass, not
    // the body's frame.
    Pose world_a = multiply_poses(get_body_pose(physics, def->body_a), def->frame_a);
    Pose world_b = multiply_poses(get_body_pose(physics, def->body_b), def->frame_b);
    ChFrame<> frame_a =
        def->kind == JOINT_PRISMATIC ? make_slide_frame(world_a) : to_chrono(world_a);
    ChFrame<> frame_b =
        def->kind == JOINT_PRISMATIC ? make_slide_frame(world_b) : to_chrono(world_b);

    // The child's marker is the moving one; angles and slides are measured from the
    // parent's.
    switch (def->kind) {
    case JOINT_REVOLUTE:
        joint.link = chrono_types::make_shared<ChLinkLockRevolute>();
        break;
    case JOINT_PRISMATIC:
        joint.link = chrono_types::make_shared<ChLinkLockPrismatic>();
        break;
    case JOINT_SPHERICAL:
        joint.link = chrono_types::make_shared<ChLinkLockSpherical>();
        break;
    }
    joint.link->Initialize(child, parent, false, frame_b, frame_a);
    if (def->limited && def->kind != JOINT_SPHERICAL) {
        ChLinkLimit &limit =
            def->kind == JOINT_PRISMATIC ? joint.link->LimitZ() : joint.link->LimitRz();
        limit.SetActive(true);
        limit.SetMin(def->lower);
        limit.SetMax(def->upper);
    }
    physics->system.AddLink(joint.link);

    if (def->kind != JOINT_SPHERICAL) {
        joint.speed = chrono_types::make_shared<ChFunctionConst>(0.0);
        joint.effort = chrono_types::make_shared<ChFunctionConst>(0.0);
        if (def->kind == JOINT_PRISMATIC) {
            auto speed = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
            speed->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::FREE);
            speed->SetAvoidPositionDrift(false);
            speed->SetSpeedFunction(joint.speed);
            auto force = chrono_types::make_shared<ChLinkMotorLinearForce>();
            force->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::FREE);
            force->SetForceFunction(joint.effort);
            joint.speed_motor = speed;
            joint.effort_motor = force;
        } else {
            auto speed = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            speed->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::FREE);
            speed->AvoidAngleDrift(false);
            speed->SetSpeedFunction(joint.speed);
            auto torque = chrono_types::make_shared<ChLinkMotorRotationTorque>();
            torque->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::FREE);
            torque->SetTorqueFunction(joint.effort);
            joint.speed_motor = speed;
            joint.effort_motor = torque;
        }
        joint.speed_motor->Initialize(child, parent, false, frame_b, frame_a);
        joint.effort_motor->Initialize(child, parent, false, frame_b, frame_a);
        joint.speed_motor->SetDisabled(true);
        joint.effort_motor->SetDisabled(true);
        physics->system.AddLink(joint.speed_motor);
        physics->system.AddLink(joint.effort_motor);
    }
    physics->joints.push_back(joint);
    return (u32)physics->joints.size() - 1;
}

void drive_joint(Physics *physics, u32 joint_index, Joint_Motor motor, f32 value, f32 max_effort)
{
    Physics_Joint *joint = &physics->joints[joint_index];
    if (joint->motor != motor) {
        joint->saturated = false;
    }
    joint->motor = motor;
    joint->value = value;
    joint->max_effort = max_effort;
}

f32 get_joint_effort(const Physics *physics, u32 joint)
{
    return physics->joints[joint].last_effort;
}

f32 get_joint_separation(const Physics *physics, u32 joint_index)
{
    const Physics_Joint *joint = &physics->joints[joint_index];
    v3 a = transform_point(get_body_pose(physics, joint->body_a), joint->frame_a.p);
    v3 b = transform_point(get_body_pose(physics, joint->body_b), joint->frame_b.p);
    if (joint->kind == JOINT_PRISMATIC) {
        // Sliding along the axis is the joint's motion, not a separation.
        Pose frame = multiply_poses(get_body_pose(physics, joint->body_a), joint->frame_a);
        v3 axis = rotate_vector(frame.q, v3{1.0f, 0.0f, 0.0f});
        v3 apart = b - a;
        return get_length(apart - dot(apart, axis) * axis);
    }
    return get_length(b - a);
}

// The soil's surface as SCM wants it: a triangle mesh in the SCM frame (centred on the
// patch, its lowest point at 0), from which SCM samples its own grid. SCM samples inside
// triangles only, so the mesh runs half a sample beyond the patch on every side, and it is
// nudged a fraction of a micrometre so that no sample falls exactly on an edge. Two corner
// points, used by no triangle, fix its bounds, which SCM takes as the patch's extent plus
// one sample all round.
static void make_soil_mesh(const Soil *soil, const Terrain *terrain, f64 base, f64 center_x,
                           f64 center_y, ChTriangleMeshConnected *mesh)
{
    const Terrain *grid = &soil->grid;
    f64 d = grid->spacing;
    f64 x0 = grid->origin_x;
    f64 y0 = grid->origin_y;
    f64 x1 = x0 + (grid->cols - 1) * d;
    f64 y1 = y0 - (grid->rows - 1) * d;
    // The terrain's own sample lines through the patch, plus the half-sample ring.
    std::vector<f64> xs, ys;
    xs.push_back(x0 - 0.5 * d);
    for (u32 c = 0; c <= soil->cell_cols; c++) {
        xs.push_back(terrain->origin_x + (f64)(soil->first_col + c) * terrain->spacing);
    }
    xs.push_back(x1 + 0.5 * d);
    ys.push_back(y0 + 0.5 * d);
    for (u32 r = 0; r <= soil->cell_rows; r++) {
        ys.push_back(terrain->origin_y - (f64)(soil->first_row + r) * terrain->spacing);
    }
    ys.push_back(y1 - 0.5 * d);

    std::vector<ChVector3d> &vertices = mesh->GetCoordsVertices();
    std::vector<ChVector3i> &faces = mesh->GetIndicesVertexes();
    const f64 nudge_x = 1e-7, nudge_y = 3e-7;
    u32 stride = (u32)xs.size();
    for (u32 r = 0; r < ys.size(); r++) {
        for (u32 c = 0; c < xs.size(); c++) {
            f64 height = get_terrain_height(terrain, (f32)xs[c], (f32)ys[r]);
            vertices.push_back(
                ChVector3d(xs[c] - center_x + nudge_x, ys[r] - center_y + nudge_y, height - base));
        }
    }
    // Split each cell along its south-west to north-east diagonal, as the terrain is.
    for (u32 r = 0; r + 1 < ys.size(); r++) {
        for (u32 c = 0; c + 1 < xs.size(); c++) {
            int v11 = (int)(r * stride + c); // north-west
            int v12 = v11 + 1;
            int v21 = v11 + (int)stride;
            int v22 = v21 + 1;
            faces.push_back(ChVector3i(v11, v21, v12));
            faces.push_back(ChVector3i(v22, v12, v21));
        }
    }
    f64 half_x = 0.5 * (x1 - x0) + d;
    f64 half_y = 0.5 * (y0 - y1) + d;
    vertices.push_back(ChVector3d(-half_x, -half_y, 0.0));
    vertices.push_back(ChVector3d(half_x, half_y, 0.0));
}

bool create_physics_soil(Physics *physics, Soil *soil, const Terrain *terrain,
                         const Soil_Settings *settings, char *error, u32 error_size)
{
    const Terrain *grid = &soil->grid;
    physics->soil = soil;
    physics->soil_half_cols = (i32)(grid->cols - 1) / 2;
    physics->soil_half_rows = (i32)(grid->rows - 1) / 2;
    f64 d = grid->spacing;
    f64 center_x = grid->origin_x + physics->soil_half_cols * d;
    f64 center_y = grid->origin_y - physics->soil_half_rows * d;
    // Below everything, so every mesh height is at least 0.
    f64 base = INFINITY;
    for (u32 i = 0; i < grid->rows * grid->cols; i++) {
        base = fmin(base, grid->heights[i]);
    }
    base -= 1.0;
    physics->soil_base = base;

    ChTriangleMeshConnected mesh;
    make_soil_mesh(soil, terrain, base, center_x, center_y, &mesh);
    auto scm = chrono_types::make_shared<vehicle::SCMTerrain>(&physics->system, false);
    scm->SetReferenceFrame(ChCoordsys<>(ChVector3d(center_x, center_y, base), QUNIT));
    scm->SetSoilParameters(settings->bekker_kphi, settings->bekker_kc, settings->bekker_n,
                           settings->cohesion, settings->friction_angle, settings->janosi_shear,
                           settings->elastic_stiffness, settings->damping);
    if (settings->bulldozing) {
        scm->EnableBulldozing(true);
        scm->SetBulldozingParameters(settings->erosion_angle, 1.0, 5, 6);
    }
    // A hair over the spacing, so rounding can't add a sample to the grid.
    scm->Initialize(mesh, d * (1.0 + 1e-9));
    physics->scm = scm;

    // SCM's grid must be ours sample for sample.
    for (u32 row = 0; row < grid->rows; row += max(1u, (grid->rows - 1) / 16)) {
        for (u32 col = 0; col < grid->cols; col++) {
            f64 x = grid->origin_x + col * d;
            f64 y = grid->origin_y - row * d;
            f64 height = scm->GetInitHeight(ChVector3d(x, y, 0.0));
            f32 expected = grid->heights[row * grid->cols + col];
            if (fabs(height - expected) > 1e-3) {
                snprintf(error, error_size,
                         "soil: SCM's surface at (%.3f, %.3f) is %.3f m, not %.3f m", x, y, height,
                         (f64)expected);
                return false;
            }
        }
    }
    return true;
}

void add_soil_domain(Physics *physics, u32 body, v3 center, v3 size)
{
    if (!physics->scm) {
        return;
    }
    physics->scm->AddActiveDomain(physics->bodies[body], to_chrono(center), to_chrono(size));
}

v3 get_soil_force(const Physics *physics, u32 body)
{
    ChVector3d force, torque;
    if (!physics->scm || !physics->scm->GetContactForceBody(physics->bodies[body], force, torque)) {
        return v3{0.0f, 0.0f, 0.0f};
    }
    return from_chrono(force);
}

f32 cast_physics_ray(Physics *physics, v3 origin, v3 direction, f32 max_distance)
{
    ChCollisionSystem::ChRayhitResult result;
    ChVector3d from = to_chrono(origin);
    ChVector3d to = to_chrono(origin + max_distance * direction);
    if (!physics->system.GetCollisionSystem()->RayHit(from, to, result) || !result.hit) {
        return -1.0f;
    }
    // Chrono reports the hit an envelope short of the surface; this is the surface.
    ChVector3d surface =
        result.abs_hitPoint + result.abs_hitNormal * result.hitModel->GetEnvelope();
    return (f32)(surface - from).Length();
}
