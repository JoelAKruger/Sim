#pragma once

#include "core/math.h"
#include "core/soil.h"

// The multibody physics, on Project Chrono. This header is plain C++. core/physics.cpp is
// the only file that includes Chrono (which needs exceptions and RTTI), as
// ros/ros_bridge.cpp is for rclcpp. Bodies and joints are indices, never pointers.
//
// Joint frames follow what the rest of the sim is written for: a revolute joint turns
// about its frames' z axes, and a prismatic joint slides along their x axes.

struct Physics;

struct Physics_Settings {
    v3 gravity; // m/s²
    u32 threads; // Chrono's threads; 1 is single-threaded and repeatable
    u32 solver_iterations;
};

// How a collision shape touches others.
struct Shape_Material {
    f32 friction;
    u32 group; // 0 meets everything; shapes sharing a group 1..13 don't meet each other
};

enum Joint_Kind {
    JOINT_REVOLUTE,
    JOINT_PRISMATIC,
    JOINT_SPHERICAL,
};

struct Joint_Def {
    Joint_Kind kind;
    u32 body_a; // the parent
    u32 body_b; // the child
    Pose frame_a; // joint frame in body A
    Pose frame_b; // the same frame in body B when the joint is at zero
    bool limited;
    f32 lower; // rad or m
    f32 upper;
};

// What drives a revolute or prismatic joint through the next step.
enum Joint_Motor {
    MOTOR_OFF, // free to turn
    MOTOR_SPEED, // tracks a speed with at most max_effort; stalls at the limit
    MOTOR_EFFORT, // applies a torque (or force) across the joint
};

// The soil's Bekker-Wong and Mohr-Coulomb parameters, and how displaced soil heaps up.
struct Soil_Settings {
    f32 bekker_kphi; // Pa/m^n, frictional modulus
    f32 bekker_kc; // Pa/m^(n-1), cohesive modulus
    f32 bekker_n; // sinkage exponent
    f32 cohesion; // Pa
    f32 friction_angle; // degrees
    f32 janosi_shear; // m
    f32 elastic_stiffness; // Pa/m, more than kphi
    f32 damping; // Pa·s/m
    bool bulldozing; // displaced soil heaps up at the rut's edges
    f32 erosion_angle; // degrees; heaps steeper than this slide
};

Physics *create_physics(const Physics_Settings *settings);
void destroy_physics(Physics *physics);

// Advances by seconds, in substeps equal steps. Soil deformed on the way is written to the
// soil passed to create_physics_soil.
void step_physics(Physics *physics, f32 seconds, u32 substeps);

// A body at pose. Fixed bodies never move.
u32 add_body(Physics *physics, Pose pose, bool fixed);
// Mass properties: center in the body frame, inertia about it in the body's axes. Set
// before any joint is attached.
void set_body_mass(Physics *physics, u32 body, f32 mass, v3 center, Mat3 inertia);

// Collision shapes, placed in the body frame. A cylinder's axis is its pose's z.
void add_box_shape(Physics *physics, u32 body, Pose pose, v3 half_extents,
                   const Shape_Material *material);
void add_sphere_shape(Physics *physics, u32 body, v3 center, f32 radius,
                      const Shape_Material *material);
void add_cylinder_shape(Physics *physics, u32 body, Pose pose, f32 radius, f32 length,
                        const Shape_Material *material);
void add_hull_shape(Physics *physics, u32 body, const v3 *points, u32 count,
                    const Shape_Material *material);
// A fixed triangle mesh, counter-clockwise seen from outside.
void add_mesh_shape(Physics *physics, u32 body, const v3 *vertices, u32 vertex_count,
                    const u32 *indices, u32 triangle_count, const Shape_Material *material);

// The convex hull of points as triangles (three vertices each, counter-clockwise seen from
// outside). Returns the vertex count, 0 if the points are degenerate or it won't fit.
u32 make_convex_hull(const v3 *points, u32 count, v3 *triangles, u32 capacity);

Pose get_body_pose(const Physics *physics, u32 body);
v3 get_body_velocity(const Physics *physics, u32 body); // of the centre of mass
v3 get_body_angular_velocity(const Physics *physics, u32 body); // world frame
v3 get_point_velocity(const Physics *physics, u32 body, v3 world_point);
void set_body_pose(Physics *physics, u32 body, Pose pose);
void set_body_velocity(Physics *physics, u32 body, v3 linear, v3 angular);
// Forces act through the next step only.
void apply_body_force(Physics *physics, u32 body, v3 force, v3 world_point);
void apply_body_torque(Physics *physics, u32 body, v3 torque);

// Both bodies must be where the joint is at zero: its two frames coincide.
u32 add_joint(Physics *physics, const Joint_Def *def);
// value is the speed (rad/s or m/s) for MOTOR_SPEED, the effort (N·m or N) for MOTOR_EFFORT.
void drive_joint(Physics *physics, u32 joint, Joint_Motor motor, f32 value, f32 max_effort);
// The torque (or force) the motor applied to the child over the last step.
f32 get_joint_effort(const Physics *physics, u32 joint);
// How far the joint's two frames have come apart, in m: 0 when it holds perfectly.
f32 get_joint_separation(const Physics *physics, u32 joint);

// Deformable soil (SCM) over the soil's grid, starting from the terrain's surface there.
// Bodies only sink into it within the domains added for them. Steps write the soil's new
// heights into soil, which must outlive the physics.
bool create_physics_soil(Physics *physics, Soil *soil, const Terrain *terrain,
                         const Soil_Settings *settings, char *error, u32 error_size);
// Watches a box (center and size in the body frame) around a body for soil contact.
void add_soil_domain(Physics *physics, u32 body, v3 center, v3 size);
// The force the soil put on a body over the last substep.
v3 get_soil_force(const Physics *physics, u32 body);

// Distance to the first collision shape along a unit direction, or -1 for a miss. Slow and
// not thread-safe (the sensors use core/raycast.h): for checking that the physics collides
// with the surfaces everything else assumes.
f32 cast_physics_ray(Physics *physics, v3 origin, v3 direction, f32 max_distance);
