#pragma once

#include "core/math.h"

#include "core/allocator.h"

// A parsed URDF: plain arrays, allocated once from an allocator. Frames follow URDF: each
// joint's origin places the child link frame in the parent link frame at q = 0.

#define URDF_NAME_SIZE 64
#define URDF_PATH_SIZE 256
#define URDF_MAX_CLOSURES 16
#define URDF_MAX_CONTROL_PARAMS 16

enum Urdf_Geometry_Type {
    URDF_GEOMETRY_NONE,
    URDF_GEOMETRY_BOX,
    URDF_GEOMETRY_CYLINDER,
    URDF_GEOMETRY_SPHERE,
    URDF_GEOMETRY_MESH,
};

struct Urdf_Geometry {
    Urdf_Geometry_Type type;
    v3 size; // box: full extents
    f32 radius; // cylinder, sphere
    f32 length; // cylinder, along z
    v3 scale; // mesh
    char mesh[URDF_PATH_SIZE]; // as written: file://, package:// or a relative path
};

// One <visual> or <collision>.
struct Urdf_Shape {
    u32 link;
    Pose origin; // in the link frame
    Urdf_Geometry geometry;
    f32 color[4]; // visuals: rgba from the material, resolved after parsing
    char material[URDF_NAME_SIZE];
};

struct Urdf_Inertial {
    bool present;
    f32 mass;
    Pose origin; // centre of mass and inertia axes, in the link frame
    Mat3 inertia; // about the centre of mass, in the inertia axes
};

struct Urdf_Link {
    char name[URDF_NAME_SIZE];
    Urdf_Inertial inertial;
    u32 first_visual;
    u32 visual_count;
    u32 first_collision;
    u32 collision_count;
    i32 parent_joint; // -1 for the root
};

enum Urdf_Joint_Type {
    URDF_JOINT_FIXED,
    URDF_JOINT_REVOLUTE,
    URDF_JOINT_CONTINUOUS,
    URDF_JOINT_PRISMATIC,
    URDF_JOINT_FLOATING,
    URDF_JOINT_PLANAR,
};

struct Urdf_Joint {
    char name[URDF_NAME_SIZE];
    Urdf_Joint_Type type;
    u32 parent; // link indices
    u32 child;
    Pose origin; // child frame in the parent frame at q = 0
    v3 axis; // unit, in the child frame
    bool has_limits;
    f32 lower;
    f32 upper;
    f32 effort; // 0 when not given
    f32 velocity;
    f32 damping;
    f32 friction;
    // From <ros2_control> or <transmission>: driven by an actuator, and how.
    bool actuated;
    bool position_command; // has a position command interface
    i32 control; // index into Urdf_Model::controls, or -1
};

// A link welded to another at run time, closing a kinematic loop. Written in URDF as a
// Gazebo DetachableJoint plugin, since URDF itself can only describe trees.
struct Urdf_Closure {
    u32 parent_link;
    u32 child_link;
};

struct Urdf_Control_Param {
    char name[URDF_NAME_SIZE];
    char value[URDF_NAME_SIZE];
};

// One <ros2_control> block: the hardware plugin and its parameters (CAN id, gear ratio
// and so on), kept for the CAN adapter.
struct Urdf_Control {
    char name[URDF_NAME_SIZE];
    char plugin[URDF_NAME_SIZE];
    Urdf_Control_Param params[URDF_MAX_CONTROL_PARAMS];
    u32 param_count;
};

struct Urdf_Model {
    char name[URDF_NAME_SIZE];
    Urdf_Link *links;
    u32 link_count;
    Urdf_Joint *joints;
    u32 joint_count;
    Urdf_Shape *visuals;
    u32 visual_count;
    Urdf_Shape *collisions;
    u32 collision_count;
    Urdf_Control *controls;
    u32 control_count;
    Urdf_Closure closures[URDF_MAX_CLOSURES];
    u32 closure_count;
    u32 root; // link index
    bool self_collide; // <gazebo><self_collide>; false when not given, as in Gazebo
};

// Parses and validates a URDF (already expanded, if it came from xacro). The model
// must describe a single tree; closures may then join links in it.
bool parse_urdf(Urdf_Model *model, Linear_Allocator *allocator, const char *xml, u64 xml_size,
                char *error, u32 error_size);

i32 find_urdf_link(const Urdf_Model *model, const char *name);
i32 find_urdf_joint(const Urdf_Model *model, const char *name);
const char *get_control_param(const Urdf_Model *model, i32 control, const char *name);

// URDF roll-pitch-yaw (fixed axes x, then y, then z) as a quaternion.
Quat make_quat_from_rpy(f64 roll, f64 pitch, f64 yaw);
