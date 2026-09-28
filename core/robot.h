#pragma once

#include "core/physics.h"

#include "core/allocator.h"
#include "core/raycast.h"
#include "core/urdf.h"

// A URDF robot as physics bodies and joints.
//
// Motors are near-ideal: each driven joint applies up to Robot_Settings::motor_torque, a
// position-controlled joint moves at Robot_Settings::servo_speed, and
// the URDF's effort and velocity limits are ignored (they are placeholders). Its position
// limits are enforced.
//
// Links joined by fixed joints, or welded by a loop closure, share one body. A chain of
// three revolute joints about one point through geometry-less dummy links (how URDF
// writes a ball joint) becomes one spherical joint, and the three angles are recovered
// from it for joint states. Every other movable joint maps to one physics joint.

enum Joint_Drive {
    DRIVE_NONE, // passive; URDF dynamics friction, if any, acts as a brake
    DRIVE_VELOCITY, // the motor tracks a commanded speed
    DRIVE_POSITION, // the motor moves to a commanded position (within the limits) at servo_speed
    DRIVE_EFFORT, // a commanded torque (or force) is applied across the joint
};

// One movable URDF joint as simulated.
struct Robot_Joint {
    u32 urdf_joint;
    i32 physics_joint; // -1 for a joint folded into a ball
    i32 ball; // index into Robot::balls, or -1
    u32 ball_axis; // which of the ball's three angles this is
    Joint_Drive drive; // as driven now
    Joint_Drive default_drive; // as the URDF describes it; what teleop and holding use
    u32 body_a;
    u32 body_b;
    Pose frame_a; // joint frame in body A; its z (x for prismatic) is the axis
    Pose frame_b; // the same frame in body B with the joint at zero
    f32 position; // rad or m; continuous joints keep counting past a turn
    f32 velocity; // rad/s or m/s
    f32 effort; // N·m or N applied across the joint in the last step
    // DRIVE_VELOCITY: rad/s or m/s; DRIVE_POSITION: rad or m; DRIVE_EFFORT: N·m or N
    f32 command;
};

// A ball joint written in URDF as three revolute joints about one point.
struct Robot_Ball {
    u32 physics_joint;
    u32 body_a;
    u32 body_b;
    Pose frame_a; // the first joint's frame in body A
    Pose frame_b; // the same frame in body B at q = 0
    Mat3 basis; // columns: the three joint axes in frame A (third made right-handed)
    f32 third_sign; // -1 when the URDF's third axis points the other way
    u32 joints[3]; // Robot_Joint indices, in chain order
};

struct Robot_Body {
    u32 physics_body;
    u32 frame_link; // this body's frame is this link's frame
    f32 mass;
    Pose previous; // pose before the last step, for render interpolation
    Pose current;
    Pose start; // pose at spawn, for reset_robot
};

// A wheel that drives the robot, for keyboard teleop.
struct Robot_Wheel {
    u32 joint; // Robot_Joint index
    f32 forward; // m, ahead of the root link (x in the root frame)
    f32 lateral; // m, left of it (y)
    f32 radius;
    f32 forward_sign; // +1 if positive joint speed rolls the robot forward
    i32 steer_joint; // the position-controlled joint that steers this wheel, or -1
    f32 steer_sign; // +1 if a positive steer angle turns the wheel to the left
};

struct Robot_Settings {
    f32 friction;
    f32 motor_torque; // N·m (or N) every driven joint's motor can apply
    f32 servo_speed; // rad/s (or m/s) a position-controlled joint moves at; 0 is instant
    u32 collision_group; // shared by the robot's shapes when self-collision is off (1..14)
    const char *resource_dir; // the URDF file's directory, for relative mesh paths; may be ""
};

struct Robot {
    Urdf_Model model;
    Robot_Settings settings;
    Physics *physics; // the bodies and joints are in it
    Robot_Body *bodies;
    u32 body_count;
    i32 *link_body; // per link: body index, or -1 for a dummy link inside a ball joint
    Pose *link_in_body; // per link
    Robot_Joint *joints;
    u32 joint_count;
    i32 *urdf_joint_index; // per URDF joint: Robot_Joint index, or -1 for fixed joints
    Robot_Ball *balls;
    u32 ball_count;
    Robot_Wheel *wheels;
    u32 wheel_count;
    f32 step_seconds;
};

// Collision shapes' bounds in the root link's frame with every joint at zero, for
// placing the robot before its bodies exist.
void get_rest_bounds(const Urdf_Model *model, v3 *lower, v3 *upper);

// Builds the robot with its root link at spawn. The model's arrays must outlive the robot.
// Its collision shapes also go into scene for the sensors, and into the soil's watch (when
// the physics has soil).
bool create_robot(Robot *robot, Physics *physics, Ray_Scene *scene, Linear_Allocator *allocator,
                  const Urdf_Model *model, const Robot_Settings *settings, Pose spawn,
                  f32 step_seconds, char *error, u32 error_size);
void destroy_robot(Robot *robot);

// Before a step: turns joint commands into motor targets.
void apply_robot_commands(Robot *robot);

// Switches a joint's drive at run time. DRIVE_NONE lets it coast, braked only by its URDF
// friction. Joints folded into a ball joint can't be driven and are left alone.
void set_joint_drive(Robot *robot, u32 joint, Joint_Drive drive);
// After a step: body poses and joint states.
void read_robot_state(Robot *robot);

// Puts every body back where the robot spawned, at rest with every joint at zero and every
// command cleared. For righting a robot that has flipped or got stuck.
void reset_robot(Robot *robot);

// Drives the wheels for a forward speed and yaw rate. Steered wheels point along the
// ground velocity they need (swerve kinematics, so a pure turn spins on the spot);
// wheels without steering skid-steer. While a pivot is still swinging, its wheel drives only
// the part of its speed along the way it points. At rest the steering holds its angles.
void drive_robot(Robot *robot, f32 forward_mps, f32 turn_radps);

// Keyboard driving. drive and turn are -1..1. Turning while moving follows an arc, and the
// steering points the same way whichever way the rover is going, as a car's does: reversing
// with left held swings the rover's nose right. Turning on the spot spins at turn_rate. The
// whole motion is scaled down so no wheel moves faster than max_speed over the ground.
void steer_robot(Robot *robot, f32 drive, f32 turn, f32 max_speed, f32 turn_rate);

// Keyboard control of a position-controlled joint (a digger's): moves its target by direction
// (-1..1) x rate x seconds, within its limits. Other joints are left alone.
void nudge_robot_joint(Robot *robot, u32 joint, f32 direction, f32 rate, f32 seconds);

// A link's pose, blending the last two steps by alpha (for rendering).
Pose get_link_pose(const Robot *robot, u32 link, f32 alpha);

i32 find_robot_joint(const Robot *robot, const char *name);
