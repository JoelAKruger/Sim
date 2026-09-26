#pragma once

#include "core/allocator.h"
#include "core/robot.h"

// Actuators are the robot's motor controllers: joints whose URDF <ros2_control> block gives
// a controller node id ("canid"). Commands reach them from an adapter (CAN) in joint units,
// and like real controllers they have a watchdog: a command not renewed within the timeout
// expires, and the joint holds (velocity 0, or its last position target).
// Protocol details (frame layouts, encoder counts, gear ratios) belong to the adapter.

enum Actuator_Mode {
    ACTUATOR_OFF, // coast
    ACTUATOR_VELOCITY, // rad/s or m/s
    ACTUATOR_POSITION, // rad or m
    ACTUATOR_EFFORT, // N·m or N
};

struct Actuator_Command {
    u32 node_id;
    Actuator_Mode mode;
    f32 value;
};

// What a controller reports back.
struct Actuator_Telemetry {
    u32 node_id;
    Actuator_Mode mode;
    bool timed_out; // the watchdog has expired the last command
    f32 position;
    f32 velocity;
    f32 effort;
};

struct Robot_Actuator {
    u32 joint; // Robot_Joint index
    u32 node_id;
    bool active; // a command is in force (received, and not yet expired)
    bool timed_out; // the last command expired rather than being replaced
    Actuator_Mode mode;
    f32 command;
    u64 command_step; // the world step when the command arrived
};

struct Actuator_Set {
    Robot_Actuator *actuators;
    u32 count;
};

// One actuator per driven joint with a node id. A node id used twice is an error.
bool create_actuators(Actuator_Set *set, const Robot *robot, Linear_Allocator *allocator,
                      char *error, u32 error_size);

// Takes a command, stamped with the current step. False if no actuator has that node id.
bool command_actuator(Actuator_Set *set, const Actuator_Command *command, u64 step);

// Before a step: expires stale commands, then puts each joint in the mode and target its
// command asks for, or holds it. timeout_steps of 0 disables the watchdog.
void update_actuators(Actuator_Set *set, Robot *robot, u64 step, u64 timeout_steps);

// After a step: one entry per actuator. Returns the count written.
u32 get_actuator_telemetry(const Actuator_Set *set, const Robot *robot,
                           Actuator_Telemetry *telemetry, u32 capacity);

// Forgets every command, so each joint holds until the next one arrives.
void reset_actuators(Actuator_Set *set);
