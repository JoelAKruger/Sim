#include "core/actuator.h"

#include <stdio.h>
#include <stdlib.h>

// The node id param in a <ros2_control> block. "canid" is what Nova's URDFs use.
static const char *node_id_params[] = {"canid", "can_id", "node_id"};

static bool get_node_id(const Robot *robot, const Urdf_Joint *urdf, u32 *node_id)
{
    for (u32 i = 0; i < ARRAY_COUNT(node_id_params); i++) {
        const char *text = get_control_param(&robot->model, urdf->control, node_id_params[i]);
        if (text) {
            char *end = NULL;
            unsigned long value = strtoul(text, &end, 0);
            if (end == text || *end != 0 || value > UINT32_MAX) {
                log_warning("actuator: %s's %s \"%s\" is not a node id; not an actuator",
                            urdf->name, node_id_params[i], text);
                return false;
            }
            *node_id = (u32)value;
            return true;
        }
    }
    return false;
}

bool create_actuators(Actuator_Set *set, const Robot *robot, Linear_Allocator *allocator,
                      char *error, u32 error_size)
{
    *set = {};
    set->actuators = ALLOCATE_ARRAY(allocator, Robot_Actuator, max(robot->joint_count, 1u));
    if (!set->actuators) {
        snprintf(error, error_size, "out of memory for the actuators");
        return false;
    }
    for (u32 j = 0; j < robot->joint_count; j++) {
        const Robot_Joint *joint = &robot->joints[j];
        const Urdf_Joint *urdf = &robot->model.joints[joint->urdf_joint];
        u32 node_id = 0;
        if (joint->default_drive == DRIVE_NONE || joint->physics_joint < 0 || urdf->control < 0 ||
            !get_node_id(robot, urdf, &node_id)) {
            continue;
        }
        for (u32 a = 0; a < set->count; a++) {
            if (set->actuators[a].node_id == node_id) {
                u32 other_joint = robot->joints[set->actuators[a].joint].urdf_joint;
                const Urdf_Joint *other = &robot->model.joints[other_joint];
                snprintf(error, error_size, "joints %s and %s both have controller node id %u",
                         other->name, urdf->name, node_id);
                return false;
            }
        }
        set->actuators[set->count++] = Robot_Actuator{.joint = j, .node_id = node_id};
    }
    return true;
}

bool command_actuator(Actuator_Set *set, const Actuator_Command *command, u64 step)
{
    for (u32 a = 0; a < set->count; a++) {
        Robot_Actuator *actuator = &set->actuators[a];
        if (actuator->node_id == command->node_id) {
            actuator->active = true;
            actuator->timed_out = false;
            actuator->mode = command->mode;
            actuator->command = command->value;
            actuator->command_step = step;
            return true;
        }
    }
    return false;
}

static Joint_Drive get_drive_of_mode(Actuator_Mode mode)
{
    switch (mode) {
    case ACTUATOR_VELOCITY:
        return DRIVE_VELOCITY;
    case ACTUATOR_POSITION:
        return DRIVE_POSITION;
    case ACTUATOR_EFFORT:
        return DRIVE_EFFORT;
    default:
        return DRIVE_NONE;
    }
}

static Actuator_Mode get_mode_of_drive(Joint_Drive drive)
{
    switch (drive) {
    case DRIVE_VELOCITY:
        return ACTUATOR_VELOCITY;
    case DRIVE_POSITION:
        return ACTUATOR_POSITION;
    case DRIVE_EFFORT:
        return ACTUATOR_EFFORT;
    default:
        return ACTUATOR_OFF;
    }
}

void update_actuators(Actuator_Set *set, Robot *robot, u64 step, u64 timeout_steps)
{
    for (u32 a = 0; a < set->count; a++) {
        Robot_Actuator *actuator = &set->actuators[a];
        Robot_Joint *joint = &robot->joints[actuator->joint];
        if (actuator->active && timeout_steps > 0 &&
            step - actuator->command_step > timeout_steps) {
            actuator->active = false;
            actuator->timed_out = true;
            log_warning("actuator %u (%s): command expired; holding", actuator->node_id,
                        robot->model.joints[joint->urdf_joint].name);
        }
        bool was_position = joint->drive == DRIVE_POSITION;
        if (actuator->active) {
            set_joint_drive(robot, actuator->joint, get_drive_of_mode(actuator->mode));
            joint->command = actuator->command;
            continue;
        }
        // Hold: stop a velocity joint; keep a position joint where it is aiming, or where
        // it is if it wasn't aiming anywhere.
        set_joint_drive(robot, actuator->joint, joint->default_drive);
        if (joint->default_drive == DRIVE_VELOCITY) {
            joint->command = 0.0f;
        } else if (joint->default_drive == DRIVE_POSITION && !was_position) {
            joint->command = joint->position;
        }
    }
}

u32 get_actuator_telemetry(const Actuator_Set *set, const Robot *robot,
                           Actuator_Telemetry *telemetry, u32 capacity)
{
    u32 count = min(set->count, capacity);
    for (u32 a = 0; a < count; a++) {
        const Robot_Actuator *actuator = &set->actuators[a];
        const Robot_Joint *joint = &robot->joints[actuator->joint];
        telemetry[a] = Actuator_Telemetry{
            .node_id = actuator->node_id,
            .mode = get_mode_of_drive(joint->drive),
            .timed_out = actuator->timed_out,
            .position = joint->position,
            .velocity = joint->velocity,
            .effort = joint->effort,
        };
    }
    return count;
}

void reset_actuators(Actuator_Set *set)
{
    for (u32 a = 0; a < set->count; a++) {
        set->actuators[a].active = false;
        set->actuators[a].timed_out = false;
    }
}
