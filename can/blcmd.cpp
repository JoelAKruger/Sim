#include "can/blcmd.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const Blcmd_Node default_node = {
    .node_id = 0, .direction = 1.0f, .zero_offset = BLCMD_DEFAULT_ZERO_OFFSET};

static const char *function_names[BLCMD_FUNCTION_COUNT] = {
    "Stop",
    "Twitch Forward",
    "Twitch Backward",
    "Drive at Speed",
    "Drive to Position",
    "Drive at Current",
    "Drive Open Loop",
    "Home Rotor",
    "Resolver Zero Flag",
    "Get Configuration",
    "Set Configuration",
    "Reset BLCMD",
};

const char *get_blcmd_function_name(u32 function)
{
    return function < BLCMD_FUNCTION_COUNT ? function_names[function] : "unknown";
}

bool is_blcmd_frame(const Can_Frame *frame)
{
    u32 node = get_frame_node(frame);
    return frame->id <= 0xff && is_blcmd_node(node);
}

bool make_blcmd_node(const Robot *robot, const Robot_Actuator *actuator, Blcmd_Node *node)
{
    if (!is_blcmd_node(actuator->node_id)) {
        return false;
    }
    const Urdf_Joint *urdf = &robot->model.joints[robot->joints[actuator->joint].urdf_joint];
    const char *reversed = get_control_param(&robot->model, urdf->control, "reversed");
    const char *zero_offset = get_control_param(&robot->model, urdf->control, "zero_offset");
    node->node_id = actuator->node_id;
    node->direction = reversed && strcmp(reversed, "true") == 0 ? -1.0f : 1.0f;
    node->zero_offset =
        zero_offset ? (u16)strtoul(zero_offset, NULL, 0) : (u16)BLCMD_DEFAULT_ZERO_OFFSET;
    return true;
}

const Blcmd_Node *find_blcmd_node(const Blcmd_Node *nodes, u32 count, u32 node_id)
{
    for (u32 i = 0; i < count; i++) {
        if (nodes[i].node_id == node_id) {
            return &nodes[i];
        }
    }
    return NULL;
}

static u16 read_u16(const u8 *data) { return (u16)(data[0] << 8 | data[1]); }

Blcmd_Message decode_blcmd_frame(const Can_Frame *frame, const Blcmd_Node *node)
{
    Blcmd_Message message = {};
    if (!is_blcmd_frame(frame)) {
        return message;
    }
    message.node_id = get_frame_node(frame);
    message.function = get_frame_function(frame);
    const Blcmd_Node *mapping = node ? node : &default_node;
    message.command.node_id = message.node_id;
    switch (message.function) {
    case BLCMD_STOP:
        message.kind = BLCMD_MESSAGE_IGNORED; // as the Unity sim did: the joint carries on
        return message;
    case BLCMD_DRIVE_AT_SPEED: {
        if (frame->length < 2) {
            return message;
        }
        i16 value = (i16)read_u16(frame->data);
        message.command.mode = ACTUATOR_VELOCITY;
        message.command.value = mapping->direction * BLCMD_FULL_SPEED * (f32)value / 32767.0f;
        break;
    }
    case BLCMD_DRIVE_TO_POSITION: {
        if (frame->length < 2) {
            return message;
        }
        i32 counts = (i32)(i16)read_u16(frame->data) - (i32)mapping->zero_offset;
        message.command.mode = ACTUATOR_POSITION;
        message.command.value = mapping->direction * PI_F32 * (f32)counts / 65535.0f;
        break;
    }
    default:
        message.kind = message.function < BLCMD_FUNCTION_COUNT ? BLCMD_MESSAGE_UNSUPPORTED
                                                               : BLCMD_MESSAGE_NONE;
        return message;
    }
    message.kind = BLCMD_MESSAGE_COMMAND;
    return message;
}

static Can_Frame make_blcmd_frame(u32 node_id, u32 function, u16 value)
{
    Can_Frame frame = {.id = node_id << 4 | function, .length = 2};
    frame.data[0] = (u8)(value >> 8);
    frame.data[1] = (u8)value;
    return frame;
}

Can_Frame encode_speed_frame(const Blcmd_Node *node, f32 joint_radps)
{
    f32 scaled = node->direction * joint_radps / BLCMD_FULL_SPEED * 32767.0f;
    i16 value = (i16)lroundf(min(max(scaled, -32767.0f), 32767.0f));
    return make_blcmd_frame(node->node_id, BLCMD_DRIVE_AT_SPEED, (u16)value);
}

Can_Frame encode_position_frame(const Blcmd_Node *node, f32 joint_rad)
{
    f32 value = (f32)node->zero_offset + node->direction * joint_rad * 65535.0f / PI_F32;
    i16 clamped = (i16)lroundf(min(max(value, -32768.0f), 32767.0f));
    return make_blcmd_frame(node->node_id, BLCMD_DRIVE_TO_POSITION, (u16)clamped);
}
