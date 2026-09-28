#pragma once

#include "can/can_frame.h"
#include "core/actuator.h"

// Nova's BLCMD motor controllers, as the previous (Unity) simulator implemented them. Only
// this file and blcmd.cpp know their protocol.
//
// Nodes 1..15 except 9, the LED strip (the URDF <ros2_control> canid): Banksia's wheels and
// pivots are 1..8, a digger's joints 10 and up. A value is a big-endian 16-bit number in
// bytes 0..1:
//   3 Drive at Speed     signed, rad/s at the joint = 30 · value / 32767
//   4 Drive to Position  signed, rad at the joint = π · (value − zero_offset) / 65535,
//                        zero_offset from the URDF (0x397D if absent). The value is
//                        not wrapped, so with Banksia's offset the range is −2.28 to
//                        +0.87 rad before any reversal.
//   0 Stop               ignored, as the Unity sim ignored it: the last command stands
// Both are negated for a joint whose URDF says reversed=true. The other functions (twitch,
// current, open loop, homing, configuration, reset) aren't simulated. Nothing is sent back.
//
// For example, with can-utils:
//   cansend can0 013#7FFF      node 1 (flw): full speed
//   cansend can0 054#397D      node 5 (flp): position 0

#define BLCMD_FIRST_NODE 1u
#define BLCMD_LAST_NODE 15u
#define BLCMD_SKIPPED_NODE 9u // the LED strip

inline bool is_blcmd_node(u32 node)
{
    return node >= BLCMD_FIRST_NODE && node <= BLCMD_LAST_NODE && node != BLCMD_SKIPPED_NODE;
}
#define BLCMD_FULL_SPEED 30.0f // rad/s at the joint for a speed value of 32767
#define BLCMD_DEFAULT_ZERO_OFFSET 0x397Du

enum Blcmd_Function {
    BLCMD_STOP = 0,
    BLCMD_TWITCH_FORWARD = 1,
    BLCMD_TWITCH_BACKWARD = 2,
    BLCMD_DRIVE_AT_SPEED = 3,
    BLCMD_DRIVE_TO_POSITION = 4,
    BLCMD_DRIVE_AT_CURRENT = 5,
    BLCMD_DRIVE_OPEN_LOOP = 6,
    BLCMD_HOME_ROTOR = 7,
    BLCMD_RESOLVER_ZERO_FLAG = 8,
    BLCMD_GET_CONFIGURATION = 9,
    BLCMD_SET_CONFIGURATION = 10,
    BLCMD_RESET = 11,
    BLCMD_FUNCTION_COUNT = 12,
};

// How one BLCMD's numbers map onto its joint, from the URDF.
struct Blcmd_Node {
    u32 node_id;
    f32 direction; // -1 when reversed
    u16 zero_offset;
};

enum Blcmd_Message_Kind {
    BLCMD_MESSAGE_NONE, // not a BLCMD frame, or malformed
    BLCMD_MESSAGE_COMMAND, // .command is for the node's actuator
    BLCMD_MESSAGE_UNSUPPORTED, // a function the sim doesn't simulate
    BLCMD_MESSAGE_IGNORED, // Stop: ignored without a warning, as the Unity sim did
};

struct Blcmd_Message {
    Blcmd_Message_Kind kind;
    u32 node_id;
    u32 function;
    Actuator_Command command;
};

bool is_blcmd_frame(const Can_Frame *frame);

// An actuator's BLCMD mapping, from its URDF <ros2_control> parameters (reversed,
// zero_offset). False if its node id isn't a BLCMD address.
bool make_blcmd_node(const Robot *robot, const Robot_Actuator *actuator, Blcmd_Node *node);

// The node table entry for a node id, or NULL.
const Blcmd_Node *find_blcmd_node(const Blcmd_Node *nodes, u32 count, u32 node_id);

// Decodes one frame. node is the entry for the frame's node id (NULL if the robot has no
// such BLCMD); a command for a missing BLCMD still decodes, with the default mapping, so
// the caller can report it.
Blcmd_Message decode_blcmd_frame(const Can_Frame *frame, const Blcmd_Node *node);

// For tools and tests: the frames a driver would send.
Can_Frame encode_speed_frame(const Blcmd_Node *node, f32 joint_radps);
Can_Frame encode_position_frame(const Blcmd_Node *node, f32 joint_rad);

const char *get_blcmd_function_name(u32 function);
