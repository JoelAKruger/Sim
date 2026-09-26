#pragma once

#include "core/common.h"

// A classic CAN data frame, independent of the socket API so the device codecs (BLCMDs,
// the LED strip) and their tests don't need SocketCAN.
//
// Every Nova device on the bus uses 11-bit ids of the form node << 4 | function, with bits
// 8..10 clear: nodes 1..8 are BLCMDs, node 9 the LED strip.
struct Can_Frame {
    u32 id; // 11-bit
    u8 length; // 0..8
    u8 data[8];
};

inline u32 get_frame_node(const Can_Frame *frame) { return frame->id >> 4; }
inline u32 get_frame_function(const Can_Frame *frame) { return frame->id & 0xf; }
