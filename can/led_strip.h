#pragma once

#include "can/can_frame.h"
#include "core/status_light.h"

// The rover's LED strip (ledstrip_v3 firmware), as the previous (Unity) simulator handled
// it. Only this file and led_strip.cpp know its protocol.
//
// Node 9; the function is the low four bits of the id:
//   1 brightness   byte 0 / 128 for every channel that is on
//   2 red          byte 0, raw (so any non-zero value shows full on)
//   3 green        byte 0, raw
//   4 blue         byte 0, raw
//   5 preset       byte 0: 1 red, 2 green, 3 blue
//   6 all          four bits each: red in the top of byte 0, blue in the bottom of byte 0,
//                  green in the top of byte 1
//   7 pink
//   8 reset (off)
//
// For example, with can-utils:
//   cansend can0 095#02        green

#define LED_STRIP_NODE 9u

// The strip's colour exactly as the Unity sim kept it: unclamped channels.
struct Led_Strip_State {
    f32 red;
    f32 green;
    f32 blue;
};

struct Led_Strip_Command {
    u32 function;
    u8 data[2];
};

bool is_led_strip_frame(const Can_Frame *frame);

// False for anything that isn't a well-formed LED strip command.
bool decode_led_strip_frame(const Can_Frame *frame, Led_Strip_Command *command);

Led_Strip_State get_default_led_strip_state(void);
void apply_led_strip_command(Led_Strip_State *state, const Led_Strip_Command *command);

// What the strip shows, each channel clamped to 0..1.
Status_Light get_led_strip_light(const Led_Strip_State *state);
