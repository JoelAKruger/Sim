#include "can/led_strip.h"

enum Led_Strip_Function {
    LED_BRIGHTNESS = 1,
    LED_RED = 2,
    LED_GREEN = 3,
    LED_BLUE = 4,
    LED_PRESET = 5,
    LED_ALL = 6,
    LED_PINK = 7,
    LED_RESET = 8,
};

bool is_led_strip_frame(const Can_Frame *frame)
{
    return frame->id <= 0xff && get_frame_node(frame) == LED_STRIP_NODE;
}

bool decode_led_strip_frame(const Can_Frame *frame, Led_Strip_Command *command)
{
    if (!is_led_strip_frame(frame)) {
        return false;
    }
    u32 function = get_frame_function(frame);
    u32 needed = function == LED_ALL ? 2 : function <= LED_PRESET ? 1 : 0;
    if (function < LED_BRIGHTNESS || function > LED_RESET || frame->length < needed) {
        return false;
    }
    command->function = function;
    command->data[0] = frame->length > 0 ? frame->data[0] : 0;
    command->data[1] = frame->length > 1 ? frame->data[1] : 0;
    return true;
}

Led_Strip_State get_default_led_strip_state(void) { return Led_Strip_State{0.0f, 0.0f, 0.0f}; }

// Ported line for line from the Unity sim's HandleLEDCommand.
void apply_led_strip_command(Led_Strip_State *state, const Led_Strip_Command *command)
{
    const u8 *data = command->data;
    f32 value = (f32)data[0];
    switch (command->function) {
    case LED_BRIGHTNESS:
        // Every channel that is on takes value / 128.
        state->red = state->red != 0.0f ? value / 128.0f : 0.0f;
        state->green = state->green != 0.0f ? value / 128.0f : 0.0f;
        state->blue = state->blue != 0.0f ? value / 128.0f : 0.0f;
        break;
    case LED_RED:
        state->red = value;
        break;
    case LED_GREEN:
        state->green = value;
        break;
    case LED_BLUE:
        state->blue = value;
        break;
    case LED_PRESET:
        if (data[0] >= 1 && data[0] <= 3) {
            state->red = data[0] == 1 ? 1.0f : 0.0f;
            state->green = data[0] == 2 ? 1.0f : 0.0f;
            state->blue = data[0] == 3 ? 1.0f : 0.0f;
        }
        break;
    case LED_ALL:
        state->red = (f32)(data[0] >> 4) / 15.0f;
        state->green = (f32)(data[1] >> 4) / 15.0f;
        state->blue = (f32)(data[0] & 0xf) / 15.0f;
        break;
    case LED_PINK:
        *state = Led_Strip_State{1.0f, 0.0f, 0.5f};
        break;
    case LED_RESET:
        *state = get_default_led_strip_state();
        break;
    default:
        break;
    }
}

Status_Light get_led_strip_light(const Led_Strip_State *state)
{
    return Status_Light{.red = min(max(state->red, 0.0f), 1.0f),
                        .green = min(max(state->green, 0.0f), 1.0f),
                        .blue = min(max(state->blue, 0.0f), 1.0f)};
}
