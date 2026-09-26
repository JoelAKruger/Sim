#include "can/led_strip.h"
#include "tests/check.h"

static Can_Frame make_frame(u32 id, u8 length, u8 byte0, u8 byte1)
{
    Can_Frame frame = {.id = id, .length = length};
    frame.data[0] = byte0;
    frame.data[1] = byte1;
    return frame;
}

static void apply(Led_Strip_State *state, u32 function, u8 byte0, u8 byte1)
{
    Led_Strip_Command command = {.function = function, .data = {byte0, byte1}};
    apply_led_strip_command(state, &command);
}

// The LED strip, as the Unity sim handled it.
static void test_commands(void)
{
    Led_Strip_State state = get_default_led_strip_state();
    Status_Light shown = get_led_strip_light(&state);
    CHECK(shown.red == 0.0f && shown.green == 0.0f && shown.blue == 0.0f);

    Can_Frame preset_green = make_frame(0x095, 1, 2, 0); // the example in led_strip.h
    Led_Strip_Command command;
    CHECK(decode_led_strip_frame(&preset_green, &command));
    CHECK(command.function == 5 && command.data[0] == 2);
    apply_led_strip_command(&state, &command);
    shown = get_led_strip_light(&state);
    CHECK(shown.red == 0.0f && shown.green == 1.0f && shown.blue == 0.0f);

    apply(&state, 1, 64, 0); // brightness: lit channels take 64 / 128
    shown = get_led_strip_light(&state);
    CHECK(shown.red == 0.0f && shown.green == 0.5f && shown.blue == 0.0f);

    apply(&state, 2, 10, 0); // the raw byte: any non-zero value is full on
    CHECK(state.red == 10.0f && get_led_strip_light(&state).red == 1.0f);

    apply(&state, 6, 0xf3, 0x80); // red 15, blue 3, green 8 (of 15)
    shown = get_led_strip_light(&state);
    CHECK(shown.red == 1.0f);
    CHECK_NEAR(shown.green, 8.0 / 15.0, 1e-6);
    CHECK_NEAR(shown.blue, 3.0 / 15.0, 1e-6);

    apply(&state, 7, 0, 0); // pink
    shown = get_led_strip_light(&state);
    CHECK(shown.red == 1.0f && shown.green == 0.0f && shown.blue == 0.5f);
    apply(&state, 8, 0, 0); // reset
    shown = get_led_strip_light(&state);
    CHECK(shown.red == 0.0f && shown.green == 0.0f && shown.blue == 0.0f);
}

static void test_rejects(void)
{
    Led_Strip_Command command;
    Can_Frame all_too_short = make_frame(0x096, 1, 0xff, 0);
    CHECK(!decode_led_strip_frame(&all_too_short, &command));
    Can_Frame unknown_function = make_frame(0x099, 1, 0, 0);
    CHECK(!decode_led_strip_frame(&unknown_function, &command));
    Can_Frame blcmd = make_frame(0x013, 2, 0x7f, 0xff);
    CHECK(!decode_led_strip_frame(&blcmd, &command));
    Can_Frame high_id = make_frame(0x195, 1, 2, 0); // bits 8..10 set: not a Nova device
    CHECK(!decode_led_strip_frame(&high_id, &command));
}

int main(void)
{
    test_commands();
    test_rejects();
    return report_checks("led_strip");
}
