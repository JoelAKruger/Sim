#include <math.h>
#include <string.h>

#include "can/blcmd.h"
#include "tests/check.h"

// Banksia's front-left wheel (reversed) and front-right pivot (not reversed).
static const Blcmd_Node flw = {.node_id = 1, .direction = -1.0f, .zero_offset = 0x397D};
static const Blcmd_Node frp = {.node_id = 8, .direction = 1.0f, .zero_offset = 0x397D};

static Can_Frame make_frame(u32 id, u8 length, u8 byte0, u8 byte1)
{
    Can_Frame frame = {.id = id, .length = length};
    frame.data[0] = byte0;
    frame.data[1] = byte1;
    return frame;
}

// The examples in blcmd.h.
static void test_documented_examples(void)
{
    Can_Frame full_speed = make_frame(0x013, 2, 0x7f, 0xff);
    Blcmd_Message message = decode_blcmd_frame(&full_speed, &flw);
    CHECK(message.kind == BLCMD_MESSAGE_COMMAND);
    CHECK(message.command.node_id == 1 && message.command.mode == ACTUATOR_VELOCITY);
    CHECK_NEAR(message.command.value, -30.0, 1e-5); // reversed

    const Blcmd_Node flp = {.node_id = 5, .direction = -1.0f, .zero_offset = 0x397D};
    Can_Frame zero = make_frame(0x054, 2, 0x39, 0x7d);
    message = decode_blcmd_frame(&zero, &flp);
    CHECK(message.kind == BLCMD_MESSAGE_COMMAND);
    CHECK(message.command.node_id == 5 && message.command.mode == ACTUATOR_POSITION);
    CHECK(message.command.value == 0.0f);
}

// The conversions the Unity sim used, including its sign flips.
static void test_matches_unity(void)
{
    // Drive at Speed: 30 · value / Int16.MaxValue, big-endian.
    i16 speeds[] = {1000, -1000, 32767, -32768, 0};
    for (u32 i = 0; i < ARRAY_COUNT(speeds); i++) {
        u16 value = (u16)speeds[i];
        Can_Frame frame = make_frame(0x043, 2, (u8)(value >> 8), (u8)value); // frw: not reversed
        const Blcmd_Node frw = {.node_id = 4, .direction = 1.0f, .zero_offset = 0x397D};
        Blcmd_Message message = decode_blcmd_frame(&frame, &frw);
        CHECK_NEAR(message.command.value, 30.0 * speeds[i] / 32767.0, 1e-5);
    }
    // Drive to Position: π · (value − 0x397D) / UInt16.MaxValue, value signed, no wrap.
    i16 positions[] = {0x397D, 0x397D + 1000, 0x397D - 20000, -18052, -25000, -32768, 0x7fff};
    for (u32 i = 0; i < ARRAY_COUNT(positions); i++) {
        u16 value = (u16)positions[i];
        Can_Frame frame = make_frame(0x084, 2, (u8)(value >> 8), (u8)value);
        Blcmd_Message message = decode_blcmd_frame(&frame, &frp);
        CHECK_NEAR(message.command.value, M_PI * (positions[i] - 0x397D) / 65535.0, 1e-5);
    }
}

// Positions over the whole signed range, far past −π/2 (the pivots' limits are ±π).
static void test_position_round_trip(void)
{
    f32 angles[] = {-2.2f, -1.8f, -1.5707f, -0.9f, 0.0f, 0.5f, 0.85f};
    for (u32 i = 0; i < ARRAY_COUNT(angles); i++) {
        Can_Frame frame = encode_position_frame(&frp, angles[i]);
        Blcmd_Message message = decode_blcmd_frame(&frame, &frp);
        CHECK(message.kind == BLCMD_MESSAGE_COMMAND);
        CHECK_NEAR(message.command.value, angles[i], 1e-4);

        // A reversed pivot takes the mirrored range.
        frame = encode_position_frame(&flw, -angles[i]);
        message = decode_blcmd_frame(&frame, &flw);
        CHECK_NEAR(message.command.value, -angles[i], 1e-4);
    }
}

static void test_speed_round_trip(void)
{
    f32 speeds[] = {-30.0f, -2.5f, 0.0f, 0.001f, 12.0f, 30.0f};
    for (u32 i = 0; i < ARRAY_COUNT(speeds); i++) {
        Can_Frame frame = encode_speed_frame(&flw, speeds[i]);
        CHECK(frame.id == 0x013 && frame.length == 2);
        Blcmd_Message message = decode_blcmd_frame(&frame, &flw);
        CHECK_NEAR(message.command.value, speeds[i], 30.0 / 32767.0);
    }
}

static void test_other_frames(void)
{
    Can_Frame stop = make_frame(0x020, 0, 0, 0);
    Blcmd_Message message = decode_blcmd_frame(&stop, NULL);
    CHECK(message.kind == BLCMD_MESSAGE_IGNORED); // as in Unity
    CHECK(message.node_id == 2 && message.function == BLCMD_STOP);

    Can_Frame current = make_frame(0x015, 2, 0x10, 0x00);
    CHECK(decode_blcmd_frame(&current, &flw).kind == BLCMD_MESSAGE_UNSUPPORTED);
    Can_Frame reset = make_frame(0x01b, 0, 0, 0);
    CHECK(decode_blcmd_frame(&reset, &flw).kind == BLCMD_MESSAGE_UNSUPPORTED);
    Can_Frame no_function = make_frame(0x01f, 2, 0, 0);
    CHECK(decode_blcmd_frame(&no_function, &flw).kind == BLCMD_MESSAGE_NONE);

    Can_Frame short_speed = make_frame(0x013, 1, 0x7f, 0);
    CHECK(decode_blcmd_frame(&short_speed, &flw).kind == BLCMD_MESSAGE_NONE);
    Can_Frame high_id = make_frame(0x113, 2, 0x7f, 0xff); // bits 8..10 set: not ours
    CHECK(decode_blcmd_frame(&high_id, &flw).kind == BLCMD_MESSAGE_NONE);
    Can_Frame node_zero = make_frame(0x003, 2, 0x7f, 0xff);
    CHECK(decode_blcmd_frame(&node_zero, NULL).kind == BLCMD_MESSAGE_NONE);
    Can_Frame node_ten = make_frame(0x0a3, 2, 0x7f, 0xff);
    CHECK(decode_blcmd_frame(&node_ten, NULL).kind == BLCMD_MESSAGE_NONE);
}

int main(void)
{
    test_documented_examples();
    test_matches_unity();
    test_position_round_trip();
    test_speed_round_trip();
    test_other_frames();
    return report_checks("blcmd");
}
