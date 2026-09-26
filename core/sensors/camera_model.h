#pragma once

#include <math.h>

#include "core/common.h"

// The pinhole model and image formats of the simulated RGB-D camera, apart from the GL
// code that renders it, so they can be tested without a window.

// OpenCV/ROS convention: pixel centres are at whole numbers, so the centre of a w-wide
// image is at (w - 1) / 2. Pixels are square.
struct Camera_Intrinsics {
    u32 width;
    u32 height;
    f32 fx;
    f32 fy;
    f32 cx;
    f32 cy;
};

inline Camera_Intrinsics make_camera_intrinsics(u32 width, u32 height, f32 horizontal_fov_deg)
{
    f32 half = 0.5f * horizontal_fov_deg * PI_F32 / 180.0f;
    f32 focal = 0.5f * (f32)width / tanf(half);
    return Camera_Intrinsics{.width = width,
                             .height = height,
                             .fx = focal,
                             .fy = focal,
                             .cx = 0.5f * (f32)(width - 1),
                             .cy = 0.5f * (f32)(height - 1)};
}

inline f32 get_vertical_fov_deg(const Camera_Intrinsics *camera)
{
    return 2.0f * atanf(0.5f * (f32)camera->height / camera->fy) * 180.0f / PI_F32;
}

// Depth as a RealSense reports it: millimetres along the optical axis, 0 for no data
// (nearer than near or beyond far).
inline u16 encode_depth_mm(f32 metres, f32 near, f32 far)
{
    if (!(metres >= near && metres <= far)) {
        return 0;
    }
    return (u16)min(roundf(metres * 1000.0f), 65535.0f);
}

// The depth pass writes millimetres into the red (high byte) and green (low byte) channels.
inline u16 unpack_depth_mm(u8 high, u8 low) { return (u16)(high << 8 | low); }

// A frame as it crosses to the ROS thread: this header, then width·height·3 bytes of rgb8,
// padded to an even length, then width·height 16-bit depths; rows top first.
struct Camera_Frame_Header {
    u64 stamp_ns; // sim time
    u32 width;
    u32 height;
};

inline u64 get_camera_rgb_size(u32 width, u32 height)
{
    return ((u64)width * height * 3 + 1) & ~1ull;
}

inline u64 get_camera_frame_size(u32 width, u32 height)
{
    return sizeof(Camera_Frame_Header) + get_camera_rgb_size(width, height) +
           (u64)width * height * 2;
}
inline u8 *get_camera_frame_rgb(Camera_Frame_Header *frame) { return (u8 *)(frame + 1); }
inline u16 *get_camera_frame_depth(Camera_Frame_Header *frame)
{
    return (u16 *)(get_camera_frame_rgb(frame) + get_camera_rgb_size(frame->width, frame->height));
}
