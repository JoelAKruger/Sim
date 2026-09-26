#pragma once

#include <math.h>
#include <stdio.h>

#include "core/common.h"

// The pinhole model and image formats of the simulated RGB-D camera, apart from the GL
// code that renders it, so they can be tested without a window.

// The rotation from a body frame (x forward, y left, z up) to an optical frame (z forward,
// x right, y down): roll -90°, then yaw -90°, as realsense2_camera builds it.
inline b3Quat get_optical_rotation(void) { return b3Quat{{-0.5f, 0.5f, -0.5f}, 0.5f}; }

// An optical frame in the camera's <name>_link, from the sensor's offset in that link.
inline b3Transform get_optical_transform(v3 offset)
{
    return b3Transform{offset, get_optical_rotation()};
}

// The driver's frame names: <name><suffix>, e.g. "d415" and "_color_optical_frame".
inline void make_camera_frame_name(const char *name, const char *suffix, char *out, u32 size)
{
    snprintf(out, size, "%s%s", name, suffix);
}

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

// A frame as it crosses to the ROS thread: this header, then the pixels, rows top first:
// rgb8 for colour, 16-bit millimetres for depth.
struct Camera_Frame_Header {
    u64 stamp_ns; // sim time
    u32 width;
    u32 height;
};

inline u64 get_color_frame_size(u32 width, u32 height)
{
    return sizeof(Camera_Frame_Header) + (u64)width * height * 3;
}
inline u64 get_depth_frame_size(u32 width, u32 height)
{
    return sizeof(Camera_Frame_Header) + (u64)width * height * 2;
}
inline u8 *get_color_pixels(Camera_Frame_Header *frame) { return (u8 *)(frame + 1); }
inline u16 *get_depth_pixels(Camera_Frame_Header *frame) { return (u16 *)(frame + 1); }
