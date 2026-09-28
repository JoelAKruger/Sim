#pragma once

#include "core/sensors/camera_model.h"
#include "core/sensors/mount.h"
#include "render/viewer.h"

// The RGB-D camera, rendered with the viewer's own scene into off-screen targets.
//
// Colour is the lit scene. Depth is the same scene drawn with a shader that writes
// millimetres along the optical axis into two bytes of each pixel, so it comes back exact.
// The camera is mounted on the URDF link <name>_link (x forward, z up), and each image
// looks along +z of its optical frame (x right, y down), derived from that link as
// realsense2_camera does. It needs the GL context the viewer opened.
// One stream (colour or depth): where it looks from, its pinhole model and its target.
struct Camera_Stream {
    const Camera_Stream_Config *config;
    Pose in_link; // its optical frame in <name>_link
    Camera_Intrinsics intrinsics;
    RenderTexture2D target; // only when the stream is enabled
    u64 next_frame; // index of the next frame slot
};

struct Sensor_Camera {
    const Camera_Config *config;
    Sensor_Mount mount; // <name>_link
    Camera_Stream color; // optical frame at color_offset
    Camera_Stream depth; // optical frame at the link's origin
    Shader depth_shader;
    Material depth_material;
    i32 near_location;
    i32 far_location;
    i32 range_location;
    u8 *staging; // one RGBA image of the larger stream, for readback
};

bool create_sensor_camera(Sensor_Camera *camera, const Camera_Config *config, const World *world,
                          char *error, u32 error_size);
void destroy_sensor_camera(Sensor_Camera *camera);

// True when the stream is enabled and a frame slot has come due at this sim time.
bool is_camera_stream_due(const Camera_Stream *stream, u64 sim_time_ns);

// Render one stream at the world's current state into frame, which must hold
// get_color_frame_size or get_depth_frame_size bytes, and move it on to its next slot.
void render_color_frame(Sensor_Camera *camera, const Viewer *viewer, const World *world,
                        Camera_Frame_Header *frame);
void render_depth_frame(Sensor_Camera *camera, const Viewer *viewer, const World *world,
                        Camera_Frame_Header *frame);
