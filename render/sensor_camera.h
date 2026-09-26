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
struct Sensor_Camera {
    const Camera_Config *config;
    Sensor_Mount mount; // <name>_link
    b3Transform color_in_link; // the colour optical frame in <name>_link
    b3Transform depth_in_link; // the depth optical frame (at the link's origin)
    Camera_Intrinsics color;
    Camera_Intrinsics depth;
    RenderTexture2D color_target;
    RenderTexture2D depth_target;
    Shader depth_shader;
    Material depth_material;
    i32 near_location;
    i32 far_location;
    i32 range_location;
    u64 next_frame; // index of the next frame slot
    u8 *staging; // one RGBA image, for readback
};

bool create_sensor_camera(Sensor_Camera *camera, const Camera_Config *config, const World *world,
                          char *error, u32 error_size);
void destroy_sensor_camera(Sensor_Camera *camera);

// True when a frame slot has come due at this sim time.
bool is_sensor_camera_due(const Sensor_Camera *camera, u64 sim_time_ns);

// Renders both images at the world's current state into frame, which must hold
// get_camera_frame_size bytes, and moves on to the next slot.
void render_sensor_camera(Sensor_Camera *camera, const Viewer *viewer, const World *world,
                          Camera_Frame_Header *frame);
