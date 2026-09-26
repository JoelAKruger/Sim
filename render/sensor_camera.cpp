#include "render/sensor_camera.h"

#include <raymath.h>
#include <rlgl.h>
#include <string.h>

// The viewer's clip planes, restored after each sensor render.
#define VIEWER_NEAR 0.05
#define VIEWER_FAR 2000.0

static const char *depth_vertex_shader = R"glsl(
#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
void main()
{
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

// Linear depth from the depth buffer, in whole millimetres split over red (high byte) and
// green (low byte). Anything outside the depth range reads 0, as on a RealSense.
static const char *depth_fragment_shader = R"glsl(
#version 330
uniform float near_plane;
uniform float far_plane;
uniform vec2 depth_range;
out vec4 final_color;
void main()
{
    float ndc = gl_FragCoord.z * 2.0 - 1.0;
    float z = 2.0 * near_plane * far_plane / (far_plane + near_plane - ndc * (far_plane - near_plane));
    float mm = (z < depth_range.x || z > depth_range.y) ? 0.0 : min(floor(z * 1000.0 + 0.5), 65535.0);
    final_color = vec4(floor(mm / 256.0) / 255.0, mod(mm, 256.0) / 255.0, 0.0, 1.0);
}
)glsl";

bool create_sensor_camera(Sensor_Camera *camera, const Camera_Config *config, const World *world,
                          char *error, u32 error_size)
{
    *camera = {};
    camera->config = config;
    char link[CONFIG_STRING_SIZE + 16];
    make_camera_frame_name(config->name, "_link", link, sizeof(link));
    if (!find_sensor_mount(&world->robot, link, "camera.name", &camera->mount, error, error_size)) {
        return false;
    }
    const f32 *offset = config->color_offset;
    camera->color_in_link = get_optical_transform(v3{offset[0], offset[1], offset[2]});
    camera->depth_in_link = get_optical_transform(v3{0.0f, 0.0f, 0.0f});
    u32 width = config->resolution[0];
    u32 height = config->resolution[1];
    camera->color = make_camera_intrinsics(width, height, config->horizontal_fov);
    camera->depth = make_camera_intrinsics(width, height, config->depth_horizontal_fov);
    camera->color_target = LoadRenderTexture((i32)width, (i32)height);
    camera->depth_target = LoadRenderTexture((i32)width, (i32)height);
    camera->depth_shader = LoadShaderFromMemory(depth_vertex_shader, depth_fragment_shader);
    camera->near_location = GetShaderLocation(camera->depth_shader, "near_plane");
    camera->far_location = GetShaderLocation(camera->depth_shader, "far_plane");
    camera->range_location = GetShaderLocation(camera->depth_shader, "depth_range");
    camera->depth_material = LoadMaterialDefault();
    camera->depth_material.shader = camera->depth_shader;
    camera->staging = (u8 *)MemAlloc(width * height * 4);
    return true;
}

void destroy_sensor_camera(Sensor_Camera *camera)
{
    UnloadRenderTexture(camera->color_target);
    UnloadRenderTexture(camera->depth_target);
    UnloadMaterial(camera->depth_material); // also unloads the depth shader
    MemFree(camera->staging);
    *camera = {};
}

static u64 get_frame_time_ns(const Sensor_Camera *camera, u64 frame)
{
    return (u64)((f64)frame * 1e9 / (f64)camera->config->rate);
}

bool is_sensor_camera_due(const Sensor_Camera *camera, u64 sim_time_ns)
{
    return sim_time_ns >= get_frame_time_ns(camera, camera->next_frame);
}

// A Raylib camera looking along +z of an optical frame, with -y up.
static Camera3D make_optical_camera(b3Transform pose, const Camera_Intrinsics *intrinsics)
{
    v3 forward = b3RotateVector(pose.q, v3{0.0f, 0.0f, 1.0f});
    v3 up = b3RotateVector(pose.q, v3{0.0f, -1.0f, 0.0f});
    Camera3D camera = {};
    camera.position = Vector3{pose.p.x, pose.p.y, pose.p.z};
    camera.target = Vector3{pose.p.x + forward.x, pose.p.y + forward.y, pose.p.z + forward.z};
    camera.up = Vector3{up.x, up.y, up.z};
    camera.fovy = get_vertical_fov_deg(intrinsics);
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

// Reads a render target back, top row first (GL stores it bottom-up).
static void read_target(const RenderTexture2D *target, u8 *rgba, u32 width, u32 height)
{
    u8 *pixels = (u8 *)rlReadTexturePixels(target->texture.id, (i32)width, (i32)height,
                                           target->texture.format);
    if (!pixels) {
        memset(rgba, 0, (u64)width * height * 4);
        return;
    }
    for (u32 row = 0; row < height; row++) {
        memcpy(rgba + (u64)row * width * 4, pixels + (u64)(height - 1 - row) * width * 4,
               (u64)width * 4);
    }
    MemFree(pixels);
}

void render_sensor_camera(Sensor_Camera *camera, const Viewer *viewer, const World *world,
                          Camera_Frame_Header *frame)
{
    const Camera_Config *config = camera->config;
    u32 width = camera->color.width;
    u32 height = camera->color.height;
    u64 now = get_sim_time_ns(world);
    while (is_sensor_camera_due(camera, now)) {
        camera->next_frame++; // a slow renderer skips frames rather than falling behind
    }

    b3Transform link_pose = get_sensor_pose(&world->robot, &camera->mount);

    // Colour: the lit scene, as the viewer draws it.
    Material color_material = viewer->material;
    BeginTextureMode(camera->color_target);
    ClearBackground(BLACK);
    rlSetClipPlanes(VIEWER_NEAR, VIEWER_FAR);
    BeginMode3D(
        make_optical_camera(b3MulTransforms(link_pose, camera->color_in_link), &camera->color));
    draw_scene(viewer, world, 1.0f, &color_material, false);
    EndMode3D();
    EndTextureMode();

    // Depth: the same scene through the depth shader, with clip planes just outside the
    // depth range so the depth buffer spends its precision where it counts.
    f32 near_plane = 0.5f * config->depth_range[0];
    f32 far_plane = config->depth_range[1] + 1.0f;
    SetShaderValue(camera->depth_shader, camera->near_location, &near_plane, SHADER_UNIFORM_FLOAT);
    SetShaderValue(camera->depth_shader, camera->far_location, &far_plane, SHADER_UNIFORM_FLOAT);
    SetShaderValue(camera->depth_shader, camera->range_location, config->depth_range,
                   SHADER_UNIFORM_VEC2);
    BeginTextureMode(camera->depth_target);
    ClearBackground(BLANK); // no data
    rlSetClipPlanes(near_plane, far_plane);
    BeginMode3D(
        make_optical_camera(b3MulTransforms(link_pose, camera->depth_in_link), &camera->depth));
    BeginShaderMode(camera->depth_shader); // for anything drawn without a material
    draw_scene(viewer, world, 1.0f, &camera->depth_material, false);
    EndShaderMode();
    EndMode3D();
    EndTextureMode();
    rlSetClipPlanes(VIEWER_NEAR, VIEWER_FAR);

    // Both back into the frame, through the RGBA staging buffer.
    u8 *rgba = camera->staging;
    frame->stamp_ns = now;
    frame->width = width;
    frame->height = height;
    u8 *rgb = get_camera_frame_rgb(frame);
    read_target(&camera->color_target, rgba, width, height);
    for (u64 i = 0; i < (u64)width * height; i++) {
        rgb[i * 3 + 0] = rgba[i * 4 + 0];
        rgb[i * 3 + 1] = rgba[i * 4 + 1];
        rgb[i * 3 + 2] = rgba[i * 4 + 2];
    }
    u16 *depth = get_camera_frame_depth(frame);
    read_target(&camera->depth_target, rgba, width, height);
    for (u64 i = 0; i < (u64)width * height; i++) {
        depth[i] = unpack_depth_mm(rgba[i * 4 + 0], rgba[i * 4 + 1]);
    }
}
