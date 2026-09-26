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

static void create_stream(Camera_Stream *stream, const Camera_Stream_Config *config, v3 offset)
{
    stream->config = config;
    stream->in_link = get_optical_transform(offset);
    stream->intrinsics = make_camera_intrinsics(config->resolution[0], config->resolution[1],
                                                config->horizontal_fov);
    if (config->enabled) {
        stream->target = LoadRenderTexture((i32)config->resolution[0], (i32)config->resolution[1]);
    }
}

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
    create_stream(&camera->color, &config->color, v3{offset[0], offset[1], offset[2]});
    create_stream(&camera->depth, &config->depth, v3{0.0f, 0.0f, 0.0f});
    camera->depth_shader = LoadShaderFromMemory(depth_vertex_shader, depth_fragment_shader);
    camera->near_location = GetShaderLocation(camera->depth_shader, "near_plane");
    camera->far_location = GetShaderLocation(camera->depth_shader, "far_plane");
    camera->range_location = GetShaderLocation(camera->depth_shader, "depth_range");
    camera->depth_material = LoadMaterialDefault();
    camera->depth_material.shader = camera->depth_shader;
    u64 pixels = 0;
    for (u32 i = 0; i < 2; i++) {
        const Camera_Stream_Config *stream = i == 0 ? &config->color : &config->depth;
        if (stream->enabled) {
            pixels = max(pixels, (u64)stream->resolution[0] * stream->resolution[1]);
        }
    }
    camera->staging = (u8 *)MemAlloc((u32)(pixels * 4));
    return true;
}

void destroy_sensor_camera(Sensor_Camera *camera)
{
    if (camera->color.config->enabled) {
        UnloadRenderTexture(camera->color.target);
    }
    if (camera->depth.config->enabled) {
        UnloadRenderTexture(camera->depth.target);
    }
    UnloadMaterial(camera->depth_material); // also unloads the depth shader
    MemFree(camera->staging);
    *camera = {};
}

static u64 get_frame_time_ns(const Camera_Stream *stream, u64 frame)
{
    return (u64)((f64)frame * 1e9 / (f64)stream->config->rate);
}

bool is_camera_stream_due(const Camera_Stream *stream, u64 sim_time_ns)
{
    return stream->config->enabled && sim_time_ns >= get_frame_time_ns(stream, stream->next_frame);
}

// Moves the stream past every slot up to now: a slow renderer skips frames rather than
// falling behind. Fills in the frame's header.
static void advance_stream(Camera_Stream *stream, const World *world, Camera_Frame_Header *frame)
{
    u64 now = get_sim_time_ns(world);
    while (now >= get_frame_time_ns(stream, stream->next_frame)) {
        stream->next_frame++;
    }
    frame->stamp_ns = now;
    frame->width = stream->intrinsics.width;
    frame->height = stream->intrinsics.height;
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

static Camera3D make_stream_camera(const Sensor_Camera *camera, const Camera_Stream *stream,
                                   const World *world)
{
    b3Transform link_pose = get_sensor_pose(&world->robot, &camera->mount);
    return make_optical_camera(b3MulTransforms(link_pose, stream->in_link), &stream->intrinsics);
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

// Colour: the lit scene, as the viewer draws it, shadows included (from the viewer's latest
// shadow map). No stars: a camera exposed for sunlit regolith can't see them.
void render_color_frame(Sensor_Camera *camera, const Viewer *viewer, const World *world,
                        Camera_Frame_Header *frame)
{
    Camera_Stream *stream = &camera->color;
    advance_stream(stream, world, frame);
    Material color_material = viewer->material;
    BeginTextureMode(stream->target);
    ClearBackground(BLACK);
    rlSetClipPlanes(VIEWER_NEAR, VIEWER_FAR);
    BeginMode3D(make_stream_camera(camera, stream, world));
    begin_lit_drawing(&viewer->shading);
    draw_scene(viewer, world, 1.0f, &color_material, false);
    end_lit_drawing();
    EndMode3D();
    EndTextureMode();

    u32 width = frame->width;
    u32 height = frame->height;
    read_target(&stream->target, camera->staging, width, height);
    u8 *rgb = get_color_pixels(frame);
    for (u64 i = 0; i < (u64)width * height; i++) {
        rgb[i * 3 + 0] = camera->staging[i * 4 + 0];
        rgb[i * 3 + 1] = camera->staging[i * 4 + 1];
        rgb[i * 3 + 2] = camera->staging[i * 4 + 2];
    }
}

// Depth: the same scene through the depth shader, with clip planes just outside the depth
// range so the depth buffer spends its precision where it counts.
void render_depth_frame(Sensor_Camera *camera, const Viewer *viewer, const World *world,
                        Camera_Frame_Header *frame)
{
    const Camera_Config *config = camera->config;
    Camera_Stream *stream = &camera->depth;
    advance_stream(stream, world, frame);
    f32 near_plane = 0.5f * config->depth_range[0];
    f32 far_plane = config->depth_range[1] + 1.0f;
    SetShaderValue(camera->depth_shader, camera->near_location, &near_plane, SHADER_UNIFORM_FLOAT);
    SetShaderValue(camera->depth_shader, camera->far_location, &far_plane, SHADER_UNIFORM_FLOAT);
    SetShaderValue(camera->depth_shader, camera->range_location, config->depth_range,
                   SHADER_UNIFORM_VEC2);
    BeginTextureMode(stream->target);
    ClearBackground(BLANK); // no data
    rlSetClipPlanes(near_plane, far_plane);
    BeginMode3D(make_stream_camera(camera, stream, world));
    BeginShaderMode(camera->depth_shader); // for anything drawn without a material
    draw_scene(viewer, world, 1.0f, &camera->depth_material, false);
    EndShaderMode();
    EndMode3D();
    EndTextureMode();
    rlSetClipPlanes(VIEWER_NEAR, VIEWER_FAR);

    u32 width = frame->width;
    u32 height = frame->height;
    read_target(&stream->target, camera->staging, width, height);
    u16 *depth = get_depth_pixels(frame);
    for (u64 i = 0; i < (u64)width * height; i++) {
        depth[i] = unpack_depth_mm(camera->staging[i * 4 + 0], camera->staging[i * 4 + 1]);
    }
}
