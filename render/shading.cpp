#include "render/shading.h"

#include <math.h>
#include <raymath.h>
#include <rlgl.h>

#define SHADOW_EXTENT 60.0f // m covered by the shadow map, centred on the focus
#define SHADOW_DISTANCE 150.0f // m from the focus back towards the sun, for the shadow camera
#define SHADOW_MAP_SLOT 10 // texture unit the shadow map is bound to; the material maps use 0..
#define EDGE_FADE 30.0f // m over which the terrain fades to black at its edges

static const char *lit_vertex_shader = R"glsl(
#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 frag_position;
out vec3 frag_normal;
out vec4 frag_color;
void main()
{
    frag_position = vec3(matModel * vec4(vertexPosition, 1.0));
    frag_normal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
    frag_color = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

// A hard sun with filtered shadows and a faint fill, since there is no atmosphere to
// scatter light. Regolith and rock get procedural grain (in world space, so it doesn't swim)
// that tints the colour and roughens the normal; the ground darkens on slopes. Everything
// fades to black towards the terrain's edges.
static const char *lit_fragment_shader = R"glsl(
#version 330
in vec3 frag_position;
in vec3 frag_normal;
in vec4 frag_color;
uniform vec4 colDiffuse;
uniform vec3 sun_direction;
uniform mat4 light_matrix;
uniform sampler2D shadow_map;
uniform int shadow_filter;
uniform float shadow_texel;
uniform int surface;
uniform float detail;
uniform vec4 terrain_bounds;
uniform float edge_fade;
out vec4 final_color;

float hash(vec2 p)
{
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

float noise(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), u.x),
               mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), u.x), u.y);
}

float get_sunlight(vec3 position, vec3 normal)
{
    if (shadow_filter == 0) {
        return 1.0;
    }
    // Looked up a little way out along the normal, which keeps flat ground free of acne.
    vec4 clip = light_matrix * vec4(position + normal * 1.5 * shadow_texel, 1.0);
    vec3 coords = clip.xyz / clip.w * 0.5 + 0.5;
    if (coords.x <= 0.0 || coords.x >= 1.0 || coords.y <= 0.0 || coords.y >= 1.0 ||
        coords.z >= 1.0) {
        return 1.0;
    }
    vec2 texel = 1.0 / vec2(textureSize(shadow_map, 0));
    float lit = 0.0;
    float count = 0.0;
    for (int x = -shadow_filter; x <= shadow_filter; x++) {
        for (int y = -shadow_filter; y <= shadow_filter; y++) {
            float depth = texture(shadow_map, coords.xy + vec2(x, y) * texel).r;
            lit += coords.z - 0.0003 > depth ? 0.0 : 1.0;
            count += 1.0;
        }
    }
    // Shadows thin out towards the edge of the mapped square instead of stopping dead.
    vec2 edge = min(coords.xy, 1.0 - coords.xy);
    return mix(1.0, lit / count, smoothstep(0.0, 0.05, min(edge.x, edge.y)));
}

void main()
{
    vec3 geometric = normalize(frag_normal);
    vec3 normal = geometric;
    vec3 albedo = frag_color.rgb * colDiffuse.rgb;
    if (detail > 0.0 && surface != 0) {
        vec2 p = frag_position.xy;
        float scale = surface == 1 ? 7.0 : 16.0;
        // Grain finer than a pixel only aliases into ripples, so it fades out with distance
        // (how many grain cells one pixel spans).
        float cells = length(fwidth(p * scale));
        float fine = 1.0 - smoothstep(0.25, 0.8, cells);
        float grain = 0.6 * noise(p * scale) + 0.4 * noise(p * scale * 2.7);
        float h = 0.5 / scale;
        float gx = noise((p + vec2(h, 0.0)) * scale) - noise((p - vec2(h, 0.0)) * scale);
        float gy = noise((p + vec2(0.0, h)) * scale) - noise((p - vec2(0.0, h)) * scale);
        normal = normalize(normal - fine * vec3(gx, gy, 0.0) * (surface == 1 ? 0.35 : 0.6));
        albedo *= mix(0.975, 0.8 + 0.35 * grain, fine);
        if (surface == 1) {
            float patches = 0.6 * noise(p * 0.35) + 0.4 * noise(p * 0.07);
            albedo *= 0.85 + 0.3 * patches;
            albedo *= mix(0.7, 1.0, smoothstep(0.75, 0.97, geometric.z));
        }
    }
    float sun = max(dot(normal, -sun_direction), 0.0);
    float sunlight = sun > 0.0 ? get_sunlight(frag_position, geometric) : 1.0;
    float fill = 0.06 + 0.05 * max(normal.z, 0.0);
    vec3 color = albedo * (fill + 1.2 * sun * sunlight);
    vec2 inside = min(frag_position.xy - terrain_bounds.xy, terrain_bounds.zw - frag_position.xy);
    color *= smoothstep(0.0, edge_fade, min(inside.x, inside.y));
    final_color = vec4(color, 1.0);
}
)glsl";

static const char *caster_vertex_shader = R"glsl(
#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
void main()
{
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

static const char *caster_fragment_shader = R"glsl(
#version 330
out vec4 final_color;
void main()
{
    final_color = vec4(1.0);
}
)glsl";

static void unload_shadow_map(Shading *shading)
{
    if (shading->shadow_framebuffer) {
        rlUnloadTexture(shading->shadow_depth);
        rlUnloadFramebuffer(shading->shadow_framebuffer);
    }
    shading->shadow_framebuffer = 0;
    shading->shadow_depth = 0;
    shading->shadow_size = 0;
}

static void load_shadow_map(Shading *shading, i32 size)
{
    unload_shadow_map(shading);
    u32 framebuffer = rlLoadFramebuffer();
    if (!framebuffer) {
        log_warning("viewer: no framebuffer for shadows; they are off");
        return;
    }
    rlEnableFramebuffer(framebuffer);
    u32 depth = rlLoadTextureDepth(size, size, false);
    rlFramebufferAttach(framebuffer, depth, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    bool complete = rlFramebufferComplete(framebuffer);
    rlDisableFramebuffer();
    if (!complete) {
        log_warning("viewer: the shadow framebuffer is incomplete; shadows are off");
        rlUnloadTexture(depth);
        rlUnloadFramebuffer(framebuffer);
        return;
    }
    shading->shadow_framebuffer = framebuffer;
    shading->shadow_depth = depth;
    shading->shadow_size = size;
}

void set_shading_quality(Shading *shading, u32 quality)
{
    shading->quality = min(quality, 2u);
    i32 filter = (i32)shading->quality; // 0 off, 1 = 3×3, 2 = 5×5
    i32 size = shading->quality == 0 ? 0 : shading->quality == 1 ? 2048 : 4096;
    if (size != shading->shadow_size) {
        if (size == 0) {
            unload_shadow_map(shading);
        } else {
            load_shadow_map(shading, size);
        }
    }
    if (!shading->shadow_framebuffer) {
        filter = 0;
    }
    f32 texel = shading->shadow_size ? shading->shadow_extent / (f32)shading->shadow_size : 0.0f;
    f32 detail = shading->quality > 0 ? 1.0f : 0.0f;
    SetShaderValue(shading->lit, shading->shadow_filter_location, &filter, SHADER_UNIFORM_INT);
    SetShaderValue(shading->lit, shading->shadow_texel_location, &texel, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shading->lit, shading->detail_location, &detail, SHADER_UNIFORM_FLOAT);
}

void create_shading(Shading *shading, const Terrain *terrain, u32 quality)
{
    *shading = {};
    shading->lit = LoadShaderFromMemory(lit_vertex_shader, lit_fragment_shader);
    Shader *lit = &shading->lit;
    lit->locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(*lit, "matModel");
    lit->locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(*lit, "matNormal");
    shading->sun_direction_location = GetShaderLocation(*lit, "sun_direction");
    shading->light_matrix_location = GetShaderLocation(*lit, "light_matrix");
    shading->shadow_map_location = GetShaderLocation(*lit, "shadow_map");
    shading->shadow_filter_location = GetShaderLocation(*lit, "shadow_filter");
    shading->shadow_texel_location = GetShaderLocation(*lit, "shadow_texel");
    shading->surface_location = GetShaderLocation(*lit, "surface");
    shading->detail_location = GetShaderLocation(*lit, "detail");
    shading->terrain_bounds_location = GetShaderLocation(*lit, "terrain_bounds");
    shading->edge_fade_location = GetShaderLocation(*lit, "edge_fade");

    // A low sun, 25 degrees up, as at a lunar polar site: long, hard shadows.
    shading->sun_direction = Vector3Normalize(Vector3{-0.75f, 0.42f, -0.40f});
    SetShaderValue(*lit, shading->sun_direction_location, &shading->sun_direction,
                   SHADER_UNIFORM_VEC3);
    f32 width = (f32)(terrain->cols - 1) * terrain->spacing;
    f32 depth = (f32)(terrain->rows - 1) * terrain->spacing;
    f32 bounds[4] = {terrain->origin_x, terrain->origin_y - depth, terrain->origin_x + width,
                     terrain->origin_y};
    f32 edge_fade = min(EDGE_FADE, 0.15f * min(width, depth));
    SetShaderValue(*lit, shading->terrain_bounds_location, bounds, SHADER_UNIFORM_VEC4);
    SetShaderValue(*lit, shading->edge_fade_location, &edge_fade, SHADER_UNIFORM_FLOAT);
    i32 slot = SHADOW_MAP_SLOT;
    SetShaderValue(*lit, shading->shadow_map_location, &slot, SHADER_UNIFORM_INT);

    shading->caster = LoadShaderFromMemory(caster_vertex_shader, caster_fragment_shader);
    shading->caster_material = LoadMaterialDefault();
    shading->caster_material.shader = shading->caster;
    shading->shadow_extent = SHADOW_EXTENT;
    shading->light_matrix = MatrixIdentity();
    set_shading_quality(shading, quality);

    // A fixed sky: mostly faint stars, a few bright ones.
    Random random = create_random(20260927, 5);
    for (u32 i = 0; i < SHADING_STARS; i++) {
        f32 z = 2.0f * get_random_f32(&random) - 1.0f;
        f32 angle = 2.0f * PI_F32 * get_random_f32(&random);
        f32 ring = sqrtf(max(1.0f - z * z, 0.0f));
        shading->stars[i] = Vector3{ring * cosf(angle), ring * sinf(angle), z};
        f32 u = get_random_f32(&random);
        shading->star_brightness[i] = 0.25f + 0.75f * u * u * u * u;
    }
}

void destroy_shading(Shading *shading)
{
    unload_shadow_map(shading);
    UnloadMaterial(shading->caster_material); // also unloads the caster shader
    UnloadShader(shading->lit);
    *shading = {};
}

void set_shading_surface(const Shading *shading, Shading_Surface surface)
{
    i32 value = (i32)surface;
    SetShaderValue(shading->lit, shading->surface_location, &value, SHADER_UNIFORM_INT);
}

void render_shadow_map(Shading *shading, Vector3 focus, Shadow_Caster_Function *draw_casters,
                       void *context)
{
    if (!shading->shadow_framebuffer) {
        return;
    }
    // Keep the map's texels fixed to the world as the focus moves, so shadow edges don't
    // shimmer: snap the focus to whole texels across the light's view.
    Vector3 forward = shading->sun_direction;
    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, Vector3{0.0f, 0.0f, 1.0f}));
    Vector3 up = Vector3CrossProduct(right, forward);
    f32 texel = shading->shadow_extent / (f32)shading->shadow_size;
    f32 across = Vector3DotProduct(focus, right);
    f32 along = Vector3DotProduct(focus, up);
    focus = Vector3Add(focus, Vector3Scale(right, floorf(across / texel) * texel - across));
    focus = Vector3Add(focus, Vector3Scale(up, floorf(along / texel) * texel - along));

    Camera3D light = {};
    light.position = Vector3Subtract(focus, Vector3Scale(forward, SHADOW_DISTANCE));
    light.target = focus;
    light.up = Vector3{0.0f, 0.0f, 1.0f};
    light.fovy = shading->shadow_extent;
    light.projection = CAMERA_ORTHOGRAPHIC;

    RenderTexture2D target = {};
    target.id = shading->shadow_framebuffer;
    target.texture.width = shading->shadow_size;
    target.texture.height = shading->shadow_size;
    target.depth.id = shading->shadow_depth;
    f64 near = rlGetCullDistanceNear();
    f64 far = rlGetCullDistanceFar();
    rlSetClipPlanes(1.0, 2.0 * SHADOW_DISTANCE);
    BeginTextureMode(target);
    ClearBackground(WHITE);
    BeginMode3D(light);
    Matrix view = rlGetMatrixModelview();
    Matrix projection = rlGetMatrixProjection();
    draw_casters(context, &shading->caster_material);
    EndMode3D();
    EndTextureMode();
    rlSetClipPlanes(near, far);
    shading->light_matrix = MatrixMultiply(view, projection);
    SetShaderValueMatrix(shading->lit, shading->light_matrix_location, shading->light_matrix);
}

void begin_lit_drawing(const Shading *shading)
{
    if (shading->shadow_framebuffer) {
        rlActiveTextureSlot(SHADOW_MAP_SLOT);
        rlEnableTexture(shading->shadow_depth);
        rlActiveTextureSlot(0);
    }
}

void end_lit_drawing(void)
{
    rlActiveTextureSlot(SHADOW_MAP_SLOT);
    rlDisableTexture();
    rlActiveTextureSlot(0);
}

void draw_stars(const Shading *shading, Camera3D camera, Rectangle view)
{
    Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    for (u32 i = 0; i < SHADING_STARS; i++) {
        Vector3 direction = shading->stars[i];
        if (Vector3DotProduct(direction, forward) <= 0.1f) {
            continue;
        }
        Vector3 far_away = Vector3Add(camera.position, Vector3Scale(direction, 1000.0f));
        Vector2 screen = GetWorldToScreenEx(far_away, camera, (i32)view.width, (i32)view.height);
        if (screen.x < 0.0f || screen.y < 0.0f || screen.x >= view.width ||
            screen.y >= view.height) {
            continue;
        }
        f32 brightness = shading->star_brightness[i];
        u8 level = (u8)(255.0f * brightness);
        i32 size = brightness > 0.8f ? 2 : 1;
        DrawRectangle((i32)(view.x + screen.x), (i32)(view.y + screen.y), size, size,
                      Color{level, level, (u8)min(255.0f, level * 1.05f), 255});
    }
}
