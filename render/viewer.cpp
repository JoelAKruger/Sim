#include "render/viewer.h"

#include <math.h>
#include <raymath.h>
#include <rlgl.h>
#include <stdio.h>
#include <string.h>

#include "core/sensors/camera_model.h"
#include "core/stl.h"

// Cells per terrain tile edge. 128 keeps each tile's vertex count below the 65536 that
// Raylib's 16-bit indices can address.
#define TILE_CELLS 128

// The overlay's layout, in pixels. The 3D view gets the rest of the window.
#define TOP_BAR_HEIGHT 36.0f
#define HINT_BAR_HEIGHT 28.0f
#define PANEL_WIDTH 268.0f
#define PANEL_PADDING 18.0f
#define ROW_HEIGHT 30.0f

static const char *lit_vertex_shader = R"glsl(
#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 frag_normal;
out vec4 frag_color;
void main()
{
    frag_normal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
    frag_color = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

// Harsh direct sun plus a faint fill, since there is no atmosphere to scatter light.
static const char *lit_fragment_shader = R"glsl(
#version 330
in vec3 frag_normal;
in vec4 frag_color;
uniform vec4 colDiffuse;
uniform vec3 sun_direction;
out vec4 final_color;
void main()
{
    vec3 normal = normalize(frag_normal);
    float sun = max(dot(normal, -sun_direction), 0.0);
    float fill = 0.10 + 0.06 * normal.z;
    vec3 albedo = frag_color.rgb * colDiffuse.rgb;
    final_color = vec4(albedo * (fill + 1.05 * sun), 1.0);
}
)glsl";

static Vector3 convert_vector3(v3 v) { return Vector3{v.x, v.y, v.z}; }

static Quaternion convert_quaternion(b3Quat q) { return Quaternion{q.v.x, q.v.y, q.v.z, q.s}; }

static Matrix make_transform_matrix(b3Transform transform)
{
    return MatrixMultiply(QuaternionToMatrix(convert_quaternion(transform.q)),
                          MatrixTranslate(transform.p.x, transform.p.y, transform.p.z));
}

static Matrix make_pose_matrix(b3Transform previous, b3Transform current, f32 alpha, Vector3 scale)
{
    Vector3 position = Vector3Lerp(convert_vector3(previous.p), convert_vector3(current.p), alpha);
    Quaternion rotation =
        QuaternionSlerp(convert_quaternion(previous.q), convert_quaternion(current.q), alpha);
    Matrix matrix = MatrixScale(scale.x, scale.y, scale.z);
    matrix = MatrixMultiply(matrix, QuaternionToMatrix(rotation));
    return MatrixMultiply(matrix, MatrixTranslate(position.x, position.y, position.z));
}

static f32 get_sample_height(const Terrain *terrain, i32 row, i32 col)
{
    row = min(max(row, 0), (i32)terrain->rows - 1);
    col = min(max(col, 0), (i32)terrain->cols - 1);
    return terrain->heights[row * (i32)terrain->cols + col];
}

// One tile of the terrain as an indexed mesh, triangulated exactly as Box3D does.
static Mesh build_terrain_tile(const Terrain *terrain, u32 first_row, u32 first_col, u32 cell_rows,
                               u32 cell_cols)
{
    u32 stride = cell_cols + 1;
    Mesh mesh = {};
    mesh.vertexCount = (int)((cell_rows + 1) * stride);
    mesh.triangleCount = (int)(cell_rows * cell_cols * 2);
    mesh.vertices = (float *)MemAlloc((u32)mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = (float *)MemAlloc((u32)mesh.vertexCount * 3 * sizeof(float));
    mesh.colors = (unsigned char *)MemAlloc((u32)mesh.vertexCount * 4);
    mesh.indices = (unsigned short *)MemAlloc((u32)mesh.triangleCount * 3 * sizeof(u16));

    for (u32 r = 0; r <= cell_rows; r++) {
        for (u32 c = 0; c <= cell_cols; c++) {
            i32 row = (i32)(first_row + r);
            i32 col = (i32)(first_col + c);
            u32 v = r * stride + c;
            mesh.vertices[v * 3 + 0] = terrain->origin_x + (f32)col * terrain->spacing;
            mesh.vertices[v * 3 + 1] = terrain->origin_y - (f32)row * terrain->spacing;
            mesh.vertices[v * 3 + 2] = get_sample_height(terrain, row, col);

            // Central differences; y decreases as the row index increases.
            f32 dz_dx = (get_sample_height(terrain, row, col + 1) -
                         get_sample_height(terrain, row, col - 1)) /
                        (2.0f * terrain->spacing);
            f32 dz_dy = (get_sample_height(terrain, row - 1, col) -
                         get_sample_height(terrain, row + 1, col)) /
                        (2.0f * terrain->spacing);
            Vector3 normal = Vector3Normalize(Vector3{-dz_dx, -dz_dy, 1.0f});
            mesh.normals[v * 3 + 0] = normal.x;
            mesh.normals[v * 3 + 1] = normal.y;
            mesh.normals[v * 3 + 2] = normal.z;

            // Regolith grey with a little per-sample variation so flat ground reads as
            // a surface rather than a flat colour.
            u32 hash = (u32)row * 0x9e3779b1u ^ (u32)col * 0x85ebca77u;
            hash ^= hash >> 15;
            hash *= 0x2c1b3c6du;
            hash ^= hash >> 12;
            u8 grey = (u8)(142 + (hash & 15));
            mesh.colors[v * 4 + 0] = grey;
            mesh.colors[v * 4 + 1] = (u8)(grey - 3);
            mesh.colors[v * 4 + 2] = (u8)(grey - 8);
            mesh.colors[v * 4 + 3] = 255;
        }
    }

    // Box3D's two triangles per cell: (11, 21, 12) and (22, 12, 21), where 11 is the
    // cell's north-west sample. Both wind counter-clockwise seen from above.
    u32 index = 0;
    for (u32 r = 0; r < cell_rows; r++) {
        for (u32 c = 0; c < cell_cols; c++) {
            u16 v11 = (u16)(r * stride + c);
            u16 v12 = (u16)(v11 + 1);
            u16 v21 = (u16)(v11 + stride);
            u16 v22 = (u16)(v21 + 1);
            u16 triangles[6] = {v11, v21, v12, v22, v12, v21};
            for (u32 i = 0; i < 6; i++) {
                mesh.indices[index++] = triangles[i];
            }
        }
    }

    UploadMesh(&mesh, false);
    return mesh;
}

static Camera3D make_camera_from_orbit(const Orbit_Camera *orbit)
{
    Vector3 offset = {
        cosf(orbit->pitch) * cosf(orbit->yaw),
        cosf(orbit->pitch) * sinf(orbit->yaw),
        sinf(orbit->pitch),
    };
    Camera3D camera = {};
    camera.target = orbit->target;
    camera.position = Vector3Add(orbit->target, Vector3Scale(offset, orbit->distance));
    camera.up = Vector3{0.0f, 0.0f, 1.0f};
    camera.fovy = 55.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

// Returns true if the user panned, which ends following the robot.
// The mouse belongs to the 3D view unless it is over the UI. A drag belongs to wherever it
// started, so orbiting past the panel keeps orbiting and a press on the panel never does.
static bool handle_orbit_input(Orbit_Camera *orbit, const Terrain *terrain, f32 frame_seconds,
                               bool over_ui, bool *drag_in_view)
{
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) || IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) {
        *drag_in_view = !over_ui;
    }
    Vector2 mouse = GetMouseDelta();
    if (*drag_in_view && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        orbit->yaw -= mouse.x * 0.005f;
        orbit->pitch = min(max(orbit->pitch + mouse.y * 0.005f, -0.05f), 1.55f);
    }
    if (!over_ui) {
        orbit->distance =
            min(max(orbit->distance * (1.0f - 0.1f * GetMouseWheelMove()), 1.0f), 500.0f);
    }

    // Pan in the ground plane, relative to the view direction.
    Vector3 forward = {-cosf(orbit->yaw), -sinf(orbit->yaw), 0.0f};
    Vector3 right = {forward.y, -forward.x, 0.0f}; // forward x up
    f32 pan_x = 0.0f;
    f32 pan_y = 0.0f;
    if (*drag_in_view && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        pan_x -= mouse.x * 0.0015f * orbit->distance;
        pan_y += mouse.y * 0.0015f * orbit->distance;
    }
    f32 key_speed = 0.8f * orbit->distance * frame_seconds;
    if (IsKeyDown(KEY_W))
        pan_y += key_speed;
    if (IsKeyDown(KEY_S))
        pan_y -= key_speed;
    if (IsKeyDown(KEY_D))
        pan_x += key_speed;
    if (IsKeyDown(KEY_A))
        pan_x -= key_speed;
    orbit->target = Vector3Add(orbit->target, Vector3Scale(right, pan_x));
    orbit->target = Vector3Add(orbit->target, Vector3Scale(forward, pan_y));
    orbit->target.z = get_terrain_height(terrain, orbit->target.x, orbit->target.y);
    return pan_x != 0.0f || pan_y != 0.0f;
}

static u32 add_mesh(Viewer *viewer, Mesh mesh, const char *path, v3 scale)
{
    Viewer_Mesh *entry = &viewer->meshes[viewer->mesh_count];
    entry->mesh = mesh;
    snprintf(entry->path, sizeof(entry->path), "%s", path);
    entry->scale = scale;
    return viewer->mesh_count++;
}

// Uploads an STL to the GPU, then drops the CPU copy: meshes can be tens of MB.
static Mesh make_mesh_from_stl(const Stl_Mesh *stl, v3 scale)
{
    Mesh mesh = {};
    u32 count = stl->triangle_count * 3;
    mesh.vertexCount = (int)count;
    mesh.triangleCount = (int)stl->triangle_count;
    mesh.vertices = (float *)MemAlloc(count * 3 * sizeof(float));
    mesh.normals = (float *)MemAlloc(count * 3 * sizeof(float));
    for (u32 i = 0; i < count; i++) {
        mesh.vertices[i * 3 + 0] = stl->positions[i * 3 + 0] * scale.x;
        mesh.vertices[i * 3 + 1] = stl->positions[i * 3 + 1] * scale.y;
        mesh.vertices[i * 3 + 2] = stl->positions[i * 3 + 2] * scale.z;
        // Normals scale by the inverse, so non-uniform scaling keeps them perpendicular.
        Vector3 normal = {stl->normals[i * 3 + 0] / scale.x, stl->normals[i * 3 + 1] / scale.y,
                          stl->normals[i * 3 + 2] / scale.z};
        normal = Vector3Normalize(normal);
        mesh.normals[i * 3 + 0] = normal.x;
        mesh.normals[i * 3 + 1] = normal.y;
        mesh.normals[i * 3 + 2] = normal.z;
    }
    UploadMesh(&mesh, false);
    MemFree(mesh.vertices);
    MemFree(mesh.normals);
    mesh.vertices = NULL;
    mesh.normals = NULL;
    return mesh;
}

static i32 load_mesh_file(Viewer *viewer, const Urdf_Geometry *geometry, const char *resource_dir,
                          u32 *triangles)
{
    char path[URDF_PATH_SIZE];
    if (!resolve_resource_path(geometry->mesh, resource_dir, path, sizeof(path))) {
        log_warning("viewer: mesh %s not found; not drawn", geometry->mesh);
        return -1;
    }
    for (u32 i = 0; i < viewer->mesh_count; i++) {
        const Viewer_Mesh *entry = &viewer->meshes[i];
        if (strcmp(entry->path, path) == 0 && entry->scale.x == geometry->scale.x &&
            entry->scale.y == geometry->scale.y && entry->scale.z == geometry->scale.z) {
            return (i32)i;
        }
    }
    Stl_Mesh stl;
    char error[256];
    if (!load_stl(&stl, path, error, sizeof(error))) {
        log_warning("viewer: %s; not drawn", error);
        return -1;
    }
    *triangles += stl.triangle_count;
    Mesh mesh = make_mesh_from_stl(&stl, geometry->scale);
    free_stl(&stl);
    return (i32)add_mesh(viewer, mesh, path, geometry->scale);
}

static Color make_color(const f32 *rgba)
{
    return Color{(u8)(min(max(rgba[0], 0.0f), 1.0f) * 255.0f),
                 (u8)(min(max(rgba[1], 0.0f), 1.0f) * 255.0f),
                 (u8)(min(max(rgba[2], 0.0f), 1.0f) * 255.0f), 255};
}

// Adds a shape to draw: solid in color, or as a collision wireframe. False if its mesh
// could not be loaded.
static bool add_visual(Viewer *viewer, const Urdf_Shape *shape, bool collision, const f32 *color,
                       const char *resource_dir, u32 *triangles)
{
    const Urdf_Geometry *geometry = &shape->geometry;
    Matrix correction = MatrixIdentity();
    v3 unit = {1.0f, 1.0f, 1.0f};
    i32 mesh = -1;
    switch (geometry->type) {
    case URDF_GEOMETRY_BOX:
        mesh = (i32)add_mesh(
            viewer, GenMeshCube(geometry->size.x, geometry->size.y, geometry->size.z), "", unit);
        break;
    case URDF_GEOMETRY_CYLINDER:
        // Raylib's cylinder stands on y = 0 and rises along +y; URDF's is centred on z.
        mesh = (i32)add_mesh(viewer, GenMeshCylinder(geometry->radius, geometry->length, 32), "",
                             unit);
        correction = MatrixMultiply(MatrixRotateX(0.5f * PI_F32),
                                    MatrixTranslate(0.0f, 0.0f, -0.5f * geometry->length));
        break;
    case URDF_GEOMETRY_SPHERE:
        mesh = (i32)add_mesh(viewer, GenMeshSphere(geometry->radius, 12, 18), "", unit);
        break;
    case URDF_GEOMETRY_MESH:
        mesh = load_mesh_file(viewer, geometry, resource_dir, triangles);
        break;
    default:
        break;
    }
    if (mesh < 0) {
        return false;
    }
    viewer->visuals[viewer->visual_count++] = Viewer_Visual{
        .link = shape->link,
        .mesh = (u32)mesh,
        .origin = shape->origin,
        .correction = correction,
        .color = make_color(color),
        .collision = collision,
    };
    return true;
}

static void unload_robot_meshes(Viewer *viewer)
{
    for (u32 i = 0; i < viewer->mesh_count; i++) {
        UnloadMesh(viewer->meshes[i].mesh);
    }
    viewer->mesh_count = 0;
    viewer->visual_count = 0;
}

static void load_robot_meshes(Viewer *viewer, const World *world, Linear_Allocator *allocator)
{
    u64 start = get_time_ns();
    const Robot *robot = &world->robot;
    const Urdf_Model *model = &robot->model;
    // Collision shapes can appear twice: as wireframes, and standing in for a visual.
    u32 capacity = max(model->visual_count + 2 * model->collision_count, 1u);
    viewer->meshes = ALLOCATE_ARRAY(allocator, Viewer_Mesh, capacity);
    viewer->visuals = ALLOCATE_ARRAY(allocator, Viewer_Visual, capacity);
    // Per link: the colour of a visual that couldn't be loaded (DAE, say), or NULL.
    const f32 **missing = ALLOCATE_ARRAY(allocator, const f32 *, max(model->link_count, 1u));
    if (!viewer->meshes || !viewer->visuals || !missing) {
        return;
    }
    const char *dir = robot->settings.resource_dir;
    u32 triangles = 0;
    for (u32 i = 0; i < model->visual_count; i++) {
        const Urdf_Shape *shape = &model->visuals[i];
        if (!add_visual(viewer, shape, false, shape->color, dir, &triangles)) {
            missing[shape->link] = shape->color;
        }
    }
    f32 highlight[4] = {1.0f, 0.85f, 0.1f, 1.0f};
    for (u32 i = 0; i < model->collision_count; i++) {
        const Urdf_Shape *shape = &model->collisions[i];
        add_visual(viewer, shape, true, highlight, dir, &triangles);
        if (missing[shape->link]) {
            add_visual(viewer, shape, false, missing[shape->link], dir, &triangles);
        }
    }
    for (u32 link = 0; link < model->link_count; link++) {
        if (missing[link]) {
            log_warning(
                "viewer: %s's visual could not be drawn; showing its collision shapes instead",
                model->links[link].name);
        }
    }
    log_info("viewer: %s drawn with %u meshes (%u k triangles from files) in %.0f ms", model->name,
             viewer->mesh_count, triangles / 1000, (f64)(get_time_ns() - start) * 1e-6);
}

static void draw_robot(const Viewer *viewer, const World *world, f32 alpha, Material *material,
                       bool show_collisions)
{
    const Robot *robot = &world->robot;
    for (u32 pass = 0; pass < 2; pass++) {
        bool collision = pass == 1;
        if (collision && !show_collisions) {
            break;
        }
        if (collision) {
            rlDrawRenderBatchActive();
            rlEnableWireMode();
        }
        for (u32 i = 0; i < viewer->visual_count; i++) {
            const Viewer_Visual *visual = &viewer->visuals[i];
            if (visual->collision != collision) {
                continue;
            }
            b3Transform pose =
                b3MulTransforms(get_link_pose(robot, visual->link, alpha), visual->origin);
            material->maps[MATERIAL_MAP_DIFFUSE].color = visual->color;
            DrawMesh(viewer->meshes[visual->mesh].mesh, *material,
                     MatrixMultiply(visual->correction, make_transform_matrix(pose)));
        }
        if (collision) {
            rlDrawRenderBatchActive();
            rlDisableWireMode();
        }
    }
}

bool create_viewer(Viewer *viewer, const World *world, Linear_Allocator *allocator)
{
    *viewer = {};
    viewer->follow = true;
    SetTraceLogLevel(LOG_WARNING);
    // No vsync: on Wayland a vsync'd swap waits for the compositor, which stops asking for
    // frames while the window is minimised or hidden, and that would freeze the simulation
    // too. Frames are paced by a timer instead (below).
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1600, 900, "Regolith");
    if (!IsWindowReady()) {
        log_error("viewer: could not open a window (no display or GL driver); "
                  "--headless runs without one");
        return false;
    }
    SetWindowMinSize(1024, 720); // the panel's contents need this much height
    rlSetClipPlanes(0.05, 2000.0);
    i32 refresh_hz = GetMonitorRefreshRate(GetCurrentMonitor());
    SetTargetFPS(refresh_hz > 0 ? refresh_hz : 60);

    create_ui(&viewer->ui);
    viewer->show_camera_preview = true;

    viewer->lit = LoadShaderFromMemory(lit_vertex_shader, lit_fragment_shader);
    viewer->lit.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(viewer->lit, "matModel");
    viewer->lit.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(viewer->lit, "matNormal");
    viewer->sun_direction_location = GetShaderLocation(viewer->lit, "sun_direction");
    Vector3 sun_direction = Vector3Normalize(Vector3{-0.55f, 0.35f, -0.45f});
    SetShaderValue(viewer->lit, viewer->sun_direction_location, &sun_direction,
                   SHADER_UNIFORM_VEC3);
    viewer->material = LoadMaterialDefault();
    viewer->material.shader = viewer->lit;

    const Terrain *terrain = &world->terrain;
    u32 tile_rows = (terrain->rows - 1 + TILE_CELLS - 1) / TILE_CELLS;
    u32 tile_cols = (terrain->cols - 1 + TILE_CELLS - 1) / TILE_CELLS;
    viewer->terrain_tiles = ALLOCATE_ARRAY(allocator, Mesh, tile_rows * tile_cols);
    if (!viewer->terrain_tiles) {
        return false;
    }
    for (u32 tile_row = 0; tile_row < tile_rows; tile_row++) {
        for (u32 tile_col = 0; tile_col < tile_cols; tile_col++) {
            u32 first_row = tile_row * TILE_CELLS;
            u32 first_col = tile_col * TILE_CELLS;
            u32 cell_rows = min((u32)TILE_CELLS, terrain->rows - 1 - first_row);
            u32 cell_cols = min((u32)TILE_CELLS, terrain->cols - 1 - first_col);
            viewer->terrain_tiles[viewer->terrain_tile_count++] =
                build_terrain_tile(terrain, first_row, first_col, cell_rows, cell_cols);
        }
    }
    viewer->box = GenMeshCube(1.0f, 1.0f, 1.0f);

    viewer->camera.target = Vector3{0.0f, 0.0f, get_terrain_height(terrain, 0.0f, 0.0f)};
    viewer->camera.yaw = -2.4f;
    viewer->camera.pitch = 0.45f;
    viewer->camera.distance = 18.0f;
    if (world->has_robot) {
        load_robot_meshes(viewer, world, allocator);
    }
    return true;
}

void destroy_viewer(Viewer *viewer)
{
    unload_robot_meshes(viewer);
    for (u32 i = 0; i < viewer->terrain_tile_count; i++) {
        UnloadMesh(viewer->terrain_tiles[i]);
    }
    UnloadMesh(viewer->box);
    UnloadMaterial(viewer->material); // also unloads the lit shader
    destroy_ui(&viewer->ui);
    CloseWindow();
    *viewer = {};
}

void draw_scene(const Viewer *viewer, const World *world, f32 alpha, Material *material,
                bool show_collisions)
{
    material->maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    for (u32 i = 0; i < viewer->terrain_tile_count; i++) {
        DrawMesh(viewer->terrain_tiles[i], *material, MatrixIdentity());
    }
    for (u32 i = 0; i < world->prop_count; i++) {
        const Prop *prop = &world->props[i];
        material->maps[MATERIAL_MAP_DIFFUSE].color =
            Color{(u8)(prop->color >> 16), (u8)(prop->color >> 8), (u8)prop->color, 255};
        Vector3 size = convert_vector3(2.0f * prop->half_extents);
        DrawMesh(viewer->box, *material,
                 make_pose_matrix(prop->previous, prop->current, alpha, size));
    }
    if (world->has_robot) {
        draw_robot(viewer, world, alpha, material, show_collisions);
    }
}

// The last complete LiDAR frame, placed with the sensor's current pose and coloured by
// height above it: a wrong mount shows at a glance.
static void draw_lidar_points(const World *world)
{
    const Lidar_Frame *frame = world->has_lidar ? get_last_lidar_frame(&world->lidar) : NULL;
    if (!frame) {
        return;
    }
    b3Transform pose = get_sensor_pose(&world->robot, &world->lidar.mount);
    const f32 size = 0.02f;
    for (u32 i = 0; i < frame->count; i++) {
        const Lidar_Point *point = &frame->points[i];
        v3 position = b3TransformPoint(pose, v3{point->x, point->y, point->z});
        f32 height = min(max((position.z - pose.p.z + 2.0f) / 4.0f, 0.0f), 1.0f);
        Color color = ColorFromHSV(240.0f * (1.0f - height), 0.9f, 1.0f);
        Vector3 p = convert_vector3(position);
        DrawLine3D(Vector3{p.x - size, p.y, p.z}, Vector3{p.x + size, p.y, p.z}, color);
        DrawLine3D(Vector3{p.x, p.y - size, p.z}, Vector3{p.x, p.y + size, p.z}, color);
        DrawLine3D(Vector3{p.x, p.y, p.z - size}, Vector3{p.x, p.y, p.z + size}, color);
    }
}

// The part of the window the 3D view gets: right of the panel, between the bars.
static Rectangle get_view_area(void)
{
    return Rectangle{PANEL_WIDTH, TOP_BAR_HEIGHT, (f32)GetScreenWidth() - PANEL_WIDTH,
                     (f32)GetScreenHeight() - TOP_BAR_HEIGHT - HINT_BAR_HEIGHT};
}

// BeginMode3D, but into part of the window, so the view is centred where it can be seen
// rather than behind the panel.
static void begin_view_3d(Camera3D camera, Rectangle view)
{
    rlDrawRenderBatchActive();
    f32 scale_x = (f32)GetRenderWidth() / (f32)GetScreenWidth();
    f32 scale_y = (f32)GetRenderHeight() / (f32)GetScreenHeight();
    // GL counts rows from the bottom.
    rlViewport((i32)(view.x * scale_x),
               (i32)(((f32)GetScreenHeight() - view.y - view.height) * scale_y),
               (i32)(view.width * scale_x), (i32)(view.height * scale_y));
    rlMatrixMode(RL_PROJECTION);
    rlPushMatrix();
    rlLoadIdentity();
    f64 near = rlGetCullDistanceNear();
    f64 far = rlGetCullDistanceFar();
    f64 top = near * tan(0.5 * (f64)camera.fovy * DEG2RAD);
    f64 right = top * (f64)view.width / (f64)view.height;
    rlFrustum(-right, right, -top, top, near, far);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
    Matrix look = MatrixLookAt(camera.position, camera.target, camera.up);
    rlMultMatrixf(MatrixToFloat(look));
    rlEnableDepthTest();
}

static void end_view_3d(void)
{
    EndMode3D();
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
}

// text, shortened with ".." to fit max_width.
static const char *fit_ui_text(const Ui *ui, Ui_Font_Kind kind, const char *text, f32 max_width,
                               char *buffer, u32 buffer_size)
{
    if (measure_ui_text(ui, kind, text) <= max_width) {
        return text;
    }
    snprintf(buffer, buffer_size, "%s", text);
    for (u32 length = (u32)strlen(buffer); length > 2; length--) {
        buffer[length - 2] = '.';
        buffer[length - 1] = '.';
        buffer[length] = 0;
        if (measure_ui_text(ui, kind, buffer) <= max_width) {
            break;
        }
    }
    return buffer;
}

// The top bar: what is running, and how fast.
static void draw_top_bar(Viewer *viewer, const World *world, const Frame_Stats *stats)
{
    Ui *ui = &viewer->ui;
    const Ui_Theme *theme = &ui->theme;
    f32 width = (f32)GetScreenWidth();
    draw_ui_panel(ui, Rectangle{0.0f, 0.0f, width, TOP_BAR_HEIGHT}, theme->bar);
    DrawRectangle(0, (i32)TOP_BAR_HEIGHT - 1, (i32)width, 1, theme->border);

    f32 text_y = 0.5f * (TOP_BAR_HEIGHT - get_ui_font_size(ui, UI_FONT_BODY));
    f32 x = PANEL_PADDING;
    draw_ui_text(ui, UI_FONT_BOLD, "REGOLITH", x, text_y, theme->text);
    x += measure_ui_text(ui, UI_FONT_BOLD, "REGOLITH") + 14.0f;
    if (world->has_robot) {
        draw_ui_text(ui, UI_FONT_BODY, world->robot.model.name, x, text_y, theme->text_dim);
        x += measure_ui_text(ui, UI_FONT_BODY, world->robot.model.name) + 14.0f;
    }
    if (stats->paused) {
        f32 tag_width = measure_ui_text(ui, UI_FONT_SMALL, "PAUSED") + 16.0f;
        Rectangle tag = {x, 0.5f * (TOP_BAR_HEIGHT - 20.0f), tag_width, 20.0f};
        DrawRectangleRounded(tag, 0.4f, 6, Fade(theme->amber, 0.2f));
        draw_ui_text(ui, UI_FONT_SMALL, "PAUSED", tag.x + 8.0f,
                     tag.y + 0.5f * (tag.height - get_ui_font_size(ui, UI_FONT_SMALL)),
                     theme->amber);
    }

    // Fixed-width values, so the numbers don't jitter as they change.
    f32 speed = 0.0f;
    if (world->has_robot) {
        // Speed from the last step's motion, so the viewer needn't ask Box3D.
        const Robot_Body *root =
            &world->robot.bodies[world->robot.link_body[world->robot.model.root]];
        speed = b3Length(b3Sub(root->current.p, root->previous.p)) / world->step_seconds;
    }
    char values[5][32];
    snprintf(values[0], sizeof(values[0]), "%7.2f s", (f64)get_sim_time_ns(world) * 1e-9);
    snprintf(values[1], sizeof(values[1]), "%4.2f×", stats->realtime_factor);
    snprintf(values[2], sizeof(values[2]), "%5.3f ms", stats->step_ms);
    snprintf(values[3], sizeof(values[3]), "%3d", GetFPS());
    snprintf(values[4], sizeof(values[4]), "%4.2f m/s", (f64)speed);
    const char *labels[5] = {"SIM", "RTF", "STEP", "FPS", "SPEED"};
    f32 right = width - PANEL_PADDING;
    f32 label_y = 0.5f * (TOP_BAR_HEIGHT - get_ui_font_size(ui, UI_FONT_SMALL));
    for (i32 i = 4; i >= 0; i--) {
        f32 value_width = measure_ui_text(ui, UI_FONT_BODY, values[i]);
        draw_ui_text(ui, UI_FONT_BODY, values[i], right - value_width, text_y, theme->text);
        right -= value_width + 8.0f;
        f32 label_width = measure_ui_text(ui, UI_FONT_SMALL, labels[i]);
        draw_ui_text(ui, UI_FONT_SMALL, labels[i], right - label_width, label_y, theme->text_dim);
        right -= label_width + 26.0f;
    }
}

// A sensor's row: a dot, its name, the frame it is on, and its rate.
static void draw_sensor_row(Ui *ui, f32 x, f32 y, f32 width, const char *name, bool on,
                            const char *frame, f32 rate)
{
    const Ui_Theme *theme = &ui->theme;
    f32 text_y = y + 0.5f * (ROW_HEIGHT - get_ui_font_size(ui, UI_FONT_BODY));
    f32 small_y = y + 0.5f * (ROW_HEIGHT - get_ui_font_size(ui, UI_FONT_SMALL));
    draw_ui_dot(x + 4.0f, y + 0.5f * ROW_HEIGHT, on ? theme->green : theme->text_disabled);
    draw_ui_text(ui, UI_FONT_BODY, name, x + 18.0f, text_y, on ? theme->text : theme->text_dim);
    char rate_text[32];
    snprintf(rate_text, sizeof(rate_text), on ? "%.0f Hz" : "off", (f64)rate);
    f32 rate_width = measure_ui_text(ui, UI_FONT_SMALL, rate_text);
    draw_ui_text(ui, UI_FONT_SMALL, rate_text, x + width - rate_width, small_y, theme->text_dim);
    if (on) {
        f32 frame_x = x + 90.0f;
        char fitted[128];
        const char *shown =
            fit_ui_text(ui, UI_FONT_SMALL, frame, x + width - rate_width - 12.0f - frame_x, fitted,
                        sizeof(fitted));
        draw_ui_text(ui, UI_FONT_SMALL, shown, frame_x, small_y, theme->text_dim);
    }
}

// The control panel down the left: who drives, the simulation, the view and the sensors.
static void draw_side_panel(Viewer *viewer, const World *world, const Frame_Stats *stats,
                            Viewer_Actions *actions)
{
    Ui *ui = &viewer->ui;
    const Ui_Theme *theme = &ui->theme;
    f32 height = (f32)GetScreenHeight() - TOP_BAR_HEIGHT - HINT_BAR_HEIGHT;
    draw_ui_panel(ui, Rectangle{0.0f, TOP_BAR_HEIGHT, PANEL_WIDTH, height}, theme->panel);
    DrawRectangle((i32)PANEL_WIDTH - 1, (i32)TOP_BAR_HEIGHT, 1, (i32)height, theme->border);

    f32 x = PANEL_PADDING;
    f32 width = PANEL_WIDTH - 2.0f * PANEL_PADDING;
    f32 y = TOP_BAR_HEIGHT + PANEL_PADDING;
    f32 body_offset = 0.5f * (ROW_HEIGHT - get_ui_font_size(ui, UI_FONT_BODY));

    // Control: exactly one source drives, and it must never be in doubt which.
    draw_ui_section(ui, "CONTROL", x, &y, width);
    if (world->has_robot) {
        const char *modes[2] = {"CAN", "KEYBOARD"};
        Color accents[2] = {theme->green, theme->amber};
        u32 selected = world->control_mode == CONTROL_CAN ? 0 : 1;
        i32 clicked =
            draw_ui_segmented(ui, Rectangle{x, y, width, 32.0f}, modes, accents, 2, selected);
        if (clicked >= 0 && (u32)clicked != selected) {
            actions->toggle_control = true;
        }
        y += 32.0f + 6.0f;

        char status[CONFIG_STRING_SIZE + 32];
        Color dot;
        if (world->control_mode == CONTROL_CAN) {
            bool running = stats->can_running;
            snprintf(status, sizeof(status), running ? "Listening on %s" : "No CAN interface",
                     world->config.can_interface);
            dot = running ? theme->green : theme->red;
        } else {
            snprintf(status, sizeof(status), "%s",
                     stats->can_ignored ? "Ignoring CAN commands" : "Arrow keys drive");
            dot = stats->can_ignored ? theme->amber : theme->text_dim;
        }
        draw_ui_dot(x + 4.0f, y + 0.5f * ROW_HEIGHT, dot);
        draw_ui_text(ui, UI_FONT_BODY, status, x + 18.0f, y + body_offset, theme->text);
        y += ROW_HEIGHT;

        const Status_Light *light = &world->status_light;
        bool lit = light->red + light->green + light->blue > 0.01f;
        draw_ui_text(ui, UI_FONT_BODY, "Status light", x, y + body_offset, theme->text);
        Rectangle swatch = {x + width - 40.0f, y + 8.0f, 40.0f, ROW_HEIGHT - 16.0f};
        Color color = {(u8)(light->red * 255.0f), (u8)(light->green * 255.0f),
                       (u8)(light->blue * 255.0f), 255};
        DrawRectangleRounded(swatch, 0.4f, 6, lit ? color : theme->control);
        DrawRectangleRoundedLinesEx(swatch, 0.4f, 6, 1.0f, theme->border);
        if (!lit) {
            f32 off_width = measure_ui_text(ui, UI_FONT_SMALL, "off");
            draw_ui_text(ui, UI_FONT_SMALL, "off", swatch.x - off_width - 8.0f,
                         y + 0.5f * (ROW_HEIGHT - get_ui_font_size(ui, UI_FONT_SMALL)),
                         theme->text_dim);
        }
        y += ROW_HEIGHT;
    }
    y += 14.0f;

    draw_ui_section(ui, "SIMULATION", x, &y, width);
    f32 half = 0.5f * (width - 8.0f);
    f32 button_height = 32.0f;
    if (draw_ui_button(ui, Rectangle{x, y, half, button_height}, stats->paused ? "Resume" : "Pause",
                       "P", true)) {
        actions->toggle_pause = true;
    }
    if (draw_ui_button(ui, Rectangle{x + half + 8.0f, y, half, button_height}, "Step", "N",
                       stats->paused)) {
        actions->single_step = true;
    }
    y += button_height + 8.0f;
    if (draw_ui_button(ui, Rectangle{x, y, width, button_height}, "Reset robot", "R",
                       world->has_robot)) {
        actions->reset_robot = true;
    }
    y += button_height + 22.0f;

    draw_ui_section(ui, "VIEW", x, &y, width);
    draw_ui_toggle(ui, Rectangle{x, y, width, ROW_HEIGHT}, "Follow robot", "F", &viewer->follow,
                   world->has_robot);
    y += ROW_HEIGHT;
    draw_ui_toggle(ui, Rectangle{x, y, width, ROW_HEIGHT}, "Collision shapes", "C",
                   &viewer->show_collisions, world->has_robot);
    y += ROW_HEIGHT;
    draw_ui_toggle(ui, Rectangle{x, y, width, ROW_HEIGHT}, "LiDAR points", "L", &viewer->show_lidar,
                   world->has_lidar);
    y += ROW_HEIGHT;
    draw_ui_toggle(ui, Rectangle{x, y, width, ROW_HEIGHT}, "Camera preview", NULL,
                   &viewer->show_camera_preview, viewer->camera_preview != NULL);
    y += ROW_HEIGHT + 14.0f;

    draw_ui_section(ui, "SENSORS", x, &y, width);
    const Sim_Config *config = &world->config;
    draw_sensor_row(ui, x, y, width, "LiDAR", world->has_lidar, config->lidar.frame,
                    config->lidar.rate);
    y += ROW_HEIGHT;
    draw_sensor_row(ui, x, y, width, "IMU", world->has_imu, config->imu.frame, config->imu.rate);
    y += ROW_HEIGHT;
    char camera_link[CONFIG_STRING_SIZE + 16];
    make_camera_frame_name(config->camera.name, "_link", camera_link, sizeof(camera_link));
    draw_sensor_row(ui, x, y, width, "Camera", viewer->camera_preview != NULL, camera_link,
                    config->camera.rate);
}

// Every shortcut, along the bottom: the key bright, what it does dim.
static void draw_hint_bar(Viewer *viewer)
{
    Ui *ui = &viewer->ui;
    const Ui_Theme *theme = &ui->theme;
    f32 width = (f32)GetScreenWidth();
    f32 top = (f32)GetScreenHeight() - HINT_BAR_HEIGHT;
    draw_ui_panel(ui, Rectangle{0.0f, top, width, HINT_BAR_HEIGHT}, theme->bar);
    DrawRectangle(0, (i32)top, (i32)width, 1, theme->border);
    const char *hints[][2] = {
        {"RMB", "orbit"}, {"MMB/WASD", "pan"}, {"Wheel", "zoom"}, {"Arrows", "drive"},
        {"M", "mode"},    {"R", "reset"},      {"P", "pause"},    {"N", "step"},
        {"F", "follow"},  {"C", "collisions"}, {"L", "lidar"},    {"Space", "box"},
    };
    f32 x = PANEL_PADDING;
    f32 y = top + 0.5f * (HINT_BAR_HEIGHT - get_ui_font_size(ui, UI_FONT_SMALL));
    for (u32 i = 0; i < ARRAY_COUNT(hints); i++) {
        f32 key_width = measure_ui_text(ui, UI_FONT_SMALL, hints[i][0]);
        f32 action_width = measure_ui_text(ui, UI_FONT_SMALL, hints[i][1]);
        if (x + key_width + 6.0f + action_width > width - PANEL_PADDING) {
            break; // a narrow window shows as many as fit
        }
        draw_ui_text(ui, UI_FONT_SMALL, hints[i][0], x, y, theme->text);
        x += key_width + 6.0f;
        draw_ui_text(ui, UI_FONT_SMALL, hints[i][1], x, y, theme->text_dim);
        x += action_width + 20.0f;
    }
}

// The sensor camera's latest colour image, in a card at the bottom right.
static void draw_camera_card(Viewer *viewer, const World *world)
{
    if (!viewer->camera_preview || !viewer->show_camera_preview) {
        return;
    }
    Ui *ui = &viewer->ui;
    const Ui_Theme *theme = &ui->theme;
    const Texture2D *texture = viewer->camera_preview;
    f32 screen_width = (f32)GetScreenWidth();
    f32 screen_height = (f32)GetScreenHeight();
    f32 image_width = min(320.0f, 0.25f * screen_width);
    f32 image_height = image_width * (f32)texture->height / (f32)texture->width;
    f32 title_height = 26.0f;
    Rectangle card = {screen_width - image_width - 16.0f - 12.0f,
                      screen_height - HINT_BAR_HEIGHT - 12.0f - image_height - title_height - 8.0f,
                      image_width + 16.0f, image_height + title_height + 8.0f};
    draw_ui_panel(ui, card, theme->panel);
    DrawRectangleLinesEx(card, 1.0f, theme->border);
    f32 title_y = card.y + 0.5f * (title_height - get_ui_font_size(ui, UI_FONT_SMALL)) + 2.0f;
    draw_ui_text(ui, UI_FONT_SMALL, "CAMERA", card.x + 8.0f, title_y, theme->text);
    char color_frame[CONFIG_STRING_SIZE + 32];
    make_camera_frame_name(world->config.camera.name, "_color_optical_frame", color_frame,
                           sizeof(color_frame));
    char fitted[128];
    f32 name_x = card.x + 8.0f + measure_ui_text(ui, UI_FONT_SMALL, "CAMERA") + 10.0f;
    draw_ui_text(ui, UI_FONT_SMALL,
                 fit_ui_text(ui, UI_FONT_SMALL, color_frame, card.x + card.width - 8.0f - name_x,
                             fitted, sizeof(fitted)),
                 name_x, title_y, theme->text_dim);
    // Render targets are stored bottom-up.
    Rectangle source = {0.0f, 0.0f, (f32)texture->width, -(f32)texture->height};
    Rectangle target = {card.x + 8.0f, card.y + title_height, image_width, image_height};
    DrawTexturePro(*texture, source, target, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
}

Viewer_Actions draw_frame(Viewer *viewer, const World *world, f32 alpha, const Frame_Stats *stats)
{
    Viewer_Actions actions = {};
    actions.quit = WindowShouldClose();

    begin_ui(&viewer->ui);
    f32 frame_seconds = GetFrameTime();
    if (handle_orbit_input(&viewer->camera, &world->terrain, frame_seconds,
                           is_mouse_over_ui(&viewer->ui), &viewer->drag_in_view)) {
        viewer->follow = false;
    }
    if (IsKeyPressed(KEY_F)) {
        viewer->follow = !viewer->follow;
    }
    if (IsKeyPressed(KEY_C)) {
        viewer->show_collisions = !viewer->show_collisions;
    }
    if (IsKeyPressed(KEY_L)) {
        viewer->show_lidar = !viewer->show_lidar;
    }
    if (viewer->follow && world->has_robot) {
        // Locked to the robot: a lagging camera makes stops and turns look springy.
        b3Transform root = get_link_pose(&world->robot, world->robot.model.root, alpha);
        viewer->camera.target = convert_vector3(root.p);
    }
    actions.drive = (f32)IsKeyDown(KEY_UP) - (f32)IsKeyDown(KEY_DOWN);
    actions.turn = (f32)IsKeyDown(KEY_LEFT) - (f32)IsKeyDown(KEY_RIGHT);
    if (IsKeyPressed(KEY_SPACE)) {
        actions.drop_box = true;
        actions.drop_position =
            v3{viewer->camera.target.x, viewer->camera.target.y, viewer->camera.target.z + 4.0f};
    }
    actions.toggle_pause = IsKeyPressed(KEY_P);
    actions.toggle_control = IsKeyPressed(KEY_M);
    actions.reset_robot = IsKeyPressed(KEY_R);
    actions.single_step = IsKeyPressed(KEY_N);

    BeginDrawing();
    ClearBackground(BLACK);
    Rectangle view = get_view_area();
    begin_view_3d(make_camera_from_orbit(&viewer->camera), view);

    draw_scene(viewer, world, alpha, &viewer->material, viewer->show_collisions);
    if (viewer->show_lidar) {
        draw_lidar_points(world);
    }

    end_view_3d();
    // The UI, after the 3D scene. Its clicks become actions like the shortcuts'.
    draw_top_bar(viewer, world, stats);
    draw_side_panel(viewer, world, stats, &actions);
    draw_hint_bar(viewer);
    draw_camera_card(viewer, world);
    if (viewer->screenshot_path) {
        // Flush the batched 2D overlay, then read the back buffer before EndDrawing
        // swaps it away.
        rlDrawRenderBatchActive();
        Image image = LoadImageFromScreen();
        if (!ExportImage(image, viewer->screenshot_path)) {
            log_error("viewer: could not write %s", viewer->screenshot_path);
        }
        UnloadImage(image);
        viewer->screenshot_path = NULL;
    }
    EndDrawing();
    return actions;
}
