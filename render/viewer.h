#pragma once

#include <raylib.h>

#include "core/world.h"
#include "render/shading.h"
#include "render/ui.h"

// Orbits a target point on the ground. World z is up.
struct Orbit_Camera {
    Vector3 target;
    f32 yaw; // radians about world z, measured from +x
    f32 pitch; // radians above the horizontal
    f32 distance; // m
};

// What the user asked for this frame. The viewer never changes the world itself; the
// main loop applies these between physics steps.
struct Viewer_Actions {
    bool quit;
    bool drop_box;
    v3 drop_position;
    bool toggle_pause;
    bool toggle_control; // switch between CAN and keyboard control
    bool reset_robot; // put the robot back where it spawned
    bool single_step;
    f32 drive; // -1..1, from the arrow keys
    f32 turn; // -1..1, positive to the left
};

// Measured by the main loop and shown in the overlay.
struct Frame_Stats {
    f64 realtime_factor; // achieved sim seconds per wall second
    f64 step_ms; // mean wall time of one physics step
    bool paused;
    bool can_running; // the CAN thread is listening
    bool can_ignored; // CAN commands arrived in keyboard mode within the last second
};

// A GPU mesh: a loaded file (shared by every visual that uses it) or a primitive.
struct Viewer_Mesh {
    Mesh mesh;
    char path[URDF_PATH_SIZE]; // empty for a primitive
    v3 scale;
};

// One URDF visual or collision shape to draw.
struct Viewer_Visual {
    u32 link;
    u32 mesh; // index into Viewer::meshes
    b3Transform origin; // in the link frame
    Matrix correction; // puts a generated primitive into URDF's axis convention
    Color color;
    bool collision; // drawn as a wireframe when collision view is on
};

struct Viewer {
    Shading shading; // the lit shader, shadows and stars
    Material material; // draws with shading.lit
    Mesh *terrain_tiles;
    Vector4 *terrain_tile_bounds; // per tile: a bounding sphere (x, y, z, radius)
    u32 terrain_tile_count;
    Mesh box;
    Mesh boulder_mesh; // every boulder, in world space
    bool has_boulder_mesh;
    Orbit_Camera camera;
    const char *screenshot_path; // when set, the next frame is saved here as a PNG

    // The robot's meshes, loaded with the window.
    Viewer_Mesh *meshes;
    u32 mesh_count;
    Viewer_Visual *visuals;
    u32 visual_count;
    bool show_collisions;
    bool show_lidar; // the last LiDAR frame, as points
    const Texture2D *camera_preview; // the sensor camera's colour image, or NULL
    bool show_camera_preview;
    Ui ui; // the panels and their widgets
    bool drag_in_view; // the current mouse drag started over the 3D view
    bool follow;
};

// Opens the window and uploads the terrain and the robot (if the world has one yet).
// Bookkeeping comes from the allocator. False, with a message, if no window can be opened
// (no display, or no usable GL driver).
bool create_viewer(Viewer *viewer, const World *world, Linear_Allocator *allocator);
void destroy_viewer(Viewer *viewer);

// The world as the viewer draws it, without overlays: terrain, props, the robot and its
// light. The sensor camera draws the same scene. Call inside BeginMode3D.
void draw_scene(const Viewer *viewer, const World *world, f32 alpha, Material *material,
                bool show_collisions);

// Draws one frame, blending each body between its previous and current pose by alpha,
// the fraction of a physics step that has elapsed since the last one.
Viewer_Actions draw_frame(Viewer *viewer, const World *world, f32 alpha, const Frame_Stats *stats);
