#pragma once

#include "core/common.h"

// Triangles from an STL file, three vertices each, with a flat normal per vertex.
// Meshes can be large (tens of MB), so the arrays are malloc'd and freed with free_stl
// once uploaded, rather than living in an allocator.
struct Stl_Mesh {
    f32 *positions; // triangle_count * 9
    f32 *normals; // triangle_count * 9
    u32 triangle_count;
};

// Binary or ASCII STL.
bool load_stl(Stl_Mesh *mesh, const char *path, char *error, u32 error_size);
void free_stl(Stl_Mesh *mesh);

// Turns a URDF resource reference into a file path: file:// is stripped, package://pkg/
// is looked up in each AMENT_PREFIX_PATH entry's share/pkg, and a relative path is
// relative to base_dir (the URDF's directory, when known). False if the file is missing.
bool resolve_resource_path(const char *uri, const char *base_dir, char *path, u32 path_size);
