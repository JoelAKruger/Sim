#include "core/stl.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/math.h"

static void set_flat_normals(Stl_Mesh *mesh)
{
    for (u32 t = 0; t < mesh->triangle_count; t++) {
        const f32 *p = mesh->positions + t * 9;
        v3 a = {p[0], p[1], p[2]};
        v3 b = {p[3], p[4], p[5]};
        v3 c = {p[6], p[7], p[8]};
        v3 normal = cross(b - a, c - a);
        f32 length = get_length(normal);
        normal = length > 0.0f ? (1.0f / length) * normal : v3{0.0f, 0.0f, 0.0f};
        for (u32 corner = 0; corner < 3; corner++) {
            f32 *out = mesh->normals + t * 9 + corner * 3;
            out[0] = normal.x;
            out[1] = normal.y;
            out[2] = normal.z;
        }
    }
}

static bool allocate_mesh(Stl_Mesh *mesh, u32 triangle_count)
{
    mesh->triangle_count = triangle_count;
    mesh->positions = (f32 *)malloc((u64)triangle_count * 9 * sizeof(f32));
    mesh->normals = (f32 *)malloc((u64)triangle_count * 9 * sizeof(f32));
    return mesh->positions && mesh->normals;
}

// Binary layout: 80-byte header, u32 count, then 50 bytes per triangle (normal, three
// vertices, u16 attribute), little-endian.
static bool load_binary(Stl_Mesh *mesh, const u8 *data, u64 size)
{
    u32 count;
    memcpy(&count, data + 80, 4);
    if (!allocate_mesh(mesh, count)) {
        return false;
    }
    const u8 *cursor = data + 84;
    for (u32 t = 0; t < count; t++, cursor += 50) {
        memcpy(mesh->positions + t * 9, cursor + 12, 36);
    }
    (void)size;
    return true;
}

static bool load_ascii(Stl_Mesh *mesh, const char *text)
{
    u32 vertices = 0;
    for (const char *c = strstr(text, "vertex"); c; c = strstr(c + 6, "vertex")) {
        vertices++;
    }
    if (vertices == 0 || vertices % 3 != 0 || !allocate_mesh(mesh, vertices / 3)) {
        return false;
    }
    u32 index = 0;
    for (const char *c = strstr(text, "vertex"); c; c = strstr(c, "vertex")) {
        c += 6;
        char *end;
        for (u32 axis = 0; axis < 3; axis++) {
            mesh->positions[index * 3 + axis] = strtof(c, &end);
            c = end;
        }
        index++;
    }
    return true;
}

bool load_stl(Stl_Mesh *mesh, const char *path, char *error, u32 error_size)
{
    *mesh = {};
    FILE *file = fopen(path, "rb");
    if (!file) {
        snprintf(error, error_size, "cannot open %s", path);
        return false;
    }
    fseek(file, 0, SEEK_END);
    u64 size = (u64)ftell(file);
    fseek(file, 0, SEEK_SET);
    u8 *data = (u8 *)malloc(size + 1);
    bool read = data && fread(data, 1, size, file) == size;
    fclose(file);
    if (!read) {
        free(data);
        snprintf(error, error_size, "cannot read %s", path);
        return false;
    }
    data[size] = 0;

    // Some binary files start with "solid" too, so the size decides.
    u32 count = 0;
    if (size >= 84) {
        memcpy(&count, data + 80, 4);
    }
    bool loaded;
    if (size >= 84 && size == 84 + (u64)count * 50) {
        loaded = load_binary(mesh, data, size);
    } else {
        loaded = load_ascii(mesh, (const char *)data);
    }
    free(data);
    if (!loaded) {
        free_stl(mesh);
        snprintf(error, error_size, "%s is not an STL file (the only mesh format supported)", path);
        return false;
    }
    set_flat_normals(mesh);
    return true;
}

void free_stl(Stl_Mesh *mesh)
{
    free(mesh->positions);
    free(mesh->normals);
    *mesh = {};
}

static bool is_readable(const char *path) { return access(path, R_OK) == 0; }

bool resolve_resource_path(const char *uri, const char *base_dir, char *path, u32 path_size)
{
    if (strncmp(uri, "file://", 7) == 0) {
        snprintf(path, path_size, "%s", uri + 7);
        return is_readable(path);
    }
    if (strncmp(uri, "package://", 10) == 0) {
        const char *package = uri + 10;
        const char *slash = strchr(package, '/');
        const char *prefixes = getenv("AMENT_PREFIX_PATH");
        if (!slash || !prefixes) {
            return false;
        }
        const char *prefix = prefixes;
        while (*prefix) {
            const char *end = strchr(prefix, ':');
            u32 length = end ? (u32)(end - prefix) : (u32)strlen(prefix);
            snprintf(path, path_size, "%.*s/share/%.*s%s", (int)length, prefix,
                     (int)(slash - package), package, slash);
            if (is_readable(path)) {
                return true;
            }
            prefix += length + (end ? 1 : 0);
        }
        return false;
    }
    if (uri[0] == '/' || !base_dir || !base_dir[0]) {
        snprintf(path, path_size, "%s", uri);
    } else {
        snprintf(path, path_size, "%s/%s", base_dir, uri);
    }
    return is_readable(path);
}
