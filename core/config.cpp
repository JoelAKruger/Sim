#include "core/config.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Rows are grouped by section, in the order --dump-config prints them.
// Earth gravity by default: an analogue rover is tested on Earth, and matching the
// hardware is the point. Lunar what-if runs set world.gravity to [0, 0, -1.62].
// clang-format off: one setting per row group reads better than one field per line.
const Config_Field config_fields[] = {
    {.key = "world.gravity", .type = CONFIG_F32, .count = 3,
     .offset = offsetof(Sim_Config, gravity), .minimum = -100.0, .maximum = 100.0,
     .defaults = {0.0, 0.0, -9.81}, .help = "m/s², world frame, z up. Lunar: [0, 0, -1.62]"},
    {.key = "world.physics_hz", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, physics_hz), .minimum = 30.0, .maximum = 10000.0,
     .defaults = {240.0}, .help = "fixed physics steps per simulated second"},
    {.key = "world.substeps", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, substeps), .minimum = 1.0, .maximum = 64.0,
     .defaults = {4.0}, .help = "Box3D solver substeps per step"},
    {.key = "world.workers", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, workers), .minimum = 1.0, .maximum = 64.0,
     .defaults = {1.0}, .help = "Box3D worker threads; 1 is single-threaded"},

    {.key = "terrain.heightmap", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, terrain_heightmap), .text_default = "",
     .help = "PGM heightmap, north up; empty for procedural terrain"},
    {.key = "terrain.height_m", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, terrain_height), .minimum = 0.0, .maximum = 10000.0,
     .defaults = {10.0}, .help = "m spanned by the heightmap's full pixel range"},
    {.key = "terrain.size_m", .type = CONFIG_F32, .count = 2,
     .offset = offsetof(Sim_Config, terrain_size), .minimum = 0.1, .maximum = 100000.0,
     .defaults = {200.0, 200.0}, .help = "m along x and y (procedural terrain)"},
    {.key = "terrain.spacing_m", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, terrain_spacing), .minimum = 0.02, .maximum = 10.0,
     .defaults = {0.5}, .help = "m between height samples (a heightmap's pixel size)"},
    {.key = "terrain.relief_m", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, terrain_relief), .minimum = 0.0, .maximum = 1000.0,
     .defaults = {3.0}, .help = "m, amplitude of the procedural hills"},
    {.key = "terrain.seed", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, terrain_seed), .minimum = 0.0, .maximum = 4294967295.0,
     .defaults = {1.0}, .help = "procedural terrain seed"},
    {.key = "terrain.craters", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, crater_count), .minimum = 0.0, .maximum = 512.0,
     .defaults = {60.0}, .help = "number of craters, 1 to 15 m in radius"},
    {.key = "terrain.friction", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, terrain_friction), .minimum = 0.0, .maximum = 10.0,
     .defaults = {0.8}, .help = "Coulomb friction coefficient of the ground"},

    {.key = "robot.urdf", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, robot_urdf), .text_default = "",
     .help = "URDF file; empty to take /robot_description"},
    {.key = "robot.spawn", .type = CONFIG_F32, .count = 3,
     .offset = offsetof(Sim_Config, robot_spawn), .minimum = -100000.0, .maximum = 100000.0,
     .defaults = {0.0, 0.0, 0.0}, .help = "x, y (m) and yaw (rad), dropped onto the terrain"},
    {.key = "robot.friction", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, robot_friction), .minimum = 0.0, .maximum = 10.0,
     .defaults = {1.0}, .help = "Coulomb friction of every robot shape"},
    {.key = "robot.motor_torque", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, motor_torque), .minimum = 0.001, .maximum = 1000000.0,
     .defaults = {10000.0}, .help = "N·m (or N) every motor can apply; URDF effort limits are ignored"},
    {.key = "robot.servo_speed", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, servo_speed), .minimum = 0.0, .maximum = 1000.0,
     .defaults = {2.0}, .help = "rad/s (or m/s) position-controlled joints (pivots) move at; 0 instant"},
    {.key = "robot.command_timeout_ms", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, command_timeout_ms), .minimum = 0.0, .maximum = 60000.0,
     .defaults = {0.0}, .help = "ms of sim time before an actuator command expires; 0 never"},

    {.key = "can.interface", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, can_interface), .text_default = "can0",
     .help = "SocketCAN interface for the motor controllers and LEDs; empty for no CAN"},

    {.key = "teleop.speed_mps", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, teleop_speed), .minimum = 0.0, .maximum = 100.0,
     .defaults = {0.6}, .help = "m/s at full keyboard drive; no wheel goes faster over the ground"},
    {.key = "teleop.turn_rate", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, teleop_turn_rate), .minimum = 0.0, .maximum = 100.0,
     .defaults = {0.8}, .help = "rad/s turning on the spot; arcs are scaled to teleop.speed_mps"},

    {.key = "lidar.enabled", .type = CONFIG_BOOL, .count = 1,
     .offset = offsetof(Sim_Config, lidar.enabled), .minimum = 0.0, .maximum = 1.0,
     .defaults = {0.0}, .help = "simulate the Livox Mid-360"},
    {.key = "lidar.frame", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, lidar.frame), .text_default = "livox_frame",
     .help = "URDF link the LiDAR is on (x forward, z up); also the frame_id"},
    {.key = "lidar.topic", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, lidar.topic), .text_default = "/livox/lidar",
     .help = "PointCloud2, livox_ros_driver2 xfer_format 0 layout"},
    {.key = "lidar.rate_hz", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, lidar.rate), .minimum = 0.1, .maximum = 100.0,
     .defaults = {10.0}, .help = "frames per second (the driver's publish_freq)"},
    {.key = "lidar.points_per_second", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, lidar.points_per_second), .minimum = 1.0, .maximum = 2000000.0,
     .defaults = {200000.0}, .help = "rays per second, spread evenly through each frame"},
    {.key = "lidar.vertical_fov_deg", .type = CONFIG_F32, .count = 2,
     .offset = offsetof(Sim_Config, lidar.vertical_fov), .minimum = -90.0, .maximum = 90.0,
     .defaults = {-7.0, 52.0}, .help = "lowest and highest elevation; it sees all round"},
    {.key = "lidar.range_m", .type = CONFIG_F32, .count = 2,
     .offset = offsetof(Sim_Config, lidar.range), .minimum = 0.0, .maximum = 1000.0,
     .defaults = {0.1, 40.0}, .help = "blind zone and maximum range; nothing is reported outside"},
    {.key = "lidar.range_noise_m", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, lidar.range_noise), .minimum = 0.0, .maximum = 10.0,
     .defaults = {0.02}, .help = "1 sigma range error"},
    {.key = "lidar.reflectivity", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, lidar.reflectivity), .minimum = 0.0, .maximum = 255.0,
     .defaults = {60.0}, .help = "intensity of a head-on hit; scaled by the incidence cosine"},
    {.key = "lidar.threads", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, lidar.threads), .minimum = 1.0, .maximum = 32.0,
     .defaults = {4.0}, .help = "threads casting rays; the output is the same for any number"},

    {.key = "imu.enabled", .type = CONFIG_BOOL, .count = 1,
     .offset = offsetof(Sim_Config, imu.enabled), .minimum = 0.0, .maximum = 1.0,
     .defaults = {0.0}, .help = "simulate the Mid-360's IMU"},
    {.key = "imu.frame", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, imu.frame), .text_default = "livox_frame",
     .help = "URDF link the IMU is on; also the frame_id"},
    {.key = "imu.topic", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, imu.topic), .text_default = "/livox/imu",
     .help = "sensor_msgs/Imu, without orientation"},
    {.key = "imu.rate_hz", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, imu.rate), .minimum = 1.0, .maximum = 10000.0,
     .defaults = {200.0}, .help = "samples per second; at most world.physics_hz"},
    {.key = "imu.acceleration_in_g", .type = CONFIG_BOOL, .count = 1,
     .offset = offsetof(Sim_Config, imu.acceleration_in_g), .minimum = 0.0, .maximum = 1.0,
     .defaults = {1.0}, .help = "acceleration in g, as livox_ros_driver2 sends it; false for m/s²"},
    {.key = "imu.gyro_noise", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, imu.gyro_noise), .minimum = 0.0, .maximum = 1.0,
     .defaults = {6.6e-5}, .help = "rad/s/√Hz white noise (ICM-40609: 0.0038 °/s/√Hz)"},
    {.key = "imu.accel_noise", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, imu.accel_noise), .minimum = 0.0, .maximum = 10.0,
     .defaults = {9.8e-4}, .help = "m/s²/√Hz white noise (ICM-40609: 100 µg/√Hz)"},
    {.key = "imu.gyro_bias_walk", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, imu.gyro_bias_walk), .minimum = 0.0, .maximum = 1.0,
     .defaults = {1e-5}, .help = "rad/s²/√Hz gyro bias random walk"},
    {.key = "imu.accel_bias_walk", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, imu.accel_bias_walk), .minimum = 0.0, .maximum = 10.0,
     .defaults = {1e-4}, .help = "m/s³/√Hz accelerometer bias random walk"},

    {.key = "camera.enabled", .type = CONFIG_BOOL, .count = 1,
     .offset = offsetof(Sim_Config, camera.enabled), .minimum = 0.0, .maximum = 1.0,
     .defaults = {0.0}, .help = "simulate the RGB-D camera (needs the window)"},
    {.key = "camera.frame", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, camera.frame), .text_default = "camera_color_optical_frame",
     .help = "optical frame of the colour image (z forward, x right, y down)"},
    {.key = "camera.depth_frame", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, camera.depth_frame), .text_default = "",
     .help = "optical frame of the depth image; empty for camera.frame"},
    {.key = "camera.resolution", .type = CONFIG_U32, .count = 2,
     .offset = offsetof(Sim_Config, camera.resolution), .minimum = 16.0, .maximum = 4096.0,
     .defaults = {640.0, 480.0}, .help = "pixels, width and height of both images"},
    {.key = "camera.rate_hz", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, camera.rate), .minimum = 0.1, .maximum = 240.0,
     .defaults = {30.0}, .help = "frames per second of sim time"},
    {.key = "camera.horizontal_fov_deg", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, camera.horizontal_fov), .minimum = 1.0, .maximum = 170.0,
     .defaults = {69.0}, .help = "colour; pixels are square and centred"},
    {.key = "camera.depth_horizontal_fov_deg", .type = CONFIG_F32, .count = 1,
     .offset = offsetof(Sim_Config, camera.depth_horizontal_fov), .minimum = 1.0, .maximum = 170.0,
     .defaults = {87.0}, .help = "depth; pixels are square and centred"},
    {.key = "camera.depth_range_m", .type = CONFIG_F32, .count = 2,
     .offset = offsetof(Sim_Config, camera.depth_range), .minimum = 0.01, .maximum = 65.0,
     .defaults = {0.1, 10.0}, .help = "nearest and farthest depth; outside it a pixel is 0"},
    {.key = "camera.color_topic", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, camera.color_topic),
     .text_default = "/camera/camera/color/image_raw",
     .help = "rgb8 Image; its CameraInfo goes to the sibling camera_info"},
    {.key = "camera.depth_topic", .type = CONFIG_STRING, .count = 1,
     .offset = offsetof(Sim_Config, camera.depth_topic),
     .text_default = "/camera/camera/depth/image_rect_raw",
     .help = "16UC1 Image in mm; its CameraInfo goes to the sibling camera_info"},

    {.key = "sensors.seed", .type = CONFIG_U32, .count = 1,
     .offset = offsetof(Sim_Config, sensor_seed), .minimum = 0.0, .maximum = 4294967295.0,
     .defaults = {1.0}, .help = "noise seed; the same seed gives the same readings"},
};
// clang-format on
const u32 config_field_count = ARRAY_COUNT(config_fields);

static void set_error(char *error, u32 error_size, const char *format, ...)
    __attribute__((format(printf, 3, 4)));

static void set_error(char *error, u32 error_size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(error, error_size, format, args);
    va_end(args);
}

const Config_Field *find_config_field(const char *key)
{
    for (u32 i = 0; i < config_field_count; i++) {
        if (strcmp(config_fields[i].key, key) == 0) {
            return &config_fields[i];
        }
    }
    return NULL;
}

static void write_field(Sim_Config *config, const Config_Field *field, const f64 *values)
{
    u8 *base = (u8 *)config + field->offset;
    for (u32 i = 0; i < field->count; i++) {
        if (field->type == CONFIG_F32) {
            ((f32 *)base)[i] = (f32)values[i];
        } else if (field->type == CONFIG_BOOL) {
            ((bool *)base)[i] = values[i] != 0.0;
        } else {
            ((u32 *)base)[i] = (u32)values[i];
        }
    }
}

// Pads with zeros, so identical settings are identical bytes.
static void write_field_text(Sim_Config *config, const Config_Field *field, const char *text)
{
    strncpy((char *)config + field->offset, text, CONFIG_STRING_SIZE);
}

Sim_Config get_default_config(void)
{
    Sim_Config config = {};
    for (u32 i = 0; i < config_field_count; i++) {
        const Config_Field *field = &config_fields[i];
        if (field->type == CONFIG_STRING) {
            write_field_text(&config, field, field->text_default);
        } else {
            write_field(&config, field, field->defaults);
        }
    }
    return config;
}

bool set_config(Sim_Config *config, const char *key, const f64 *values, u32 count, char *error,
                u32 error_size)
{
    const Config_Field *field = find_config_field(key);
    if (!field) {
        set_error(error, error_size, "%s is not a setting", key);
        return false;
    }
    if (field->type == CONFIG_STRING) {
        set_error(error, error_size, "%s takes text, not numbers", key);
        return false;
    }
    if (count != field->count) {
        set_error(error, error_size, "%s takes %u value%s, got %u", key, field->count,
                  field->count == 1 ? "" : "s", count);
        return false;
    }
    for (u32 i = 0; i < count; i++) {
        if (!(values[i] >= field->minimum && values[i] <= field->maximum)) {
            set_error(error, error_size, "%s must be within [%g, %g], got %g", key, field->minimum,
                      field->maximum, values[i]);
            return false;
        }
        if (field->type != CONFIG_F32 && values[i] != floor(values[i])) {
            set_error(error, error_size, "%s must be a whole number, got %g", key, values[i]);
            return false;
        }
    }
    write_field(config, field, values);
    return true;
}

bool set_config_text(Sim_Config *config, const char *key, const char *text, char *error,
                     u32 error_size)
{
    const Config_Field *field = find_config_field(key);
    if (!field) {
        set_error(error, error_size, "%s is not a setting", key);
        return false;
    }
    if (field->type != CONFIG_STRING) {
        set_error(error, error_size, "%s takes numbers, not text", key);
        return false;
    }
    if (strlen(text) >= CONFIG_STRING_SIZE) {
        set_error(error, error_size, "%s is longer than %u characters", key,
                  CONFIG_STRING_SIZE - 1);
        return false;
    }
    write_field_text(config, field, text);
    return true;
}

u32 get_config(const Sim_Config *config, const Config_Field *field, f64 *values)
{
    if (field->type == CONFIG_STRING) {
        return 0;
    }
    const u8 *base = (const u8 *)config + field->offset;
    for (u32 i = 0; i < field->count; i++) {
        if (field->type == CONFIG_F32) {
            values[i] = (f64)((const f32 *)base)[i];
        } else if (field->type == CONFIG_BOOL) {
            values[i] = ((const bool *)base)[i] ? 1.0 : 0.0;
        } else {
            values[i] = (f64)((const u32 *)base)[i];
        }
    }
    return field->count;
}

const char *get_config_text(const Sim_Config *config, const Config_Field *field)
{
    return field->type == CONFIG_STRING ? (const char *)config + field->offset : "";
}

bool validate_config(const Sim_Config *config, char *error, u32 error_size)
{
    // A heightmap's size comes from the image, checked when it loads.
    for (u32 axis = 0; axis < 2 && !config->terrain_heightmap[0]; axis++) {
        f32 samples = config->terrain_size[axis] / config->terrain_spacing;
        if (samples < 2.0f || samples > 16384.0f) {
            set_error(error, error_size,
                      "terrain.size_m / terrain.spacing_m must give 2..16384 samples per side, "
                      "got %g m / %g m",
                      (f64)config->terrain_size[axis], (f64)config->terrain_spacing);
            return false;
        }
    }
    struct Ordered_Pair {
        const char *key;
        const f32 *values;
    };
    Ordered_Pair pairs[] = {
        {"lidar.vertical_fov_deg", config->lidar.vertical_fov},
        {"lidar.range_m", config->lidar.range},
        {"camera.depth_range_m", config->camera.depth_range},
    };
    for (u32 i = 0; i < ARRAY_COUNT(pairs); i++) {
        if (!(pairs[i].values[0] < pairs[i].values[1])) {
            set_error(error, error_size,
                      "%s: the first value must be below the second, got [%g, %g]", pairs[i].key,
                      (f64)pairs[i].values[0], (f64)pairs[i].values[1]);
            return false;
        }
    }
    if (config->imu.rate > (f32)config->physics_hz) {
        set_error(
            error, error_size,
            "imu.rate_hz (%g) can't be above world.physics_hz (%u): one sample per step at most",
            (f64)config->imu.rate, config->physics_hz);
        return false;
    }
    return true;
}

// The shortest text that reads back to exactly this f32 (so 9.81, not 9.81000042), with
// a decimal point so ROS reads it as a double.
static void format_real(f32 value, char *text, u32 size)
{
    for (i32 precision = 6; precision <= 9; precision++) {
        snprintf(text, size, "%.*g", precision, (f64)value);
        if ((f32)strtod(text, NULL) == value) {
            break;
        }
    }
    if (!strpbrk(text, ".eEn")) {
        strncat(text, ".0", size - strlen(text) - 1);
    }
}

// Output that keeps counting past the end of the buffer, like snprintf.
struct Text_Output {
    char *buffer;
    u32 size;
    u32 length;
};

static void append_text(Text_Output *out, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

static void append_text(Text_Output *out, const char *format, ...)
{
    u32 start = min(out->length, out->size);
    va_list args;
    va_start(args, format);
    out->length += (u32)vsnprintf(out->buffer + start, out->size - start, format, args);
    va_end(args);
}

u32 format_config(const Sim_Config *config, char *buffer, u32 buffer_size)
{
    Text_Output out = {.buffer = buffer, .size = buffer_size, .length = 0};
    char line[2 * CONFIG_STRING_SIZE + 64]; // room for a fully escaped string
    char section[64] = "";
    append_text(&out, "# Effective Regolith settings. Use with --config, or with ROS 2 as\n"
                      "# --ros-args --params-file. Generated by regolith --dump-config.\n"
                      "regolith:\n  ros__parameters:\n");
    for (u32 i = 0; i < config_field_count; i++) {
        const Config_Field *field = &config_fields[i];
        const char *name = strchr(field->key, '.') + 1;
        u32 section_length = (u32)(name - 1 - field->key);
        if (strncmp(section, field->key, section_length) != 0 || section[section_length] != 0) {
            snprintf(section, sizeof(section), "%.*s", (int)section_length, field->key);
            append_text(&out, "    %s:\n", section);
        }

        if (field->type == CONFIG_STRING) {
            // Double-quoted, with " and \ escaped, as the config reader expects.
            char escaped[2 * CONFIG_STRING_SIZE];
            u32 length = 0;
            for (const char *c = get_config_text(config, field); *c; c++) {
                if (*c == '"' || *c == '\\') {
                    escaped[length++] = '\\';
                }
                escaped[length++] = *c;
            }
            escaped[length] = 0;
            snprintf(line, sizeof(line), "%s: \"%s\"", name, escaped);
        } else {
            f64 values[CONFIG_MAX_COUNT];
            u32 count = get_config(config, field, values);
            u32 used = (u32)snprintf(line, sizeof(line), "%s: %s", name, count > 1 ? "[" : "");
            for (u32 v = 0; v < count; v++) {
                char number[32];
                if (field->type == CONFIG_F32) {
                    format_real((f32)values[v], number, sizeof(number));
                } else if (field->type == CONFIG_BOOL) {
                    snprintf(number, sizeof(number), "%s", values[v] != 0.0 ? "true" : "false");
                } else {
                    snprintf(number, sizeof(number), "%u", (u32)values[v]);
                }
                used +=
                    (u32)snprintf(line + used, sizeof(line) - used, "%s%s", v ? ", " : "", number);
            }
            snprintf(line + used, sizeof(line) - used, "%s", count > 1 ? "]" : "");
        }
        append_text(&out, "      %-28s # %s\n", line, field->help);
    }
    return out.length;
}
