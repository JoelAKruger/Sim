#pragma once

#include "core/common.h"

#define CONFIG_STRING_SIZE 256

// Where the robot starts. Stored as three consecutive f32s, like every numeric setting.
struct Spawn_Pose {
    f32 x; // m
    f32 y; // m
    f32 yaw; // rad
};

// The Livox Mid-360 LiDAR, as livox_ros_driver2 publishes it (xfer_format 0).
struct Lidar_Config {
    bool enabled;
    char frame[CONFIG_STRING_SIZE]; // URDF link it is on (x forward, z up); also the frame_id
    char topic[CONFIG_STRING_SIZE];
    f32 rate; // frames per second
    f32 points_per_second;
    f32 vertical_fov[2]; // degrees, lowest and highest; horizontally it sees all round
    f32 range[2]; // m, blind zone and maximum range
    f32 range_noise; // m, 1 sigma
    f32 reflectivity; // 0..255 when hit head-on
    u32 threads; // ray casting threads, counting the sim's own
};

// The IMU inside the Mid-360, as livox_ros_driver2 publishes it.
struct Imu_Config {
    bool enabled;
    char frame[CONFIG_STRING_SIZE]; // URDF link it is on; also the frame_id
    char topic[CONFIG_STRING_SIZE];
    f32 rate; // samples per second
    bool acceleration_in_g; // the Livox driver publishes g, not m/s²
    f32 gyro_noise; // rad/s/√Hz
    f32 accel_noise; // m/s²/√Hz
    f32 gyro_bias_walk; // rad/s²/√Hz
    f32 accel_bias_walk; // m/s³/√Hz
};

// One image stream of the camera (colour or depth), set up independently, as
// realsense2_camera's colour and depth profiles are.
struct Camera_Stream_Config {
    bool enabled;
    u32 resolution[2]; // pixels, width and height
    f32 rate; // frames per second of sim time
    f32 horizontal_fov; // degrees; pixels are square and centred
    char topic[CONFIG_STRING_SIZE]; // the Image; its CameraInfo goes to the sibling camera_info
};

// An RGB-D camera, published as realsense2_camera does. Generic: every parameter is here.
// It sits on the URDF link <name>_link (x forward, z up), as the RealSense description
// names it. The driver's optical frames hang off that link: depth at its origin, colour at
// color_offset, both turned to z forward, x right, y down. It is on when either stream is.
struct Camera_Config {
    char name[CONFIG_STRING_SIZE]; // the driver's camera_name: frames are <name>_link, etc.
    Camera_Stream_Config color;
    f32 color_offset[3]; // m, the colour sensor from <name>_link, in that link's axes
    Camera_Stream_Config depth;
    f32 depth_range[2]; // m; outside it a depth pixel is 0 (no data)
};

inline bool is_camera_enabled(const Camera_Config *camera)
{
    return camera->color.enabled || camera->depth.enabled;
}

// Every tunable the core reads. Plain data, so any front end (a config file, ROS
// parameters) can fill it without knowing anything about Box3D.
struct Sim_Config {
    v3 gravity; // m/s², world frame (z up)
    u32 physics_hz;
    u32 substeps;
    u32 workers; // Box3D worker threads; 1 is single-threaded

    char terrain_heightmap[CONFIG_STRING_SIZE]; // PGM file; empty for procedural terrain
    f32 terrain_height; // m spanned by a heightmap's full pixel range
    f32 terrain_size[2]; // m along x and y (procedural terrain)
    f32 terrain_spacing; // m between height samples (a heightmap's pixel size)
    f32 terrain_relief; // m, amplitude of the procedural hills
    u32 terrain_seed;
    u32 crater_count;
    f32 terrain_friction;

    char robot_urdf[CONFIG_STRING_SIZE]; // URDF file; empty to wait for /robot_description
    Spawn_Pose robot_spawn; // dropped onto the terrain
    f32 robot_friction;
    f32 motor_torque; // N·m (or N) every driven joint's motor can apply
    f32 servo_speed; // rad/s (or m/s) a position-controlled joint moves at; 0 is instant
    f32 command_timeout_ms; // an actuator command expires after this; 0 never

    char can_interface[CONFIG_STRING_SIZE]; // SocketCAN interface; empty for no CAN

    f32 teleop_speed; // m/s at full keyboard drive
    f32 teleop_turn_rate; // rad/s at full keyboard turn

    Lidar_Config lidar;
    Imu_Config imu;
    Camera_Config camera;
    u32 sensor_seed; // noise seed, so sensor output is repeatable
};

// The settings schema: one row per setting, the single source of truth for its key,
// type, range, default and description. The config file reader, the ROS parameter
// bridge and --dump-config are all driven by this table.
#define CONFIG_MAX_COUNT 4

enum Config_Type {
    CONFIG_F32,
    CONFIG_U32,
    CONFIG_STRING, // a char[CONFIG_STRING_SIZE]
    CONFIG_BOOL, // a bool; true or false in files
};

struct Config_Field {
    const char *key; // "section.name"
    Config_Type type;
    u32 count; // values; 1 for a scalar or a string
    u32 offset; // offsetof(Sim_Config, ...)
    f64 minimum;
    f64 maximum;
    f64 defaults[CONFIG_MAX_COUNT];
    const char *text_default; // strings only
    const char *help;
};

extern const Config_Field config_fields[];
extern const u32 config_field_count;

const Config_Field *find_config_field(const char *key);

Sim_Config get_default_config(void);

// Sets a numeric setting from count numbers. Fails, changing nothing, if the key is
// unknown or not numeric, the count is wrong, a value is out of range, or a whole number
// was expected.
bool set_config(Sim_Config *config, const char *key, const f64 *values, u32 count, char *error,
                u32 error_size);

// Sets a string setting. Fails if the key is unknown or not a string, or the text is too
// long.
bool set_config_text(Sim_Config *config, const char *key, const char *text, char *error,
                     u32 error_size);

// Reads a numeric setting's current values. Returns the count (0 for a string setting).
u32 get_config(const Sim_Config *config, const Config_Field *field, f64 *values);

// Reads a string setting's current value ("" for a numeric setting).
const char *get_config_text(const Sim_Config *config, const Config_Field *field);

// Checks that settings are consistent with each other. Ranges are enforced when set.
bool validate_config(const Sim_Config *config, char *error, u32 error_size);

// Applies a settings file: the same YAML as a ROS parameters file, with or without the
// "regolith: ros__parameters:" wrapper. Unknown keys are warned about and counted in
// unknown_keys (which may be NULL); anything else wrong fails with file:line in error.
bool load_config_file(Sim_Config *config, const char *path, u32 *unknown_keys, char *error,
                      u32 error_size);
bool load_config_text(Sim_Config *config, const char *text, const char *name, u32 *unknown_keys,
                      char *error, u32 error_size);

// Writes every setting as YAML (loadable by both --config and --params-file). Returns the
// length it needed, like snprintf; the output is truncated if that exceeds buffer_size.
u32 format_config(const Sim_Config *config, char *buffer, u32 buffer_size);
