#include "core/sensors/mount.h"

#include <stdio.h>

#include "core/sensors/camera_model.h"

bool find_sensor_mount(const Robot *robot, const char *frame, const char *key, Sensor_Mount *mount,
                       char *error, u32 error_size)
{
    i32 link = find_urdf_link(&robot->model, frame);
    if (link < 0) {
        snprintf(error, error_size, "%s: the robot has no link named \"%s\"", key, frame);
        return false;
    }
    if (robot->link_body[link] < 0) {
        snprintf(error, error_size, "%s: \"%s\" is a dummy link inside a ball joint", key, frame);
        return false;
    }
    mount->link = (u32)link;
    mount->body = (u32)robot->link_body[link];
    mount->in_body = robot->link_in_body[link];
    return true;
}

Pose get_sensor_pose(const Robot *robot, const Sensor_Mount *mount)
{
    return multiply_poses(robot->bodies[mount->body].current, mount->in_body);
}

static bool is_sensor_unmounted(const Robot *robot, const char *frame, const char *sensor)
{
    if (find_urdf_link(&robot->model, frame) >= 0) {
        return false;
    }
    log_warning("%s: the robot has no link named \"%s\"; turned off", sensor, frame);
    return true;
}

void disable_unmounted_sensors(Sim_Config *config, const Robot *robot)
{
    if (config->lidar.enabled && is_sensor_unmounted(robot, config->lidar.frame, "lidar")) {
        config->lidar.enabled = false;
    }
    if (config->imu.enabled && is_sensor_unmounted(robot, config->imu.frame, "imu")) {
        config->imu.enabled = false;
    }
    Camera_Config *camera = &config->camera;
    char link[CONFIG_STRING_SIZE + 16];
    make_camera_frame_name(camera->name, "_link", link, sizeof(link));
    if (is_camera_enabled(camera) && is_sensor_unmounted(robot, link, "camera")) {
        camera->color.enabled = false;
        camera->depth.enabled = false;
    }
}
