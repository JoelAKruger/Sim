#include "core/sensors/mount.h"

#include <stdio.h>

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

b3Transform get_sensor_pose(const Robot *robot, const Sensor_Mount *mount)
{
    return b3MulTransforms(robot->bodies[mount->body].current, mount->in_body);
}
