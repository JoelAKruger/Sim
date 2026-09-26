#pragma once

#include "core/config.h"
#include "core/robot.h"

// Where a sensor sits: on a URDF link, which rides on one of the robot's bodies.
struct Sensor_Mount {
    u32 link;
    u32 body;
    b3Transform in_body; // the link's frame in the body's frame
};

// Finds the link named frame. key names the setting in error messages ("lidar.frame").
bool find_sensor_mount(const Robot *robot, const char *frame, const char *key, Sensor_Mount *mount,
                       char *error, u32 error_size);

// The sensor frame in the world after the last step.
b3Transform get_sensor_pose(const Robot *robot, const Sensor_Mount *mount);

// Turns off, with a warning, each enabled sensor whose frame the robot has no link for, so
// settings written for one robot still run another.
void disable_unmounted_sensors(Sim_Config *config, const Robot *robot);
