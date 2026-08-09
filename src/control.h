// control.h

#ifndef CONTROL_H
#define CONTROL_H

#include <sl/Camera.hpp>

void getCurrentPose(sl::Camera &zed, float &pose_x, float &pose_y, float &pose_yaw);

bool sendSetLocation(int esp_fd, float base_x, float base_y, float target_x, float target_y);

bool navigateToWaypoint(int esp_motor, sl::Camera &zed, float target_x, float target_y, float tolerance);

#endif