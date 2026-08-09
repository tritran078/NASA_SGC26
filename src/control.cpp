// control.cpp

#include "control.h"
#include <sl/Camera.hpp>
#include "esp_comm.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <cstdio>
#include <iostream>

// Sends a single "--set-location x0 y0 x1 y1" command over UART to the drive ESP32.
// Matches InputManager::run()'s parsing on the firmware side.
bool sendSetLocation(int esp_fd, float base_x, float base_y, float target_x, float target_y) {
    char buf[128];
    int len = snprintf(
        buf, sizeof(buf),
        "--set-location %.4f %.4f %.4f %.4f ",
        base_x * 100.0f, base_y * 100.0f, target_x * 100.0f, target_y * 100.0f
    );
    if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
        return false;
    }
    ssize_t written = write(esp_fd, buf, len);
    return written == len;
}

// Drives to (target_x, target_y) by repeatedly sending waypoint commands to the
// ESP32 and polling ZED's own tracked pose until within tolerance or timeout.
bool navigateToWaypoint(int esp_motor, sl::Camera &zed, float target_x, float target_y, float tolerance) {
    constexpr int POLL_MS = 100;
    constexpr int RESEND_TIMEOUT_MS = 8000;   // resend command if not converging
    constexpr int OVERALL_TIMEOUT_MS = 30000; // give up on this waypoint entirely

    float pose_x, pose_y, pose_yaw;
    getCurrentPose(zed, pose_x, pose_y, pose_yaw);

    if (!sendSetLocation(esp_motor, pose_x, pose_y, target_x, target_y)) {
        std::cout << "failed to send waypoint to motor ESP\n";
        return false;
    }

    int elapsed_ms = 0;
    int since_last_send_ms = 0;

    while (elapsed_ms < OVERALL_TIMEOUT_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
        elapsed_ms += POLL_MS;
        since_last_send_ms += POLL_MS;

        getCurrentPose(zed, pose_x, pose_y, pose_yaw);

        float dx = target_x - pose_x;
        float dy = target_y - pose_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist <= tolerance) {
            return true;
        }

        // rover hasn't reported progress in a while, resend the command
        if (since_last_send_ms >= RESEND_TIMEOUT_MS) {
            if (!sendSetLocation(esp_motor, pose_x, pose_y, target_x, target_y)) {
                std::cout << "failed to resend waypoint to motor ESP\n";
                return false;
            }
            since_last_send_ms = 0;
        }
    }

    std::cout << "timed out navigating to waypoint (" << target_x << ", " << target_y << ")\n";
    return false;
}

void getCurrentPose(sl::Camera &zed, float &pose_x, float &pose_y, float &pose_yaw) {
    sl::Pose cam_pose;
    zed.getPosition(cam_pose, sl::REFERENCE_FRAME::WORLD);

    sl::Translation translation = cam_pose.getTranslation();
    pose_x = translation.x;
    pose_y = translation.y;

    // ZED coordinate_system was set to RIGHT_HANDED_Z_UP, so yaw is rotation about Z
    sl::float3 euler = cam_pose.getEulerAngles(false); // false = radians
    pose_yaw = euler.z;
}