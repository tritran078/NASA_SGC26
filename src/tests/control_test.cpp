// test_control_bench.cpp
// Full bench test: real ZED pose, real controlLoop, real sendInts to esp_motor.
// Drives toward ONE hardcoded target coordinate and stops within TOLERANCE.
//
// Compile (adjust ZED SDK paths as needed):
//   g++ -std=c++17 test_control_bench.cpp control.cpp esp_comm.cpp \
//       -I/usr/local/zed/include -L/usr/local/zed/lib -lsl_zed \
//       -o test_control_bench

#include "control.h"
#include "esp_comm.h"
#include <sl/Camera.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std;

void printWheels(const string& label, vector<int> w) {
    cout << label << " -> FL:" << w[0] << " FR:" << w[1]
         << " RL:" << w[2] << " RR:" << w[3]
         << " CL:" << w[4] << " CR:" << w[5] << endl;
}

int main() {
    // --- init ZED ---
    sl::Camera zed;
    sl::InitParameters init_param;
    init_param.camera_resolution = sl::RESOLUTION::VGA;
    init_param.camera_fps = 30;
    init_param.coordinate_units = sl::UNIT::METER;
    init_param.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    if (zed.open(init_param) != sl::ERROR_CODE::SUCCESS) {
        cout << "failed to open ZED camera" << endl;
        return 1;
    }

    sl::PositionalTrackingParameters tracking_param;
    zed.enablePositionalTracking(tracking_param);

    // --- open serial to motor ESP ---
    int esp_motor = openSerial("/dev/ttyUSB2", B115200);
    if (esp_motor < 0) {
        cout << "failed to open serial port to motor ESP" << endl;
        zed.close();
        return 1;
    }

    const float TOLERANCE = 0.20f; // 20cm
    const float RATE = 50;         // ms, 20Hz

    // --- get starting pose ---
    float x0, y0, yaw0;
    getCurrentPose(zed, x0, y0, yaw0);
    cout << "start pose: x=" << x0 << " y=" << y0 << " yaw=" << yaw0 << endl;

    // --- hardcoded target: 0.5m ahead in x from start ---
    // (swap this for a manual grid-coord * RESOLUTION conversion if simulating path planner output)
    vector<float> coord = {x0 + 0.5f, y0};
    cout << "target: x=" << coord[0] << " y=" << coord[1] << endl;

    // safety pause so you can get clear of the rover before it moves
    cout << "starting in 3 seconds..." << endl;
    this_thread::sleep_for(chrono::seconds(3));

    // --- main loop, matches main.cpp's inner while(!reached) ---
    bool reached = false;
    while (!reached) {
        float x, y, yaw;
        getCurrentPose(zed, x, y, yaw);
        float dy = coord[0] - y;
        float dx = coord[1] - x;
        float dist = sqrt(dx * dx + dy * dy);

        if (dist < TOLERANCE) {
            reached = true;
            cout << "reached target" << endl;
        } else {
            vector<int> wheel_speed = controlLoop(coord, zed);
            printWheels("wheel speeds", wheel_speed);

            bool motor_bool = sendInts(esp_motor, wheel_speed);
            if (!motor_bool) cout << "failed to send wheel speeds" << endl;

            this_thread::sleep_for(chrono::milliseconds((int)RATE));
        }
    }

    // --- stop motors on exit ---
    sendInts(esp_motor, {0, 0, 0, 0, 0, 0});

    closeSerial(esp_motor);
    zed.close();
    return 0;
}
