//control code that:
//receive next coord from path
//compute dx and dy from current and next point
//compute vx and vy needed proportional to constant KP
//compute the individual wheel speed for each fo the 6 wheels
//return the wheel speed to main

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include "esp_comm.h"
#include <sl/Camera.hpp>

using namespace std;

const float KP = 600.0;



void getCurrentPose(sl::Camera& zed, float& x, float& y, float& yaw) {
    sl::Pose pose;
    zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    x = pose.getTranslation().x;
    y = pose.getTranslation().y;
    auto o = pose.getOrientation();
    yaw = atan2(2*(o.oz*o.ow + o.ox*o.oy), 1 - 2*(o.oy*o.oy + o.oz*o.oz));
}

vector<int> controlLoop(vector<float> next_coord,sl::Camera& zed){
    //delcare the pose value and get current pose
    float x,y,yaw;
    getCurrentPose(zed, x, y, yaw);

    //get the next coord in real world
    float y_coord = next_coord[0];
    float x_coord = next_coord[1];

    //calculate dy and dx
    float dy = y_coord - y;
    float dx = x_coord - x;

    //calculate vx and vy
    float vx = KP * dx;
    float vy = KP * dy;

    //calculate the individual wheel speed
    float FL =  vx - vy;
    float FR =  vx + vy;
    float CL =  vx;
    float CR =  vx;
    float RL =  vx + vy;
    float RR =  vx - vy;

    float maxWheel = max(max(max(abs(FL), abs(FR)), max(abs(RL), abs(RR))),max(abs(CL), abs(CR)));

    if (maxWheel > 255.0) {
        float scale = 255.0 / maxWheel;
        FL *= scale;
        FR *= scale;
        RL *= scale;
        RR *= scale;
        CL *= scale;
        CR *= scale;
    }

    vector<int> wheel_speed = {int(FL), int(FR), int(RL), int(RR), int(CL), int(CR)};

    //return the wheel speed
    return wheel_speed;
}
