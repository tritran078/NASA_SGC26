#pragma once
#include <vector>
#include <sl/Camera.hpp>

void getCurrentPose(sl::Camera& zed, float& x, float& y, float& yaw);
std::vector<int> controlLoop(std::vector<float> next_coord, sl::Camera& zed);