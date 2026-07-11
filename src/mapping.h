#pragma once
#include <vector>
#include <sl/Camera.hpp>
using namespace std;

struct OccupancyGrid {
    int map_width, map_height;
    float resolution;
    float fx, fy, cx, cy;
    const float log_occ = 0.85;
    const float log_free = -0.4;
    vector<float> data;

    OccupancyGrid(int w, int h, float res, float fx_, float fy_, float cx_, float cy_);

    vector<float> getStrip(sl::Mat& depth_map);
    vector<pair<float,float>> polar_Cartessian(vector<float> map_strip);
    vector<pair<float,float>> robot_to_world(vector<pair<float,float>> robot_frame_map, float yaw0, sl::Camera& zed);
    void updateAll(int robot_pos_x, int robot_pos_y, vector<pair<float,float>>& robot_frame_map);
    void bresenham(int x0, int y0, int x1, int y1);
    void updateCell(int x, int y, float delta);
    vector<int> getBinaryMap(float free_thresh = 0.4, float occ_thresh = 0.6);
};

vector<int> RunMappingSession(int scan_duration = 10);

