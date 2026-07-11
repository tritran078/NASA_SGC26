#include "mapping.h"
#include <cmath>
#include <chrono>
using namespace std;

OccupancyGrid::OccupancyGrid(int w, int h, float res, float fx_, float fy_, float cx_, float cy_) {
    map_width = w; map_height = h; resolution = res;
    fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    data.assign(map_width * map_height, 0.0f);
}

vector<float> OccupancyGrid::getStrip(sl::Mat& depth_map) {
    int cam_width = depth_map.getWidth();
    int cam_height = depth_map.getHeight() / 2;
    float* ptr = depth_map.getPtr<float>();
    vector<float> map_strip(ptr + cam_width*cam_height, ptr + (cam_width*cam_height) + cam_width);
    return map_strip;
}

vector<pair<float,float>> OccupancyGrid::polar_Cartessian(vector<float> map_strip) {
    vector<pair<float,float>> robot_frame_map;
    for (int i = 0; i < map_strip.size(); i++) {
        float depth = map_strip[i];
        if (depth <= 0 || isnan(depth) || isinf(depth)) continue;
        float X = (i - cx) * map_strip[i] / fx;
        float Y = map_strip[i];
        robot_frame_map.push_back({X, Y});
    }
    return robot_frame_map;
}

vector<pair<float,float>> OccupancyGrid::robot_to_world(vector<pair<float,float>> robot_frame_map, float yaw0, sl::Camera& zed) {
    sl::Pose pose;
    zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    sl::Orientation orientation = pose.getOrientation();
    float yaw1 = atan2(2*(orientation.oz*orientation.ow + orientation.ox*orientation.oy),
                1 - 2*(orientation.oy*orientation.oy + orientation.oz*orientation.oz));
    float phi = yaw0 - yaw1;
    for (int i = 0; i < robot_frame_map.size(); i++) {
        float world_x = robot_frame_map[i].first * cos(phi) - robot_frame_map[i].second * sin(phi);
        float world_y = robot_frame_map[i].first * sin(phi) + robot_frame_map[i].second * cos(phi);
        robot_frame_map[i] = {world_x, world_y};
    }
    return robot_frame_map;
}

void OccupancyGrid::updateAll(int robot_pos_x, int robot_pos_y, vector<pair<float,float>>& robot_frame_map) {
    for (int i = 0; i < robot_frame_map.size(); i++) {
        float world_x = robot_frame_map[i].first;
        float world_y = robot_frame_map[i].second;
        int cell_x = int(world_x / resolution) + map_width / 2;
        int cell_y = int(world_y / resolution) + map_height / 2;
        bresenham(robot_pos_x, robot_pos_y, cell_x, cell_y);
    }
}

void OccupancyGrid::bresenham(int x0, int y0, int x1, int y1) {
    int dx = x1 - x0; if (dx < 0) dx = -dx;
    int dy = y1 - y0; if (dy < 0) dy = -dy;
    dy = -dy;
    int step_x = (x0 < x1) ? 1 : -1;
    int step_y = (y0 < y1) ? 1 : -1;
    int error = dx + dy;

    while (true) {
        bool reachedEnd = (x0 == x1) && (y0 == y1);
        if (reachedEnd) { updateCell(x0, y0, log_occ); break; }
        updateCell(x0, y0, log_free);
        int error2 = 2 * error;
        if (error2 >= dy) { error += dy; x0 += step_x; }
        if (error2 <= dx) { error += dx; y0 += step_y; }
    }
}

void OccupancyGrid::updateCell(int x, int y, float delta) {
    if (x < 0 || x >= map_width || y < 0 || y >= map_height) return;
    int index = y * map_width + x;
    data[index] += delta;
    data[index] = std::max(-5.0f, std::min(5.0f, data[index]));
}

vector<int> OccupancyGrid::getBinaryMap(float free_thresh, float occ_thresh) {
    vector<int> binary_map(map_width * map_height, 2);
    for (int i = 0; i < map_width * map_height; ++i) {
        float prob = 1.0f - 1.0f / (1.0f + exp(data[i]));
        if (prob > occ_thresh) binary_map[i] = 1;
        else if (prob < free_thresh) binary_map[i] = 0;
    }
    return binary_map;
}



vector<int> RunMappingSession(int scan_duration){
    sl::Camera zed; 
    sl::InitParameters init_param;
    init_param.camera_resolution = sl::RESOLUTION::VGA;
    init_param.camera_fps = 30;
    init_param.coordinate_units = sl::UNIT::METER;
    init_param.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    if(zed.open(init_param) != sl::ERROR_CODE::SUCCESS){
        return{};
    }

    sl::PositionalTrackingParameters tracking_param;
    zed.enablePositionalTracking(tracking_param);

    sl::Pose pose;
    zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    sl::Orientation orientation = pose.getOrientation();

    float yaw0 = atan2(2*(orientation.oz*orientation.ow + orientation.ox*orientation.oy),
                    1 - 2*(orientation.oy*orientation.oy + orientation.oz*orientation.oz));

    auto calib = zed.getCameraInformation().camera_configuration.calibration_parameters.left_cam;
    float fx = calib.fx, fy = calib.fy, cx = calib.cx, cy = calib.cy;

    //create the grid map with the basic info
    OccupancyGrid grid_map(70 , 70, 0.08, fx, fy, cx, cy);

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
 	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();
	if (elapsed >= scan_duration){ break; }

        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            sl::Mat depth_map;
            zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH);

            vector<float> strip = grid_map.getStrip(depth_map);
            vector<pair<float,float>> local_pts = grid_map.polar_Cartessian(strip);
            vector<pair<float,float>> world_pts = grid_map.robot_to_world(local_pts, yaw0, zed);

            int robot_x = grid_map.map_width / 2;
            int robot_y = grid_map.map_height / 2; 

            grid_map.updateAll(robot_x, robot_y, world_pts);
        }
    }
    zed.close();  

    vector<int> binary_map = grid_map.getBinaryMap();

    return binary_map;
}

