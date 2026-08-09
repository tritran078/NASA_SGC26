#include "esp_comm.h"
#include "mapping.h"
#include "control.h"
#include <vector>
#include <termios.h>
#include <iostream>
#include <sl/Camera.hpp>
#include <chrono>
#include <cmath>
#include <thread>

int main(){

    //-------------- ZED Camera Setup ------------
    sl::Camera zed;
    sl::InitParameters init_param;
    init_param.camera_resolution = sl::RESOLUTION::VGA;
    init_param.camera_fps = 30;
    init_param.coordinate_units = sl::UNIT::METER;
    init_param.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;


    //-------------- Constants ------------
    const float TOLERANCE = 0.05; //tolerance for navigation = 20cm

    const float RESOLUTION = 0.08; //resolution of the grid map = 8cm

    const int RATE = 50; //50ms = 20hz


    //test open zed cam
    if (zed.open(init_param) != sl::ERROR_CODE::SUCCESS) {
        std::cout << "failed to open ZED camera";
        return 1;
    }
    
    //positional tracking
    sl::PositionalTrackingParameters tracking_param;
    zed.enablePositionalTracking(tracking_param);
    //open UART serial connections to esp32
    int esp_Lora = openSerial("/dev/ttyUSB0", B115200); //open port to ESP LoRa
    int esp_cam_turn = openSerial("/dev/ttyUSB1", B115200); //open port to ESP camera turn
    //int esp_motor = openSerial("/dev/ttyUSB2", B115200); //open port to ESP motor


    //if cannot open, return
    
    if(esp_Lora < 0){
        std::cout << "failed to open serial port to the LoRa ESP";
        zed.close();
        return 1;
    } else if (esp_cam_turn < 0){
        std::cout << "failed to open serial port to the camera turn ESP";
        zed.close();
        return 1;
    } /* else if (esp_motor < 0){
        std::cout << "failed to open serial port to the motor ESP";
        zed.close();
        return 1;
    } */




    //make a timer to pull out tether from the car (5 seconds)
    std::this_thread::sleep_for(std::chrono::seconds(5));




    //---------------------- MAIN LOOP ----------------------
    while(true){
        
       //--------- CAMERA TURNING CODE ---------
        std::vector<int> start_signal = {1};
        bool turning = sendInts(esp_cam_turn, start_signal); //sendInts() signaling to start turning zed cam
        if (!turning) {std::cout << "failed to send start signal to camera turn ESP";}
        
        //--------- MAPPING CODE ---------
        std::vector<uint8_t> grid_map = RunMappingSession(zed, 15); //mapping


        //--------- GET CURRENT POSE ---------
        float pose_x, pose_y, pose_yaw;
        getCurrentPose(zed, pose_x, pose_y, pose_yaw);
        int16_t grid_x = static_cast<int16_t>(std::round(pose_x / RESOLUTION));
        int16_t grid_y = static_cast<int16_t>(std::round(pose_y / RESOLUTION));
        
        //--------- SEND GRID MAP AND POSE TO ESP LORA ---------
        bool sent_succeed = writeGridAndPose(esp_Lora, grid_map, grid_x, grid_y); //send grid map to LoRa ESP
        if (!sent_succeed) {std::cout << "failed to send grid map to LoRa ESP";}

        //--------- RECEIVE WAYPOINTS FROM ESP LORA ---------
        std::vector<twoway::Waypoint> waypoints;
        bool badData = false, gaveUp = false;
        bool path_received = readWaypointBytes(esp_Lora, waypoints, badData, gaveUp);
        if (!path_received) {
            if (badData) { std::cout << "LoRa ESP reported no path data"; }
            else if (gaveUp) { std::cout << "gave up waiting for path data from LoRa ESP"; }
            continue; //nothing to drive to this cycle
        }

    
        // --------------print path in terminal-----------------
        std::cout << "path: ";
        for (const twoway::Waypoint& wp : waypoints) {
            std::cout << "(" << wp.x << "," << wp.y << ") ";
            }
        std::cout << std::endl;
        /*
       std::vector<twoway::Waypoint> waypoints = {
        {6, 0},
        {6, 6},
        {12, 6},
        {12, 0}
        };
        
        //std::vector<twoway::Waypoint> waypoints = {{6, 0},{6, -6}}; tested path

        // --------- CONTROL CODE ---------
        float current_x = 0.0f;
        float current_y = 0.0f;

        for (const twoway::Waypoint& wp : waypoints) {
            float next_x = wp.x * RESOLUTION;
            float next_y = wp.y * RESOLUTION;

            bool sent = sendSetLocation(esp_motor, current_x, current_y, next_x, next_y);
            if (!sent) {
                std::cout << "failed to send waypoint (" << next_x << ", " << next_y << "), aborting path\n";
                break;
            }

            std::cout << "navigating to (" << next_x << ", " << next_y << ") ... waiting 10s\n";
            std::this_thread::sleep_for(std::chrono::seconds(10));

            current_x = next_x;
            current_y = next_y;
        }
*/
        std::this_thread::sleep_for(std::chrono::seconds(10)); //wait for 100 seconds before next cycle

    }

    //close serial connections and zed cam
    closeSerial(esp_Lora);
    closeSerial(esp_cam_turn);
    //closeSerial(esp_motor);
    zed.close();

}

