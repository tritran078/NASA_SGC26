#include "esp_comm.h"
#include "mapping.h"
#include "control.h"
#include <vector>
#include <termios.h>
#include <iostream>
#include <sl/Camera.hpp>
#include <chrono>

int main(){

    //init zed
    sl::Camera zed;
    sl::InitParameters init_param;
    init_param.camera_resolution = sl::RESOLUTION::VGA;
    init_param.camera_fps = 30;
    init_param.coordinate_units = sl::UNIT::METER;
    init_param.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    //init path length
    const int MAX_PATH_POINTS = 50; 
    const int PATH_SIZE = MAX_PATH_POINTS * 2; // 2 for x and y
    const float TOLERANCE = 0.20; //tolerance for navigation = 20cm

    const float RESOLUTION = 0.08;

    const float RATE = 50; //50ms = 20hz

    //test open zed cam
    if (zed.open(init_param) != sl::ERROR_CODE::SUCCESS) {
        std::cout << "failed to open ZED camera";
        return 1;
    }
    
    //positional tracking
    sl::PositionalTrackingParameters tracking_param;
    zed.enablePositionalTracking(tracking_param);

    int esp_Lora = openSerial("/dev/ttyUSB0", B115200); //open port to ESP LoRa
    int esp_cam_turn = openSerial("/dev/ttyUSB1", B115200); //open port to ESP camera turn
    int esp_motor = openSerial("/dev/ttyUSB2", B115200); //open port to ESP motor

    //if cannot open, return
    if(esp_Lora < 0){
        std::cout << "failed to open serial port to the LoRa ESP";
        zed.close();
        return 1;
    } else if (esp_cam_turn < 0){
        std::cout << "failed to open serial port to the camera turn ESP";
        zed.close();
        return 1;
    } else if (esp_motor < 0){
        std::cout << "failed to open serial port to the motor ESP";
        zed.close();
        return 1;
    }


    //make a timer to pull out tether from the car (5 seconds)
    std::this_thread::sleep_for(std::chrono::seconds(5));




    while(true){
        //signal to esp camera to start turning
        std::vector<int> start_signal = {1};
        bool turning = sendInts(esp_cam_turn, start_signal); //sendInts() signaling to start turning zed cam
        if (!turning) {std::cout << "failed to send start signal to camera turn ESP";}

        //mapping for 10 seconds
        std::vector<int> grid_map = RunMappingSession(zed, 10); //mapping


        //send the grid map to ESP LoRa
        bool sent_succeed = writeGridBytes(esp_Lora, grid_map); //send grid map to LoRa ESP
        if (!sent_succeed) {std::cout << "failed to send grid map to LoRa ESP";}

        //wait and receive the path from ESP LoRa
        std::vector<int> path_data;
        bool path_received = readWaypointBytes(esp_Lora, path_data);
        if (!path_received) {std::cout << "failed to receive path data from LoRa ESP";}



        // --------- CONTROL CODE ---------
        int count = 0;
        for(int i =0; i < (path_data.size() - 1)/2; i++){
            //get next coord
            std::vector<float> coord = {path_data[count] * RESOLUTION, path_data[count+1] * RESOLUTION};
            
            bool reached = false;
            
            while(!reached){
                //calculate distance from coord        
                float x,y,yaw;
                getCurrentPose(zed, x,y,yaw);
                float dy = coord[0] - y;
                float dx = coord[1] - x;
                
                //get tolerance radius
                float dist = sqrt(dx*dx + dy*dy);

                //check if reached
                if(dist < TOLERANCE){reached = true;}
                else{
                    //if not reached, get wheel speed for one time instance
                    std::vector<int> wheel_speed = controlLoop(coord, zed);
                    //send wheel speed
                    bool motor_bool = sendInts(esp_motor, wheel_speed);
                    if(!motor_bool){std::cout << "failed to send the coordinate";}
                    
                    //loop again at this rate
                    std::this_thread::sleep_for(std::chrono::milliseconds(RATE));
                }
            }
            count +=2; //increase count get the next coord
        }
    }

    closeSerial(esp_Lora);
    closeSerial(esp_cam_turn);
    closeSerial(esp_motor);
    zed.close();

}
