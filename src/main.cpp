#include "esp_comm.h"
#include "mapping.h"
#include <vector>
#include <termios.h>
int main(){
    std::vector<int> grid_map = RunMappingSession(10);

    int fd = openSerial("/dev/ttyUSB0", B115200);

    if(fd < 0){
        std::cout<< "failed to open serial port";
        return 1;
    }

    bool sent_succeed = sendInts(fd, grid_map);

    if(sent_succeed){
        std:: cout<< "sent succeeededdeded";
        return 0;
    } else {
        std::cout<<"print no goood";
        return 1;
    }
}
