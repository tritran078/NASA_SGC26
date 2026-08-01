#pragma once
#include <vector>

int openSerial(const char* port, int baud);
void closeSerial(int fd);
bool sendInts(int fd, const std::vector<int>& data);
bool receiveInts(int fd, std::vector<int>& out, int expected_count);

bool writeGridBytes(int fd, const std::vector<int>& grid_map);
bool readWaypointBytes(int fd, std::vector<int>& path_data, int max_timeouts = 50);