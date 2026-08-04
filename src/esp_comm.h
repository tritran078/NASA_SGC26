#pragma once
#include <cstdint>
#include <vector>

#include "two_way_protocol.h"

int openSerial(const char* port, int baud);
void closeSerial(int fd);
bool sendInts(int fd, const std::vector<int>& data);
bool receiveInts(int fd, std::vector<int>& out, int expected_count);

bool writeGridAndPose(int fd, const std::vector<uint8_t>& grid_map, int16_t x, int16_t y);
bool readWaypointBytes(int fd, std::vector<twoway::Waypoint>& waypoints, bool& badData, bool& gaveUp,
                        int max_timeouts = 50);