#pragma once
#include <vector>

int openSerial(const char* port, int baud);
void closeSerial(int fd);
bool sendInts(int fd, const std::vector<int>& data);


