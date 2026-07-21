#include "esp_comm.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <vector>
#include <iostream>

using namespace std;

const uint8_t HEADER1 = 0xAA, HEADER2 = 0x55;
const int CHUNK_SIZE = 32;
const int MAX_RETRIES = 5;
const int ACK_TIMEOUT_MS = 200;

int openSerial(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = ACK_TIMEOUT_MS / 100;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

void closeSerial(int fd) {
    close(fd);
}

uint8_t computeChecksum(const uint8_t* data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

// packet layout: [0xAA][0x55][seq][count][payload...][checksum]
bool sendPacket(int fd, uint8_t seq, const int32_t* payload, uint8_t count) {
    int payloadBytes = count * sizeof(int32_t);
    uint8_t checksum = computeChecksum((uint8_t*)payload, payloadBytes);

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        uint8_t packet[4 + CHUNK_SIZE * sizeof(int32_t) + 1];
        packet[0] = HEADER1;
        packet[1] = HEADER2;
        packet[2] = seq;
        packet[3] = count;
        memcpy(packet + 4, payload, payloadBytes);
        packet[4 + payloadBytes] = checksum;

        write(fd, packet, 4 + payloadBytes + 1);

        uint8_t response;
        int n = read(fd, &response, 1);
        if (n == 1 && response == 0x06) return true;  // ACK

        cerr << "Retry seq " << (int)seq << " attempt " << (attempt + 1) << endl;
    }
    return false;
}

// one generic function — works for grid, waypoints, or scan command, on any port/target
bool sendInts(int fd, const vector<int>& data) {
    vector<int32_t> ints(data.begin(), data.end());
    uint8_t seq = 0;
    size_t offset = 0;

    while (offset < ints.size()) {
        int count = min((size_t)CHUNK_SIZE, ints.size() - offset);
        bool ok = sendPacket(fd, seq, ints.data() + offset, count);
        if (!ok) {
            cerr << "Failed to send packet " << (int)seq << endl;
            return false;
        }
        offset += count;
        seq++;
    }
    return true;
}

// esp_comm.cpp — add
bool receivePacket(int fd, int32_t* out, uint8_t& count) {
    uint8_t header[2];
    if (read(fd, header, 2) != 2) return false;
    if (header[0] != HEADER1 || header[1] != HEADER2) return false;

    uint8_t seq;
    read(fd, &seq, 1);
    read(fd, &count, 1);

    int payloadBytes = count * sizeof(int32_t);
    if (read(fd, out, payloadBytes) != payloadBytes) return false;

    uint8_t checksum;
    read(fd, &checksum, 1);
    if (computeChecksum((uint8_t*)out, payloadBytes) != checksum) return false;

    uint8_t ack = 0x06;
    write(fd, &ack, 1);
    return true;
}

bool receiveInts(int fd, std::vector<int>& out, int expected_count) {
    out.assign(expected_count, 0);
    int received = 0;
    while (received < expected_count) {
        int32_t chunk[CHUNK_SIZE];
        uint8_t count;
        if (!receivePacket(fd, chunk, count)) continue;
        for (int i = 0; i < count && received + i < expected_count; i++)
            out[received + i] = chunk[i];
        received += count;
    }
    return true;
}