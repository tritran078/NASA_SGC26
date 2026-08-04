#include "esp_comm.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
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

// --- esp_Lora link only: raw byte protocol, matches Occupancy-Grid-Compression firmware ---

// Rover firmware expects GRID_ROWS*GRID_COLS raw cell bytes (0/1/2 per cell),
// no header, no checksum, followed by the rover's grid-cell position so the
// base station can place the local scan on the master map.
bool writeGridAndPose(int fd, const vector<uint8_t>& grid_map, int16_t x, int16_t y) {
    vector<uint8_t> bytes = grid_map;
    twoway::encodePosition(bytes, x, y);
    ssize_t written = write(fd, bytes.data(), bytes.size());
    return written == (ssize_t)bytes.size();
}

// Rover firmware sends back: [count:1][x:2][y:2] repeated, little-endian int16
// (or count == twoway::BAD_DATA_SENTINEL if the rover never got a real
// response). Blocks/retries across read timeouts (VTIME) instead of bailing
// on the first empty read, since the ESP32 may still be mid-LoRa-roundtrip.
bool readWaypointBytes(int fd, vector<twoway::Waypoint>& waypoints, bool& badData, bool& gaveUp,
                        int max_timeouts) {
    waypoints.clear();
    badData = false;
    gaveUp = false;

    uint8_t count;
    int timeouts = 0;
    while (read(fd, &count, 1) != 1) {
        if (++timeouts > max_timeouts) { gaveUp = true; return false; }
    }

    if (count == twoway::BAD_DATA_SENTINEL) {
        badData = true;
        return true;
    }

    vector<uint8_t> wp_bytes(static_cast<size_t>(count) * 4);
    size_t total = 0;
    timeouts = 0;
    while (total < wp_bytes.size()) {
        ssize_t n = read(fd, wp_bytes.data() + total, wp_bytes.size() - total);
        if (n <= 0) {
            if (++timeouts > max_timeouts) { gaveUp = true; return false; }
            continue;
        }
        total += n;
        timeouts = 0;
    }

    waypoints = twoway::decodeResponse(count, wp_bytes, badData, gaveUp);
    return !badData && !gaveUp;
}