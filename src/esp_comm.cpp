#include "esp_comm.h"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <vector>

using namespace std;

namespace {
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr uint8_t ACK_BYTE = 0x06;
constexpr int CHUNK_SIZE = 32;
constexpr int MAX_RETRIES = 5;
constexpr int ACK_TIMEOUT_MS = 200;
uint8_t nextPacketSequence = 0;

uint8_t computeChecksum(const uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; ++i) checksum ^= data[i];
    return checksum;
}

bool writeAll(int fd, const uint8_t* data, size_t size) {
    size_t total = 0;
    while (total < size) {
        ssize_t written = write(fd, data + total, size - total);
        if (written > 0) {
            total += static_cast<size_t>(written);
        } else if (written < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

bool readExact(int fd, uint8_t* data, size_t size, int maxTimeouts = 1) {
    size_t total = 0;
    int timeouts = 0;
    while (total < size) {
        ssize_t count = read(fd, data + total, size - total);
        if (count > 0) {
            total += static_cast<size_t>(count);
            timeouts = 0;
        } else if (count == 0) {
            if (++timeouts > maxTimeouts) return false;
        } else if (errno != EINTR) {
            return false;
        }
    }
    return true;
}

bool sendPacket(int fd, uint8_t sequence, const int32_t* payload, uint8_t count) {
    if (count == 0 || count > CHUNK_SIZE) return false;
    int payloadBytes = count * sizeof(int32_t);
    uint8_t packet[4 + CHUNK_SIZE * sizeof(int32_t) + 1];
    packet[0] = HEADER1;
    packet[1] = HEADER2;
    packet[2] = sequence;
    packet[3] = count;
    memcpy(packet + 4, payload, payloadBytes);
    packet[4 + payloadBytes] = computeChecksum(
        reinterpret_cast<const uint8_t*>(payload), payloadBytes
    );

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        if (writeAll(fd, packet, 4 + payloadBytes + 1)) {
            uint8_t response = 0;
            if (readExact(fd, &response, 1) && response == ACK_BYTE) return true;
        }
        cerr << "Retry seq " << static_cast<int>(sequence)
             << " attempt " << attempt + 1 << endl;
    }
    return false;
}

bool receivePacket(int fd, int32_t* output, uint8_t& count) {
    uint8_t header[2];
    if (!readExact(fd, header, sizeof(header))) return false;
    if (header[0] != HEADER1 || header[1] != HEADER2) return false;

    uint8_t sequence = 0;
    if (!readExact(fd, &sequence, 1) || !readExact(fd, &count, 1)) return false;
    if (count == 0 || count > CHUNK_SIZE) return false;

    int payloadBytes = count * sizeof(int32_t);
    if (!readExact(fd, reinterpret_cast<uint8_t*>(output), payloadBytes)) return false;

    uint8_t checksum = 0;
    if (!readExact(fd, &checksum, 1)) return false;
    if (computeChecksum(reinterpret_cast<const uint8_t*>(output), payloadBytes) != checksum) {
        return false;
    }
    return writeAll(fd, &ACK_BYTE, 1);
}
}  // namespace

int openSerial(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = ACK_TIMEOUT_MS / 100;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void closeSerial(int fd) {
    close(fd);
}

bool sendInts(int fd, const vector<int>& data) {
    vector<int32_t> integers(data.begin(), data.end());
    uint8_t sequence = nextPacketSequence;
    size_t offset = 0;
    while (offset < integers.size()) {
        int count = min(static_cast<size_t>(CHUNK_SIZE), integers.size() - offset);
        if (!sendPacket(fd, sequence, integers.data() + offset, count)) return false;
        offset += count;
        ++sequence;
    }
    nextPacketSequence = sequence;
    return true;
}

bool receiveInts(int fd, vector<int>& output, int expectedCount) {
    if (expectedCount < 0) return false;
    output.assign(expectedCount, 0);
    int received = 0;
    while (received < expectedCount) {
        int32_t chunk[CHUNK_SIZE];
        uint8_t count = 0;
        if (!receivePacket(fd, chunk, count)) return false;
        if (count > expectedCount - received) return false;
        for (int i = 0; i < count; ++i) output[received + i] = chunk[i];
        received += count;
    }
    return true;
}

bool writeGridAndPose(int fd, const vector<uint8_t>& gridMap, int16_t x, int16_t y) {
    vector<uint8_t> bytes = gridMap;
    twoway::encodePosition(bytes, x, y);
    return writeAll(fd, bytes.data(), bytes.size());
}

bool readWaypointBytes(int fd, vector<twoway::Waypoint>& waypoints,
                       bool& badData, bool& gaveUp, int maxTimeouts) {
    waypoints.clear();
    badData = false;
    gaveUp = false;

    uint8_t count = 0;
    if (!readExact(fd, &count, 1, maxTimeouts)) {
        gaveUp = true;
        return false;
    }
    if (count == twoway::BAD_DATA_SENTINEL) {
        badData = true;
        return false;
    }
    if (count > twoway::MAX_WAYPOINTS) {
        gaveUp = true;
        return false;
    }

    vector<uint8_t> bytes(static_cast<size_t>(count) * 4);
    if (!readExact(fd, bytes.data(), bytes.size(), maxTimeouts)) {
        gaveUp = true;
        return false;
    }
    waypoints = twoway::decodeResponse(count, bytes, badData, gaveUp);
    return !badData && !gaveUp;
}
