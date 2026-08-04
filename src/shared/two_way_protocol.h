#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>

// Wire format shared by the Jetson <-> LoRa-ESP serial leg and the
// base-station computer <-> BASE-ESP serial leg:
//   waypoint response: [count:1][x:2][y:2]*count, little-endian int16.
//   count == BAD_DATA_SENTINEL (0xFF) means the sender is explicitly
//   signaling "no data" rather than a real zero-length list.
namespace twoway {

struct Waypoint {
    int16_t x;
    int16_t y;
};

constexpr uint8_t BAD_DATA_SENTINEL = 0xFF;
constexpr size_t MAX_WAYPOINTS = 64;

// Appends [x:2][y:2] little-endian onto an existing byte buffer, e.g. after
// grid cell bytes, so the rover's position rides along with the grid.
inline void encodePosition(std::vector<uint8_t>& bytes, int16_t x, int16_t y) {
    bytes.push_back(static_cast<uint8_t>(x & 0xFF));
    bytes.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    bytes.push_back(static_cast<uint8_t>(y & 0xFF));
    bytes.push_back(static_cast<uint8_t>((y >> 8) & 0xFF));
}

// [count:1][x:2][y:2]*count, little-endian. Caps at MAX_WAYPOINTS since the
// receiving decoder won't accept a longer list.
inline std::vector<uint8_t> encodeGoodResponse(const std::vector<Waypoint>& waypoints) {
    size_t count = std::min(waypoints.size(), MAX_WAYPOINTS);
    std::vector<uint8_t> bytes;
    bytes.reserve(1 + count * 4);
    bytes.push_back(static_cast<uint8_t>(count));
    for (size_t i = 0; i < count; i++) {
        const Waypoint& w = waypoints[i];
        bytes.push_back(static_cast<uint8_t>(w.x & 0xFF));
        bytes.push_back(static_cast<uint8_t>((w.x >> 8) & 0xFF));
        bytes.push_back(static_cast<uint8_t>(w.y & 0xFF));
        bytes.push_back(static_cast<uint8_t>((w.y >> 8) & 0xFF));
    }
    return bytes;
}

// Decodes a [x:2][y:2]*count payload given the count byte read separately.
// Sets badData if count was the bad-data sentinel, gaveUp if payload is
// shorter than count implies (truncated read upstream).
inline std::vector<Waypoint> decodeResponse(uint8_t count, const std::vector<uint8_t>& payload,
                                             bool& badData, bool& gaveUp) {
    badData = false;
    gaveUp = false;
    std::vector<Waypoint> waypoints;

    if (count == BAD_DATA_SENTINEL) {
        badData = true;
        return waypoints;
    }

    size_t needed = static_cast<size_t>(count) * 4;
    if (payload.size() < needed) {
        gaveUp = true;
        return waypoints;
    }

    waypoints.reserve(count);
    for (size_t i = 0; i < count; i++) {
        int16_t x = static_cast<int16_t>(payload[i * 4] | (payload[i * 4 + 1] << 8));
        int16_t y = static_cast<int16_t>(payload[i * 4 + 2] | (payload[i * 4 + 3] << 8));
        waypoints.push_back(Waypoint{x, y});
    }
    return waypoints;
}

} // namespace twoway
