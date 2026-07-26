const uint8_t HEADER1 = 0xAA, HEADER2 = 0x55;
const int CHUNK_SIZE_BYTES = 128;
const int CHUNK_SIZE = 32;
const int MAX_RETRIES = 5;
const int ACK_TIMEOUT_MS = 200;

int32_t receivedInts[4900];
int totalIntsReceived = 0;
uint8_t lastSeq = 255;

// ---- send function, for handing path data to Jetson ----
uint8_t computeChecksum(uint8_t* data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

bool sendPacket(uint8_t seq, int32_t* payload, uint8_t count) {
    int payloadBytes = count * sizeof(int32_t);
    uint8_t checksum = computeChecksum((uint8_t*)payload, payloadBytes);
    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        Serial.write(HEADER1);
        Serial.write(HEADER2);
        Serial.write(seq);
        Serial.write(count);
        Serial.write((uint8_t*)payload, payloadBytes);
        Serial.write(checksum);
        unsigned long start = millis();
        while (millis() - start < ACK_TIMEOUT_MS) {
            if (Serial.available()) {
                if (Serial.read() == 0x06) return true;
                break;
            }
        }
    }
    return false;
}

bool sendInts(int32_t* data, int totalCount) {
    uint8_t seq = 0;
    int offset = 0;
    while (offset < totalCount) {
        int count = min(CHUNK_SIZE, totalCount - offset);
        if (!sendPacket(seq, data + offset, count)) return false;
        offset += count;
        seq++;
    }
    return true;
}
// ---- END send function ----

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(200);
}

void loop() {
    // existing map-receive-from-Jetson logic, unchanged
    if (Serial.available() >= 4 && Serial.read() == 0xAA) {
        // ... (unchanged) ...
    }

    // Muhammad/Cao's LoRa receive + decode logic goes here,
    // ending with a call to: sendInts(path_data, count);
}
