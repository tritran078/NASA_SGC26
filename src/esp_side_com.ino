const uint8_t HEADER1 = 0xAA, HEADER2 = 0x55;
const int CHUNK_SIZE_BYTES = 128;

int32_t receivedInts[4900];   // holds the full grid once received
int totalIntsReceived = 0;
uint8_t lastSeq = 255;

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(200);
}

void loop() {
    if (Serial.available() >= 4 && Serial.read() == 0xAA) {
        if (Serial.read() != 0x55) return;

        uint8_t seq = Serial.read();
        uint8_t count = Serial.read();

        uint8_t payload[CHUNK_SIZE_BYTES];
        Serial.readBytes(payload, count * 4);

        uint8_t checksum;
        Serial.readBytes(&checksum, 1);

        uint8_t computed = 0;
        for (int i = 0; i < count * 4; i++) computed ^= payload[i];

        if (computed == checksum) {
            if (seq == 0) {
                totalIntsReceived = 0;
            }
            if (seq != lastSeq) {
                memcpy(receivedInts + totalIntsReceived, payload, count * 4);
                totalIntsReceived += count;
                lastSeq = seq;
            }
            Serial.write(0x06);
        }
    }

    // ---- YOUR LOGIC GOES HERE ----
}
