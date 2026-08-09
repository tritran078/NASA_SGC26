#include <ESP32Servo.h>

Servo myServo;
const int servoPin = 12;
const int stepDelay = 30;  // ms between each degree step

// ---- packet-receive protocol to match sendInts() on the C++ side ----
const uint8_t HEADER1 = 0xAA, HEADER2 = 0x55;

uint8_t computeChecksum(uint8_t* data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

bool receivePacket(int32_t* out, uint8_t& count) {
    if (Serial.available() < 2) return false;
    uint8_t h1 = Serial.read();
    uint8_t h2 = Serial.read();
    if (h1 != HEADER1 || h2 != HEADER2) return false;

    while (Serial.available() < 2) {}
    uint8_t seq = Serial.read();
    count = Serial.read();

    int payloadBytes = count * sizeof(int32_t);
    while (Serial.available() < payloadBytes) {}
    Serial.readBytes((uint8_t*)out, payloadBytes);

    while (Serial.available() < 1) {}
    uint8_t checksum = Serial.read();

    if (computeChecksum((uint8_t*)out, payloadBytes) != checksum) return false;

    Serial.write(0x06);  // ACK
    return true;
}
// ---- END ----

void moveSlowly(int fromAngle, int toAngle) {
    if (fromAngle < toAngle) {
        for (int angle = fromAngle; angle <= toAngle; angle++) {
            myServo.write(angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = fromAngle; angle >= toAngle; angle--) {
            myServo.write(angle);
            delay(stepDelay);
        }
    }
}

void runTurnSequence() {
    moveSlowly(90, 0);
    delay(500);
    moveSlowly(0, 180);
    delay(500);
    moveSlowly(180, 90);
    delay(500);
}

void setup() {
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50);
    myServo.attach(servoPin, 500, 2400);
    myServo.write(90);  // start at 90 degrees, idle position
    delay(1000);
}

void loop() {
    int32_t buf[32];
    uint8_t count;

    if (receivePacket(buf, count)) {
        if (count == 1 && buf[0] == 1) {
            runTurnSequence();
        }
    }
}

