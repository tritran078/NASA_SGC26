#include "ESP32Servo.h"
Servo myservo;
// Instead of degrees (write), use microseconds (writeMicroseconds) for
// much finer control near the dead zone. Typical range is 1000-2000us,
// with ~1500us = stop. Start near 1500 and nudge by small amounts.
int pulse1 = 1300;  // try values like 1465, 1470, 1475... instead of jumping by "1 degree"
int pulse2 = 1700;  // same idea for the other direction
const byte servoPin = 12;   // for rp2040

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

  Serial.write(0x06); // ACK
  return true;
}
// ---- END ----

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin);   // attach ONCE here, not inside loop()
  myservo.writeMicroseconds(1500); // start stopped
  delay(500);
}

void loop() {
  // wait for signal before turning
  int32_t buf[32];
  uint8_t count;

  if (receivePacket(buf, count)) {
    if (count == 1 && buf[0] == 1) {

      myservo.writeMicroseconds(pulse1);
      delay(15000);
      myservo.writeMicroseconds(pulse2);
      delay(15000);
        myservo.writeMicroseconds(1500); // stop
    }
  }
}
