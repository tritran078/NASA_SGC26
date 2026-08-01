// esp_motor.ino -- receives wheel speeds from Jetson (via esp_comm's sendInts
// protocol) and drives 6 TB6612FNG motor drivers accordingly.
//
// Wheel order matches control.cpp's return vector: {FL, FR, RL, RR, CL, CR}
// Each value is signed, range -255..255 (sign = direction, magnitude = PWM).

// ---- Direction pins (from your sample code) ----
#define FAI1 19   // Front Right (FR)
#define FAI2 21
#define FBI1 18   // Front Left (FL)
#define FBI2 4
#define MAI1 23   // Middle Right (CR)
#define MAI2 22
#define MBI1 13   // Middle Left (CL)
#define MBI2 15
#define RAI1 25   // Rear Right (RR)
#define RAI2 12
#define RBI1 33   // Rear Left (RL)
#define RBI2 32

// ---- PWM pins (one per motor, needed for TB6612FNG's PWMA/PWMB speed input) ----
// NOTE: pick real free GPIO pins on your board that don't collide with the
// direction pins above or your LoRa/UART wiring -- these are placeholders.
#define PWM_FR 27
#define PWM_FL 26
#define PWM_CR 14
#define PWM_CL 2
#define PWM_RR 5
#define PWM_RL 17

// ---- PWM config (ESP32 LEDC) ----
// NOTE: as of ESP32 Arduino core v3.x, ledcAttach(pin, freq, resolution)
// binds directly to the pin and replaces ledcSetup+ledcAttachPin. ledcWrite
// is now addressed by pin too, so explicit channel numbers are no longer needed.
const int PWM_FREQ = 5000;
const int PWM_RES_BITS = 8; // 0-255, matches your -255..255 range

// ---- esp_comm protocol constants (must match esp_comm.cpp exactly) ----
const uint8_t HEADER1 = 0xAA, HEADER2 = 0x55;
const int CHUNK_SIZE = 32;
const uint8_t ACK_BYTE = 0x06;

// ---- safety watchdog ----
const unsigned long WATCHDOG_TIMEOUT_MS = 250; // stop motors if no command within this window
unsigned long lastCommandMillis = 0;

uint8_t computeChecksum(const uint8_t* data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum ^= data[i];
    return sum;
}

// mirrors receivePacket() from esp_comm.cpp, using Arduino Serial instead of termios fd
bool receivePacket(int32_t* out, uint8_t& count) {
    uint8_t header[2];
    if (Serial.readBytes(header, 2) != 2) return false;
    if (header[0] != HEADER1 || header[1] != HEADER2) return false;

    uint8_t seq;
    Serial.readBytes(&seq, 1);
    Serial.readBytes(&count, 1);

    int payloadBytes = count * sizeof(int32_t);
    if (Serial.readBytes((uint8_t*)out, payloadBytes) != payloadBytes) return false;

    uint8_t checksum;
    Serial.readBytes(&checksum, 1);
    if (computeChecksum((uint8_t*)out, payloadBytes) != checksum) return false;

    Serial.write(ACK_BYTE);
    return true;
}

// mirrors receiveInts() from esp_comm.cpp
bool receiveInts(int* out, int expected_count) {
    int received = 0;
    while (received < expected_count) {
        int32_t chunk[CHUNK_SIZE];
        uint8_t count;
        if (!receivePacket(chunk, count)) continue;
        for (int i = 0; i < count && received + i < expected_count; i++)
            out[received + i] = chunk[i];
        received += count;
    }
    return true;
}

// ---- motor control ----
// dir pins set direction, PWM pin sets speed magnitude (0-255)
void setMotor(int pin1, int pin2, int pwmPin, int speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    } else if (speed < 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }

    ledcWrite(pwmPin, abs(speed));
}

void stopAllMotors() {
    setMotor(FAI1, FAI2, PWM_FR, 0);
    setMotor(FBI1, FBI2, PWM_FL, 0);
    setMotor(MAI1, MAI2, PWM_CR, 0);
    setMotor(MBI1, MBI2, PWM_CL, 0);
    setMotor(RAI1, RAI2, PWM_RR, 0);
    setMotor(RBI1, RBI2, PWM_RL, 0);
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(200); // matches ACK_TIMEOUT_MS on the Jetson side

    int dirPins[] = {FAI1, FAI2, FBI1, FBI2, MAI1, MAI2, MBI1, MBI2, RAI1, RAI2, RBI1, RBI2};
    for (int p : dirPins) pinMode(p, OUTPUT);

    ledcAttach(PWM_FR, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(PWM_FL, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(PWM_CR, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(PWM_CL, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(PWM_RR, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(PWM_RL, PWM_FREQ, PWM_RES_BITS);

    stopAllMotors();
    lastCommandMillis = millis();
}

void loop() {
    // watchdog: if we haven't heard from the Jetson in a while, stop the rover
    if (millis() - lastCommandMillis > WATCHDOG_TIMEOUT_MS) {
        stopAllMotors();
    }

    if (Serial.available() > 0) {
        int wheel_speed[6];
        if (receiveInts(wheel_speed, 6)) {
            lastCommandMillis = millis();

            // order matches control.cpp: {FL, FR, RL, RR, CL, CR}
            int FL = wheel_speed[0];
            int FR = wheel_speed[1];
            int RL = wheel_speed[2];
            int RR = wheel_speed[3];
            int CL = wheel_speed[4];
            int CR = wheel_speed[5];

            setMotor(FBI1, FBI2, PWM_FL, FL);
            setMotor(FAI1, FAI2, PWM_FR, FR);
            setMotor(RBI1, RBI2, PWM_RL, RL);
            setMotor(RAI1, RAI2, PWM_RR, RR);
            setMotor(MBI1, MBI2, PWM_CL, CL);
            setMotor(MAI1, MAI2, PWM_CR, CR);
        }
    }
}
