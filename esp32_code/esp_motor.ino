class Motor {
private:
  const uint8_t pwm_pin;
  const uint8_t in_1_pin;
  const uint8_t in_2_pin;
  unsigned int power;

public:

  Motor(uint8_t pwm_pin, uint8_t in_1_pin, uint8_t in_2_pin, unsigned int power)
    : pwm_pin(pwm_pin), in_1_pin(in_1_pin), in_2_pin(in_2_pin), power(power) {

    pinMode(pwm_pin, OUTPUT);
    pinMode(in_1_pin, OUTPUT);
    pinMode(in_2_pin, OUTPUT);

    stop();
  }

  void setPower(unsigned int new_power) {
    power = new_power;
  }

  void run() {
    analogWrite(pwm_pin, power);
  }

  void forward() {
    digitalWrite(in_1_pin, HIGH);
    digitalWrite(in_2_pin, LOW);
  }

  void reverse() {
    digitalWrite(in_1_pin, LOW);
    digitalWrite(in_2_pin, HIGH);
  }

  void stop() {
    analogWrite(pwm_pin, 0);
  }
};

Motor left_mecanum(27, 23, 4, 100);
Motor right_mecanum(12, 18, 25, 100);
Motor left_omni(14, 19, 32, 100);
Motor right_omni(13, 26, 33, 100);

// applies a signed power value to one side (two motors): sign sets direction,
// magnitude sets PWM. Used by --drive to replace the old discrete
// forward/reverse/rotate command set with continuous differential control.
void setSidePower(Motor& m1, Motor& m2, int signed_power) {
  if (signed_power >= 0) {
    m1.forward();
    m2.forward();
  } else {
    m1.reverse();
    m2.reverse();
  }
  unsigned int mag = (unsigned int) abs(signed_power);
  m1.setPower(mag);
  m2.setPower(mag);
  m1.run();
  m2.run();
}

void setup() {
  Serial.begin(115200);

  left_omni.stop();
  left_mecanum.stop();
  right_omni.stop();
  right_mecanum.stop();
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil(' ');
    if (cmd.length() > 0) {
      cmd.trim();
      Serial.print("Command: ");

      if (cmd.equals("--drive")) {
        int left_power = Serial.readStringUntil(' ').toInt();
        int right_power = Serial.readStringUntil(' ').toInt();
        Serial.print(cmd);
        Serial.print(" ");
        Serial.print(left_power);
        Serial.print(" ");
        Serial.println(right_power);

        setSidePower(left_omni, left_mecanum, left_power);
        setSidePower(right_omni, right_mecanum, right_power);

      } else if (cmd.equals("--forward")) {
        Serial.println(cmd);
        left_omni.forward();
        left_mecanum.forward();
        right_omni.forward();
        right_mecanum.forward();

        left_mecanum.run();
        left_omni.run();
        right_mecanum.run();
        right_omni.run();

      } else if (cmd.equals("--reverse")) {
        Serial.println(cmd);
        left_omni.reverse();
        left_mecanum.reverse();
        right_omni.reverse();
        right_mecanum.reverse();

        left_mecanum.run();
        left_omni.run();
        right_mecanum.run();
        right_omni.run();

      } else if (cmd.equals("--rotate-left")) {
        Serial.println(cmd);
        right_omni.reverse();
        right_mecanum.reverse();
        left_omni.forward();
        left_mecanum.forward();

        left_mecanum.run();
        left_omni.run();
        right_mecanum.run();
        right_omni.run();

      } else if (cmd.equals("--rotate-right")) {
        Serial.println(cmd);
        left_omni.reverse();
        left_mecanum.reverse();
        right_omni.forward();
        right_mecanum.forward();

        left_mecanum.run();
        left_omni.run();
        right_mecanum.run();
        right_omni.run();

      } else if (cmd.equals("--set-power") || cmd.equals("--sp")) {
        Serial.println(cmd);
        int power = Serial.readStringUntil(' ').toInt();
        left_omni.setPower(power);
        left_mecanum.setPower(power);
        right_omni.setPower(power);
        right_mecanum.setPower(power);

      } else if (cmd.equals("--stop")) {
        Serial.println(cmd);
        left_omni.stop();
        left_mecanum.stop();
        right_omni.stop();
        right_mecanum.stop();
      }
    }
  }
}
