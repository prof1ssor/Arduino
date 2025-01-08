#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create a PCA9685 driver instance on the default I2C address (0x40).
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*******************************************************************
   Adjust these for YOUR continuous servo:
   - STOP: servo holds still
   - CW:   servo spins clockwise
   - CCW:  servo spins counterclockwise
********************************************************************/
#define STOP 375  
#define CW   337
#define CCW  540

// Choose the PCA9685 channel (0..15) to which your servo is connected:
#define SERVO_CHANNEL 15

void setup() {
  Serial.begin(9600);
  Serial.println("Continuous Servo Control via Serial Commands");
  Serial.println("Commands:");
  Serial.println("  w = clockwise");
  Serial.println("  s = counterclockwise");
  Serial.println("  x = stop");

  // Initialize the PCA9685 and set frequency
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // 50 Hz typical for servos

  // Optionally start servo in STOP position
  pwm.setPWM(SERVO_CHANNEL, 0, STOP);
}

void loop() {
  // Check if there's any incoming character from the serial
  if (Serial.available() > 0) {
    char command = Serial.read();

    // If the command is '\n' (newline) or '\r' (carriage return), ignore it
    if (command == '\n' || command == '\r') {
      return; // Do nothing
    }

    switch (command) {
      case 'w':
        Serial.println("Servo rotating clockwise");
        pwm.setPWM(SERVO_CHANNEL, 0, CW);
        break;
      case 's':
        Serial.println("Servo rotating counterclockwise");
        pwm.setPWM(SERVO_CHANNEL, 0, CCW);
        break;
      case 'x':
        Serial.println("Servo rotating counterclockwise");
        pwm.setPWM(SERVO_CHANNEL, 0, STOP);
        break;
      // ...
      default:
        Serial.print("Unknown command: ");
        Serial.println(command);
        break;
    }
  }

  // No need for delays; servo state remains until a new command arrives.
}
