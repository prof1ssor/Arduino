#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo PWM constants (adjust for your servos)
#define SERVOMIN  150  // Min pulse length for servos
#define SERVOMAX  600  // Max pulse length for servos
#define STOP_PWM 307
#define FORWARD_PWM 350
#define REVERSE_PWM 260

// Servo channel assignments
int Servo1 = 0, Servo2 = 1, Servo3 = 2, Servo4 = 3, Servo5 = 4;  // Right arm
int Servo6 = 12, Servo7 = 13, Servo8 = 9, Servo9 = 10, Servo10 = 11; // Left arm

// Offset values for calibration
int servo1Offset = 0, servo2Offset = -10, servo3Offset = 0, servo4Offset = -20, servo5Offset = 0;
int servo6Offset = -20, servo7Offset = -10, servo8Offset = 0, servo9Offset = 0, servo10Offset = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685 for Servo Control...");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set PWM frequency to 50Hz for servos

  delay(10); // Wait for the PCA9685 to initialize

  // Move all servos to their zero positions
  calibrateServos();
}

void loop() {
  // Your loop logic here (e.g., controlling servos dynamically)
}

void calibrateServos() {
  Serial.println("Calibrating Servos...");

  // Set right arm servos to zero position
  setServoAngle(Servo1, 0 + servo1Offset);
  setServoAngle(Servo2, 0 + servo2Offset);
  setServoAngle(Servo3, 0 + servo3Offset);
  setServoAngle(Servo4, 0 + servo4Offset);
  setServoAngle(Servo5, 0 + servo5Offset);

  // Set left arm servos to zero position (inverted for alignment)
  setServoAngle(Servo6, 0 + servo6Offset);
  setServoAngle(Servo7, 0 + servo7Offset);
  setServoAngle(Servo8, 0 + servo8Offset);
  setServoAngle(Servo9, 0 + servo9Offset);
  setServoAngle(Servo10, 0 + servo10Offset);

  delay(2000); // Observe positions
}

// Function to set the angle of a servo using PCA9685
void setServoAngle(int channel, int angle) {
  // Map angle (0-180) to pulse length (SERVOMIN-SERVOMAX)
  int pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulselen);
}
