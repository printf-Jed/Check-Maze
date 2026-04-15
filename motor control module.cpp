# #include <Wire.h>
#include <MPU6050_light.h>

/* --- Pin Definitions (Blue Pill) --- */
// Motor A (Left)
#define MOT_L_PWM  PA8   // PWM pin (Timer 1)
#define MOT_L_IN1  PB12  // Direction 1
#define MOT_L_IN2  PB13  // Direction 2

// Motor B (Right)
#define MOT_R_PWM  PA9   // PWM pin (Timer 1)
#define MOT_R_IN1  PB14  // Direction 1
#define MOT_R_IN2  PB15  // Direction 2

/* --- PID Constants for Heading Control --- */
float Kp = 4.5;
float Ki = 0.05;
float Kd = 1.2;

/* --- Global Variables --- */
MPU6050 mpu(Wire);
long timer = 0;
float targetAngle = 0;
float errorAccumulated = 0;
float lastError = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize Motor Pins
  pinMode(MOT_L_PWM, OUTPUT);
  pinMode(MOT_L_IN1, OUTPUT);
  pinMode(MOT_L_IN2, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT);
  pinMode(MOT_R_IN1, OUTPUT);
  pinMode(MOT_R_IN2, OUTPUT);

  // Initialize MPU-6050
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // Stop if IMU not found

  Serial.println(F("Calibrating IMU... Keep robot still."));
  delay(1000);
  mpu.calcOffsets(true, true); // Calibrate gyro and accel
  Serial.println(F("Calibration Done!"));
}

/**
 * Basic Motor Drive Function
 * @param speedL: -255 to 255
 * @param speedR: -255 to 255
 */
void setMotors(int speedL, int speedR) {
  // Left Motor
  if (speedL >= 0) {
    digitalWrite(MOT_L_IN1, HIGH);
    digitalWrite(MOT_L_IN2, LOW);
  } else {
    digitalWrite(MOT_L_IN1, LOW);
    digitalWrite(MOT_L_IN2, HIGH);
  }
  analogWrite(MOT_L_PWM, abs(speedL));

  // Right Motor
  if (speedR >= 0) {
    digitalWrite(MOT_R_IN1, HIGH);
    digitalWrite(MOT_R_IN2, LOW);
  } else {
    digitalWrite(MOT_R_IN1, LOW);
    digitalWrite(MOT_R_IN2, HIGH);
  }
  analogWrite(MOT_R_PWM, abs(speedR));
}

/**
 * PID logic to keep the robot driving straight
 * @param baseSpeed: The intended forward speed (0-255)
 */
void driveStraight(int baseSpeed) {
  mpu.update();
  
  float currentAngle = mpu.getAngleZ(); // Yaw
  float error = targetAngle - currentAngle;
  
  // PID Calculation
  errorAccumulated += error;
  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * errorAccumulated) + (Kd * derivative);
  lastError = error;

  int leftOutput = baseSpeed - correction;
  int rightOutput = baseSpeed + correction;

  // Constrain outputs to valid PWM range
  leftOutput = constrain(leftOutput, -255, 255);
  rightOutput = constrain(rightOutput, -255, 255);

  setMotors(leftOutput, rightOutput);
}

/**
 * Turn the robot to a relative angle
 * @param relativeAngle: e.g., 90 for right turn, -90 for left
 */
void turnToAngle(float relativeAngle) {
  targetAngle += relativeAngle;
  bool turning = true;
  
  while (turning) {
    mpu.update();
    float currentAngle = mpu.getAngleZ();
    float error = targetAngle - currentAngle;

    if (abs(error) < 1.0) { // Threshold for finishing turn
      setMotors(0, 0);
      turning = false;
      errorAccumulated = 0; // Reset PID for next straight segment
      break;
    }

    // Proportional turn speed based on remaining angle
    int turnSpeed = constrain(Kp * error * 2, -150, 150);
    setMotors(-turnSpeed, turnSpeed);
  }
}

void loop() {
  // Example Micromouse logic:
  // Drive forward for 2 seconds keeping straight
  long startTime = millis();
  while (millis() - startTime < 2000) {
    driveStraight(150);
  }

  // Stop and turn 90 degrees right
  setMotors(0, 0);
  delay(500);
  turnToAngle(90.0);
  delay(500);
}
