/*
  ESP32-S3 Gripper Control
  REV Smart Robot Servo V2

  Command via Serial:
    'o' → Open gripper
    'c' → Close gripper

  Servo powered externally at 6V.
*/

#include <Arduino.h>

// ==========================
// USER CONFIGURATION
// ==========================

#define SERVO_PIN      5
#define PWM_CHANNEL    0
#define PWM_FREQ       50
#define PWM_RESOLUTION 16

#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500
#define SERVO_MAX_DEG  270

// ----- GRIPPER CALIBRATION -----

#define GRIP_OPEN_ANGLE   40    // << Tune this
#define GRIP_CLOSE_ANGLE  180   // << Tune this

// ==========================

uint32_t maxDuty;

// Convert angle → PWM
void setServoAngle(float angle)
{
  if (angle < 0) angle = 0;
  if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;

  float pulseWidth = SERVO_MIN_US +
                     (angle / SERVO_MAX_DEG) *
                     (SERVO_MAX_US - SERVO_MIN_US);

  float period_us = 1000000.0 / PWM_FREQ;
  uint32_t duty = (pulseWidth / period_us) * maxDuty;

  ledcWrite(PWM_CHANNEL, duty);
}

// High level commands
void openGripper()
{
  Serial.println("Opening Gripper");
  setServoAngle(GRIP_OPEN_ANGLE);
}

void closeGripper()
{
  Serial.println("Closing Gripper");
  setServoAngle(GRIP_CLOSE_ANGLE);
}

// ==========================

void setup()
{
  Serial.begin(115200);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);

  maxDuty = (1 << PWM_RESOLUTION) - 1;

  Serial.println("Gripper Ready");
  Serial.println("Send 'o' to open, 'c' to close");

  openGripper();  // Start open
}

void loop()
{
  if (Serial.available())
  {
    char cmd = Serial.read();

    if (cmd == 'o')
    {
      openGripper();
    }
    else if (cmd == 'c')
    {
      closeGripper();
    }
  }
}
