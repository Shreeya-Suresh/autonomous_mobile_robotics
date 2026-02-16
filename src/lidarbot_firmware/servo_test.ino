/*
  ESP32-S3 Control for REV Smart Robot Servo V2
  ------------------------------------------------
  - Supports 500µs – 2500µs pulse range
  - 50Hz frequency
  - 16-bit PWM resolution
  - Angular Mode (0–270 degrees default)

  IMPORTANT:
  Servo must be powered from external 6V supply.
  ESP32 GND and servo GND must be connected.
*/

#include <Arduino.h>

// ==========================
// ===== USER SETTINGS ======
// ==========================

#define SERVO_PIN      5      // GPIO connected to servo signal
#define PWM_CHANNEL    0      // LEDC channel (0–7 safe)
#define PWM_FREQ       50     // 50Hz for servo
#define PWM_RESOLUTION 16     // 16-bit resolution

// Servo pulse limits (microseconds)
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500

// Servo mechanical range
#define SERVO_MAX_DEG  270    // Default 270°


// ==========================
// ===== GLOBAL VALUES ======
// ==========================

uint32_t maxDuty;   // Maximum duty value based on resolution


// ==========================
// ===== SET SERVO ANGLE ====
// ==========================

void setServoAngle(float angle)
{
  // Clamp angle safely
  if (angle < 0) angle = 0;
  if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;

  // Convert angle to pulse width
  float pulseWidth = SERVO_MIN_US +
                     (angle / SERVO_MAX_DEG) *
                     (SERVO_MAX_US - SERVO_MIN_US);

  // Convert microseconds to duty cycle
  float period_us = 1000000.0 / PWM_FREQ;   // 20,000us
  uint32_t duty = (pulseWidth / period_us) * maxDuty;

  ledcWrite(PWM_CHANNEL, duty);
}


// ==========================
// ===== SET RAW PULSE ======
// ==========================

void setServoPulse(uint16_t pulse_us)
{
  if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
  if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

  float period_us = 1000000.0 / PWM_FREQ;
  uint32_t duty = (pulse_us / period_us) * maxDuty;

  ledcWrite(PWM_CHANNEL, duty);
}


// ==========================
// ========= SETUP ==========
// ==========================

void setup()
{
  Serial.begin(115200);

  // Configure LEDC PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);

  maxDuty = (1 << PWM_RESOLUTION) - 1;

  Serial.println("REV Smart Servo V2 Ready");

  // Move to center at startup
  setServoAngle(135);   // Middle of 270°
}


// ==========================
// =========== LOOP =========
// ==========================

void loop()
{
  // Sweep example
  for (int angle = 0; angle <= 270; angle += 10)
  {
    setServoAngle(angle);
    delay(200);
  }

  for (int angle = 270; angle >= 0; angle -= 10)
  {
    setServoAngle(angle);
    delay(200);
  }
}
