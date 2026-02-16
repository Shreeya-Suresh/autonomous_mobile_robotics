#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// =====================
// CONFIG
// =====================

#define BAUD_RATE        115200
#define LOOP_TIME_MS     10
#define MAX_WHEEL_RAD_S  18.0f
#define SERIAL_TIMEOUT   500

// =====================
// PIN DEFINITIONS
// =====================

// I2C
#define I2C_SDA 8
#define I2C_SCL 9

// Motors
#define L_PWM 4
#define L_DIR 5
#define R_PWM 6
#define R_DIR 7

// Encoders
#define ENC_L_A 16
#define ENC_L_B 17
#define ENC_R_A 18
#define ENC_R_B 13

// =====================
// GRIPPER SERVO
// =====================

#define GRIP_SERVO_PIN       10
#define GRIP_SERVO_CHANNEL   3
#define GRIP_SERVO_FREQ      50
#define GRIP_SERVO_RES       16

#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500
#define SERVO_MAX_DEG  270

#define GRIP_OPEN_ANGLE   40
#define GRIP_CLOSE_ANGLE  180

uint32_t gripMaxDuty;

// =====================
// GLOBALS
// =====================

MPU6050 mpu;

volatile long left_count = 0;
volatile long right_count = 0;

unsigned long lastLoop = 0;
unsigned long lastCommandTime = 0;

bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float yaw_offset = 0;
bool yaw_initialized = false;

#define SERIAL_BUFFER 64
char inputBuffer[SERIAL_BUFFER];
uint8_t inputIndex = 0;

#define PWM_FREQ 20000
#define PWM_RES  8

// =====================
// MOTOR CONTROL
// =====================

void setMotor(int dirPin, int pwmPin, int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    ledcWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW);
    ledcWrite(pwmPin, -speed);
  }
}

// =====================
// GRIPPER CONTROL
// =====================

void setGripperAngle(float angle)
{
  if (angle < 0) angle = 0;
  if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;

  float pulseWidth = SERVO_MIN_US +
                     (angle / SERVO_MAX_DEG) *
                     (SERVO_MAX_US - SERVO_MIN_US);

  float period_us = 1000000.0 / GRIP_SERVO_FREQ;

  uint32_t duty = (pulseWidth / period_us) * gripMaxDuty;

  ledcWrite(GRIP_SERVO_CHANNEL, duty);
}

void openGripper()
{
  setGripperAngle(GRIP_OPEN_ANGLE);
}

void closeGripper()
{
  setGripperAngle(GRIP_CLOSE_ANGLE);
}

// =====================
// ENCODER ISR
// =====================

void IRAM_ATTR leftISR() {
  if (digitalRead(ENC_L_B))
    left_count++;
  else
    left_count--;
}

void IRAM_ATTR rightISR() {
  if (digitalRead(ENC_R_B))
    right_count++;
  else
    right_count--;
}

// =====================
// SERIAL COMMAND
// =====================

void processCommand(char* cmd)
{
  // Velocity command
  if (cmd[0] == 'V' && cmd[1] == ',') {

    float wl = 0.0f, wr = 0.0f;
    sscanf(cmd, "V,%f,%f", &wl, &wr);

    float norm_l = wl / MAX_WHEEL_RAD_S;
    float norm_r = wr / MAX_WHEEL_RAD_S;

    norm_l = constrain(norm_l, -1.0f, 1.0f);
    norm_r = constrain(norm_r, -1.0f, 1.0f);

    int pwm_l = (int)(norm_l * 255.0f);
    int pwm_r = (int)(norm_r * 255.0f);

    setMotor(L_DIR, L_PWM, pwm_l);
    setMotor(R_DIR, R_PWM, pwm_r);

    lastCommandTime = millis();
  }

  // Gripper command
  if (cmd[0] == 'G' && cmd[1] == ',') {

    if (cmd[2] == 'O') {
      openGripper();
    }
    else if (cmd[2] == 'C') {
      closeGripper();
    }
  }
}

void readSerialNonBlocking()
{
  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';
      processCommand(inputBuffer);
      inputIndex = 0;
    }
    else if (inputIndex < SERIAL_BUFFER - 1) {
      inputBuffer[inputIndex++] = c;
    }
  }
}

// =====================
// SETUP
// =====================

void setup()
{
  Serial.begin(BAUD_RATE);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();

  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  }

  // Motors
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  ledcAttach(L_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_PWM, PWM_FREQ, PWM_RES);

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);

  // Gripper servo setup
  ledcSetup(GRIP_SERVO_CHANNEL, GRIP_SERVO_FREQ, GRIP_SERVO_RES);
  ledcAttachPin(GRIP_SERVO_PIN, GRIP_SERVO_CHANNEL);

  gripMaxDuty = (1 << GRIP_SERVO_RES) - 1;

  openGripper();

  Serial.println("ESP32 READY");
}

// =====================
// LOOP
// =====================

void loop()
{
  readSerialNonBlocking();

  // Safety stop
  if (millis() - lastCommandTime > SERIAL_TIMEOUT) {
    setMotor(L_DIR, L_PWM, 0);
    setMotor(R_DIR, R_PWM, 0);
  }

  if (millis() - lastLoop >= LOOP_TIME_MS) {

    long l, r;
    noInterrupts();
    l = left_count;
    r = right_count;
    interrupts();

    float yaw = 0.0f;

    if (dmpReady) {

      fifoCount = mpu.getFIFOCount();

      if (fifoCount == 1024) {
        mpu.resetFIFO();
      }
      else if (fifoCount >= packetSize) {

        while (fifoCount >= packetSize) {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
        }

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        yaw = ypr[0] * 180.0f / M_PI;

        if (!yaw_initialized) {
          yaw_offset = yaw;
          yaw_initialized = true;
        }

        yaw -= yaw_offset;

        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;
      }
    }

    Serial.print("D,");
    Serial.print(l);
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.println(yaw, 2);

    lastLoop = millis();
  }
}
