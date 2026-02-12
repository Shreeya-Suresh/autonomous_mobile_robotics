#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define BAUD_RATE      115200
#define LOOP_TIME_MS   10   // 100Hz

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
// GLOBALS
// =====================

MPU6050 mpu;

volatile long left_count = 0;
volatile long right_count = 0;

unsigned long lastLoop = 0;

// DMP
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float yaw_offset = 0;
bool yaw_initialized = false;

// =====================
// SERIAL BUFFER
// =====================

#define SERIAL_BUFFER 64
char inputBuffer[SERIAL_BUFFER];
uint8_t inputIndex = 0;

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
// MOTOR CONTROL
// =====================

#define PWM_FREQ 20000
#define PWM_RES  8

void setMotor(int dirPin, int pwmPin, int speed) {

  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    digitalWrite(dirPin, HIGH);
    ledcWrite(pwmPin, speed);   // S3 uses pin-based ledcWrite
  } else {
    digitalWrite(dirPin, LOW);
    ledcWrite(pwmPin, -speed);
  }
}

// =====================
// PROCESS SERIAL COMMAND
// =====================

void processCommand(char* cmd) {

  Serial.print("RECEIVED: ");
  Serial.println(cmd);

  if (cmd[0] == 'V' && cmd[1] == ',') {

    int l = 0, r = 0;
    sscanf(cmd, "V,%d,%d", &l, &r);

    Serial.print("Parsed L=");
    Serial.print(l);
    Serial.print(" R=");
    Serial.println(r);

    setMotor(L_DIR, L_PWM, l);
    setMotor(R_DIR, R_PWM, r);
  }
}

void readSerialNonBlocking() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';
      processCommand(inputBuffer);
      inputIndex = 0;
    }
    else {
      if (inputIndex < SERIAL_BUFFER - 1) {
        inputBuffer[inputIndex++] = c;
      }
    }
  }
}

// =====================
// SETUP
// =====================

void setup() {

  Serial.begin(BAUD_RATE);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  }

  // Motors
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  // ✅ ESP32-S3 CORRECT PWM INIT
  ledcAttach(L_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_PWM, PWM_FREQ, PWM_RES);

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);

  Serial.println("ESP32 READY");
}

// =====================
// LOOP
// =====================

void loop() {

  readSerialNonBlocking();

  if (millis() - lastLoop >= LOOP_TIME_MS) {

    long l, r;
    noInterrupts();
    l = left_count;
    r = right_count;
    interrupts();

    float yaw = 0;

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
    Serial.print(l); Serial.print(",");
    Serial.print(r); Serial.print(",");
    Serial.println(yaw, 2);

    lastLoop = millis();
  }
}
