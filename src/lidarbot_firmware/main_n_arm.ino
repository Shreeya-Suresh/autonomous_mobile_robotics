#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// =====================================================
// ===================== CONFIG ========================
// =====================================================

#define BAUD_RATE        115200
#define LOOP_TIME_MS     10
#define MAX_WHEEL_RAD_S  18.0f
#define SERIAL_TIMEOUT   500

// =====================================================
// ================= PIN DEFINITIONS ===================
// =====================================================

// I2C
#define I2C_SDA 8
#define I2C_SCL 9

// Drive Motors
#define L_PWM 4
#define L_DIR 5
#define R_PWM 6
#define R_DIR 7

// Drive Encoders
#define ENC_L_A 16
#define ENC_L_B 17
#define ENC_R_A 18
#define ENC_R_B 13

// =====================================================
// ================= GRIPPER SERVO =====================
// =====================================================

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

// =====================================================
// ============ POSITION CONTROL MOTOR =================
// =====================================================

#define POS_PWM       11
#define POS_DIR       12
#define POS_ENC_A     14
#define POS_ENC_B     15

#define POS_PWM_FREQ     20000
#define POS_PWM_RES      8
#define POS_PWM_CHANNEL  4

#define KP 0.5f
#define MAX_PWM 255
#define POS_CONTROL_INTERVAL 5  // 200 Hz

volatile long pos_encoder_count = 0;
long pos_target = 0;
unsigned long lastPosControl = 0;

// =====================================================
// ===================== GLOBALS =======================
// =====================================================

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

// =====================================================
// ================= MOTOR FUNCTIONS ===================
// =====================================================

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

// Position motor drive
void setPosMotor(int speed)
{
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed >= 0) {
    digitalWrite(POS_DIR, HIGH);
    ledcWrite(POS_PWM_CHANNEL, speed);
  } else {
    digitalWrite(POS_DIR, LOW);
    ledcWrite(POS_PWM_CHANNEL, -speed);
  }
}

// =====================================================
// ================= GRIPPER ===========================
// =====================================================

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

void openGripper() { setGripperAngle(GRIP_OPEN_ANGLE); }
void closeGripper() { setGripperAngle(GRIP_CLOSE_ANGLE); }

// =====================================================
// ================= ENCODER ISRs ======================
// =====================================================

void IRAM_ATTR leftISR() {
  if (digitalRead(ENC_L_B)) left_count++;
  else left_count--;
}

void IRAM_ATTR rightISR() {
  if (digitalRead(ENC_R_B)) right_count++;
  else right_count--;
}

void IRAM_ATTR posEncoderISR() {
  if (digitalRead(POS_ENC_B)) pos_encoder_count++;
  else pos_encoder_count--;
}

// =====================================================
// ================= SERIAL PARSER =====================
// =====================================================

void processCommand(char* cmd)
{
  // Drive velocity
  if (cmd[0] == 'V' && cmd[1] == ',') {

    float wl = 0.0f, wr = 0.0f;
    sscanf(cmd, "V,%f,%f", &wl, &wr);

    float norm_l = constrain(wl / MAX_WHEEL_RAD_S, -1.0f, 1.0f);
    float norm_r = constrain(wr / MAX_WHEEL_RAD_S, -1.0f, 1.0f);

    setMotor(L_DIR, L_PWM, norm_l * 255);
    setMotor(R_DIR, R_PWM, norm_r * 255);

    lastCommandTime = millis();
  }

  // Gripper
  if (cmd[0] == 'G' && cmd[1] == ',') {
    if (cmd[2] == 'O') openGripper();
    if (cmd[2] == 'C') closeGripper();
  }

  // Position motor
  if (cmd[0] == 'P' && cmd[1] == ',') {
    pos_target = atol(&cmd[2]);
    Serial.print("New Position Target: ");
    Serial.println(pos_target);
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

// =====================================================
// ===================== SETUP =========================
// =====================================================

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

  // Drive Motors
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  ledcAttach(L_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_PWM, PWM_FREQ, PWM_RES);

  // Drive Encoders
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);

  // Gripper
  ledcSetup(GRIP_SERVO_CHANNEL, GRIP_SERVO_FREQ, GRIP_SERVO_RES);
  ledcAttachPin(GRIP_SERVO_PIN, GRIP_SERVO_CHANNEL);
  gripMaxDuty = (1 << GRIP_SERVO_RES) - 1;
  openGripper();

  // Position Motor
  pinMode(POS_DIR, OUTPUT);
  ledcSetup(POS_PWM_CHANNEL, POS_PWM_FREQ, POS_PWM_RES);
  ledcAttachPin(POS_PWM, POS_PWM_CHANNEL);

  pinMode(POS_ENC_A, INPUT_PULLUP);
  pinMode(POS_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(POS_ENC_A), posEncoderISR, RISING);

  Serial.println("SYSTEM READY");
}

// =====================================================
// ===================== LOOP ==========================
// =====================================================

void loop()
{
  readSerialNonBlocking();

  // Safety stop drive motors
  if (millis() - lastCommandTime > SERIAL_TIMEOUT) {
    setMotor(L_DIR, L_PWM, 0);
    setMotor(R_DIR, R_PWM, 0);
  }

  // ================= POSITION CONTROL =================
  if (millis() - lastPosControl >= POS_CONTROL_INTERVAL)
  {
    long current;

    noInterrupts();
    current = pos_encoder_count;
    interrupts();

    long error = pos_target - current;

    int pwm = KP * error;

    if (abs(error) < 5) pwm = 0;

    setPosMotor(pwm);

    lastPosControl = millis();
  }

  // ================= TELEMETRY =================
  if (millis() - lastLoop >= LOOP_TIME_MS)
  {
    long l, r;
    noInterrupts();
    l = left_count;
    r = right_count;
    interrupts();

    Serial.print("D,");
    Serial.print(l);
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.println(pos_encoder_count);

    lastLoop = millis();
  }
}
