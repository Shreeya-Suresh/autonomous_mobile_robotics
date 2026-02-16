#include <Arduino.h>

// ==========================
// NEW MOTOR PINS
// ==========================

#define POS_PWM      11
#define POS_DIR      12

#define ENC_A        14
#define ENC_B        15

#define PWM_FREQ     20000
#define PWM_RES      8
#define PWM_CHANNEL  4   // new channel

// ==========================
// CONTROL PARAMETERS
// ==========================

#define KP  0.5f     // Tune this
#define MAX_PWM 255

volatile long encoder_count = 0;
long target_position = 0;

unsigned long lastControl = 0;
#define CONTROL_INTERVAL 5   // 5 ms (200 Hz)

// ==========================
// ENCODER ISR
// ==========================

void IRAM_ATTR encoderISR()
{
  if (digitalRead(ENC_B))
    encoder_count++;
  else
    encoder_count--;
}

// ==========================
// MOTOR FUNCTION
// ==========================

void setMotor(int speed)
{
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed >= 0)
  {
    digitalWrite(POS_DIR, HIGH);
    ledcWrite(PWM_CHANNEL, speed);
  }
  else
  {
    digitalWrite(POS_DIR, LOW);
    ledcWrite(PWM_CHANNEL, -speed);
  }
}

// ==========================
// SETUP
// ==========================

void setup()
{
  Serial.begin(115200);

  pinMode(POS_DIR, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(POS_PWM, PWM_CHANNEL);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.println("Position Control Test Ready");
  Serial.println("Send: P,<counts>");
}

// ==========================
// SERIAL COMMAND
// ==========================

void readSerial()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.startsWith("P,"))
    {
      target_position = cmd.substring(2).toInt();
      Serial.print("New Target: ");
      Serial.println(target_position);
    }
  }
}

// ==========================
// LOOP
// ==========================

void loop()
{
  readSerial();

  if (millis() - lastControl >= CONTROL_INTERVAL)
  {
    long current;

    noInterrupts();
    current = encoder_count;
    interrupts();

    long error = target_position - current;

    int pwm = (int)(KP * error);

    setMotor(pwm);

    Serial.print("Target: ");
    Serial.print(target_position);
    Serial.print("  Pos: ");
    Serial.print(current);
    Serial.print("  PWM: ");
    Serial.println(pwm);

    lastControl = millis();
  }
}
