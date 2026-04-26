// bsp.ino (AT8236 IN1/IN2 version)
#include <Arduino.h>
#include "config.h"
#include "motion.h"

static const uint8_t MOTOR_COUNT = 4;

// =========================================================
// AT8236: each motor uses two inputs (IN1/IN2), NOT DIR+PWM.
//
// Mapping rule (recommended):
//  cmd > 0 : IN1 = PWM, IN2 = LOW
//  cmd < 0 : IN1 = LOW, IN2 = PWM
//  cmd = 0 : IN1 = LOW, IN2 = LOW  (coast)
// =========================================================
//
// IMPORTANT: The following mapping assumes you kept the same
// wiring as the old code (pwmPin/dirPin), and we just reinterpret
// them as IN1/IN2. If a wheel direction is reversed, swap IN1/IN2
// for that wheel (swap the two pins in the arrays for that index).
//
static const uint32_t motorIn1Pin[MOTOR_COUNT] = {PD14, PD10, PB1, PA2}; // old PWM pins
static const uint32_t motorIn2Pin[MOTOR_COUNT] = {PD9, PA15, PB0, PA5}; // old DIR pins

static inline int16_t clampCmd16(int16_t v) {
  if (v > CMD_MAX) return CMD_MAX;
  if (v < -CMD_MAX) return -CMD_MAX;
  return v;
}

static inline uint8_t cmdToPwm(int16_t mag) {
  mag = constrain(mag, 0, CMD_MAX);
  // map 0..CMD_MAX to PWM_MIN..PWM_MAX
  return (uint8_t)map(mag, 0, CMD_MAX, PWM_MIN, PWM_MAX);
}

// Drive one motor using IN1/IN2 inputs of H-bridge (AT8236)
static inline void Motor_WriteIN12(uint8_t i, int16_t cmd) {
  cmd = clampCmd16(cmd);

  const uint32_t in1 = motorIn1Pin[i];
  const uint32_t in2 = motorIn2Pin[i];

  if (cmd == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }

  uint16_t mag = (uint16_t)abs(cmd);
  uint8_t pwm = cmdToPwm((int16_t)mag);

  if (cmd > 0) {
    analogWrite(in2, 0);
    pinMode(in2, OUTPUT);
    digitalWrite(in2, LOW);
    analogWrite(in1, pwm);
  } else {
    analogWrite(in1, 0);
    pinMode(in1, OUTPUT);
    digitalWrite(in1, LOW);
    analogWrite(in2, pwm);
  }
}
void Motor_Init(void) {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(motorIn1Pin[i], OUTPUT);
    pinMode(motorIn2Pin[i], OUTPUT);
    digitalWrite(motorIn1Pin[i], LOW);
    digitalWrite(motorIn2Pin[i], LOW);
  }

  Motors_Set(0, 0, 0, 0);
}

// 麦克纳姆轮底盘运动学解算（保持你原来的逻辑）
void Velocity_Controller(uint16_t angleDeg, uint8_t velocity, int8_t rot, bool drift) {
  float angle = (float)angleDeg + 90.0f;
  float rad = angle * PI / 180.0f;

  float v = (float)velocity / sqrtf(2.0f);

  float speedScale = (rot == 0) ? 1.0f : 0.5f;

  float vx = v * cosf(rad);
  float vy = v * sinf(rad);

  float base0 = (vy - vx) * speedScale;
  float base1 = (vy + vx) * speedScale;
  float base2 = (vy - vx) * speedScale;
  float base3 = (vy + vx) * speedScale;

  float r = (float)rot * speedScale;

  float m0, m1, m2, m3;
  if (drift) {
    m0 = base0;
    m1 = base1;
    m2 = base2 - r * 2.0f;
    m3 = base3 + r * 2.0f;
  } else {
    m0 = base0 + r;
    m1 = base1 - r;
    m2 = base2 - r;
    m3 = base3 + r;
  }

  Motors_Set((int8_t)lroundf(m0),
             (int8_t)lroundf(m1),
             (int8_t)lroundf(m2),
             (int8_t)lroundf(m3));
}

// 4轮统一输出：AT8236 IN1/IN2 控制
void Motors_Set(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
  int16_t cmd[MOTOR_COUNT] = {m0, m1, m2, m3};

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Motor_WriteIN12(i, cmd[i]);
  }
}