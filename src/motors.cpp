#include "motors.h"

#include <Arduino.h>

#include "config.h"
#include "polarity.h"

namespace {

constexpr uint32_t PWM_HZ = 20000;  // above hearing
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

void drive(int dirPin, int pwmPin, bool invert, float volts) {
  if (volts > MAX_MOTOR_VOLTS) volts = MAX_MOTOR_VOLTS;
  if (volts < -MAX_MOTOR_VOLTS) volts = -MAX_MOTOR_VOLTS;
  bool forward = (volts >= 0) != invert;
  float duty = fabsf(volts) / MOTOR_SUPPLY_VOLTS;
  if (duty > 1.0f) duty = 1.0f;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  ledcWrite(pwmPin, uint32_t(duty * PWM_MAX));
}

}  // namespace

void motorsBegin() {
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  ledcAttach(PIN_MOTOR_L_PWM, PWM_HZ, PWM_BITS);
  ledcAttach(PIN_MOTOR_R_PWM, PWM_HZ, PWM_BITS);
  motorsOff();
}

void motorsSetVolts(float left, float right) {
  drive(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, polarity.motL, left);
  drive(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, polarity.motR, right);
}

void motorsOff() {
  ledcWrite(PIN_MOTOR_L_PWM, 0);
  ledcWrite(PIN_MOTOR_R_PWM, 0);
}
