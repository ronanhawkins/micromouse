#pragma once
// DRI0044 motor driver: one DIR and one PWM pin per motor.

void motorsBegin();
// Signed volts per motor, converted to duty from MOTOR_SUPPLY_VOLTS and
// clamped to +-MAX_MOTOR_VOLTS.
void motorsSetVolts(float left, float right);
void motorsOff();
