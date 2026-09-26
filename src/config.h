#pragma once
// Every pin, dimension, gain and threshold in one place.
// Values marked CALIBRATE are placeholders. Measure them with the serial CLI
// (see README "Bring-up") and edit them here.

#include <stdint.h>

// ---------------------------------------------------------------- pins ----
// Rewired 2026-09-26 (the original diagram image is out of date).
// Motor 1 / encoder 1 = left, motor 2 / encoder 2 = right. Encoder A = C1, B = C2.
constexpr int PIN_ENC_L_A = 3;
constexpr int PIN_ENC_L_B = 2;
constexpr int PIN_ENC_R_A = 4;  // GPIO4/5 are strapping pins: if the board won't
constexpr int PIN_ENC_R_B = 5;  // boot/flash with encoders attached, look here

// DFRobot DRI0044 (TB6612FNG with DIR/PWM inputs).
constexpr int PIN_MOTOR_L_DIR = 0;
constexpr int PIN_MOTOR_L_PWM = 1;
constexpr int PIN_MOTOR_R_DIR = 6;
constexpr int PIN_MOTOR_R_PWM = 7;

constexpr int PIN_I2C_SDA = 22;
constexpr int PIN_I2C_SCL = 21;
constexpr uint32_t I2C_HZ = 400000;

constexpr int PIN_XSHUT_L = 18;
constexpr int PIN_XSHUT_F = 19;
constexpr int PIN_XSHUT_R = 20;
constexpr uint8_t TOF_ADDR_L = 0x30;
constexpr uint8_t TOF_ADDR_F = 0x31;
constexpr uint8_t TOF_ADDR_R = 0x32;
constexpr uint8_t IMU_ADDR = 0x68;  // MPU-6050 on the SEN0142

constexpr int PIN_STATUS_LED = 8;  // onboard WS2812 (strapping pin, output only after boot)
constexpr int PIN_BUTTON = 9;   // onboard BOOT button, active low

// ------------------------------------------------------------- polarity ----
// Set during bring-up: the `enc` command must count UP when a wheel is
// turned forwards by hand; `motor` must drive each wheel forwards.
constexpr bool INVERT_ENC_L = false;  // motor_test: enc1 counts up forwards
constexpr bool INVERT_ENC_R = true;   // motor_test: enc2 counted down forwards
constexpr bool INVERT_MOTOR_L = true;   // confirmed with tools/motor_test
constexpr bool INVERT_MOTOR_R = false;  // not yet confirmed (channel 2 fault)
constexpr bool INVERT_GYRO = true;     // confirmed: left turn read -90 before inverting

// ------------------------------------------------------------ geometry ----
constexpr float WHEEL_DIAMETER_MM = 44.0f;
// Measured ~800 with `enc`. CALIBRATE precisely: turn a wheel exactly 10
// revolutions and divide by 10, then fine-tune with `fwd 900`.
constexpr float COUNTS_PER_WHEEL_REV = 800.0f;
constexpr float MM_PER_COUNT = 3.14159265f * WHEEL_DIAMETER_MM / COUNTS_PER_WHEEL_REV;
// CALIBRATE: distance between the wheel contact centres.
constexpr float WHEELBASE_MM = 75.0f;
// CALIBRATE: from the back of the mouse to the axle (for starting against the back wall).
constexpr float BACK_TO_AXLE_MM = 35.0f;

constexpr float CELL_MM = 180.0f;
constexpr float HALF_WALL_MM = 6.0f;

// ------------------------------------------------------------ control ----
constexpr int CONTROL_PERIOD_MS = 2;  // 500 Hz (FreeRTOS tick is 1 ms)
constexpr float CONTROL_DT = CONTROL_PERIOD_MS / 1000.0f;

// Motor supply seen by the driver's VM pin. If VM comes straight from the
// 2S pack (7.4-8.4 V) the 6 V N20s are over-volted: keep MAX_MOTOR_VOLTS <= 6.
constexpr float MOTOR_SUPPLY_VOLTS = 7.4f;  // CALIBRATE: 5.0 if VM is on the buck
constexpr float MAX_MOTOR_VOLTS = 6.0f;

// Feedforward: volts needed per wheel speed, plus friction offset.
// CALIBRATE with `ff`: the mouse should roughly track speed with gains at 0.
constexpr float FF_VOLTS_PER_MMPS = 0.004f;
constexpr float FF_VOLTS_PER_MMPS2 = 0.0005f;
constexpr float FF_BIAS_VOLTS = 0.4f;

// PD on accumulated position error (mm) and angle error (deg). Start low.
constexpr float FWD_KP = 0.02f;   // volts per mm
constexpr float FWD_KD = 0.0f;    // volts per mm/s
constexpr float ROT_KP = 0.03f;   // volts per deg
constexpr float ROT_KD = 0.0f;    // volts per deg/s

// Side-wall steering: degrees of correction per mm of off-centre error, per tick.
constexpr float STEER_KP = 0.002f;
constexpr float STEER_MAX_DEG = 0.1f;

// If the position error grows beyond this the mouse is stuck: abort.
constexpr float FWD_ERROR_ABORT_MM = 40.0f;

// Speeds (mm/s, mm/s^2, deg/s, deg/s^2). Search is gentle; run is faster.
struct SpeedSet {
  float straight;
  float accel;
  float turnRate;
  float turnAccel;
};
constexpr SpeedSet SEARCH_SPEEDS{300.0f, 1500.0f, 360.0f, 3000.0f};
constexpr SpeedSet RUN_SPEEDS{800.0f, 3000.0f, 540.0f, 4500.0f};

// ------------------------------------------------------------- sensors ----
// Distances are raw sensor readings in mm (not mouse-centre distances).
// CALIBRATE with `tof` with the mouse centred in a cell between two walls:
constexpr float SIDE_NOMINAL_MM = 60.0f;   // reading to a side wall when centred
constexpr float SIDE_WALL_MM = 120.0f;     // below this = side wall present
constexpr float FRONT_WALL_MM = 150.0f;    // below this = wall ahead in this cell
// Reading to the wall ahead when the mouse is exactly at the cell centre.
constexpr float FRONT_CENTRED_MM = 55.0f;  // CALIBRATE
constexpr float FRONT_ALIGN_MAX_MM = 30.0f;  // ignore bigger corrections as glitches

constexpr uint32_t TOF_BUDGET_US = 20000;  // ~50 Hz per sensor
constexpr int WALL_SAMPLES = 3;            // readings averaged when stopped
