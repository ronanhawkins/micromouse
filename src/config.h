#pragma once
// Every pin, dimension, gain and threshold in one place.
// Values marked CALIBRATE are placeholders. Measure them with the serial CLI
// (see README "Bring-up") and edit them here.

#include <stdint.h>

// ---------------------------------------------------------------- pins ----
// Rewired 2026-09-26 (the original diagram image is out of date).
// Encoder 1 = left, encoder 2 = right. Encoder A = C1, B = C2.
constexpr int PIN_ENC_L_A = 3;
constexpr int PIN_ENC_L_B = 2;
constexpr int PIN_ENC_R_A = 4;  // GPIO4/5 are strapping pins: if the board won't
constexpr int PIN_ENC_R_B = 5;  // boot/flash with encoders attached, look here

// DFRobot DRI0044 (TB6612FNG with DIR/PWM inputs).
// Driver channel 2 drives the LEFT wheel, channel 1 the RIGHT.
constexpr int PIN_MOTOR_L_DIR = 6;
constexpr int PIN_MOTOR_L_PWM = 7;
constexpr int PIN_MOTOR_R_DIR = 0;
constexpr int PIN_MOTOR_R_PWM = 1;

constexpr int PIN_I2C_SDA = 22;
constexpr int PIN_I2C_SCL = 21;
constexpr uint32_t I2C_HZ = 100000;  // 400 kHz gave read errors from motor noise

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
// Defaults only: run the `check` command, which measures the real directions
// and saves them to flash (they then override these). See polarity.h.
constexpr bool INVERT_ENC_L = false;
constexpr bool INVERT_ENC_R = true;
constexpr bool INVERT_MOTOR_L = true;   // `motor 3 0` / `motor 0 3` ran backwards before
constexpr bool INVERT_MOTOR_R = false;
constexpr bool INVERT_GYRO = true;

// ------------------------------------------------------------ geometry ----
constexpr float WHEEL_DIAMETER_MM = 44.0f;
// Measured: 836 counts per wheel revolution (7-pulse encoder x4, ~29.9:1 gearbox).
// Fine-tune with `fwd 900` against a ruler if cells still come up short/long.
constexpr float COUNTS_PER_WHEEL_REV = 836.0f;
constexpr float MM_PER_COUNT = 3.14159265f * WHEEL_DIAMETER_MM / COUNTS_PER_WHEEL_REV;
// CALIBRATE: distance between the wheel contact centres.
constexpr float WHEELBASE_MM = 75.0f;
// Measured: back of the mouse to the axle (for starting against the back wall).
constexpr float BACK_TO_AXLE_MM = 36.0f;

// Overall footprint (for turn clearance).
constexpr float MOUSE_LENGTH_MM = 100.0f;
constexpr float MOUSE_WIDTH_MM = 100.0f;

constexpr float CELL_MM = 180.0f;
constexpr float PASSAGE_MM = 168.0f;

// Quarter turns pivot about the axle. The front corners sweep a radius of
// sqrt(64^2 + 50^2) = 81 mm against 84 mm to the walls (3 mm clearance), the
// back corners only 62 mm. Pivoting this far behind the cell centre balances
// the clearance front and back (about +-12 mm). An about-turn is two quarter
// turns, aligning on the wall in between when there is one (in a dead end
// that centres the mouse sideways, where a spin would have 3 mm clearance).
constexpr float TURN_PIVOT_BACK_MM = 10.0f;
constexpr float HALF_WALL_MM = 6.0f;

// ------------------------------------------------------------ control ----
constexpr int CONTROL_PERIOD_MS = 2;  // 500 Hz (FreeRTOS tick is 1 ms)
constexpr float CONTROL_DT = CONTROL_PERIOD_MS / 1000.0f;

// Motor supply seen by the driver's VM pin. If VM comes straight from the
// 2S pack (7.4-8.4 V) the 6 V N20s are over-volted: keep MAX_MOTOR_VOLTS <= 6.
constexpr float MOTOR_SUPPLY_VOLTS = 9.0f;  // measured on the driver's VM pin
constexpr float MAX_MOTOR_VOLTS = 6.0f;

// Feedforward: volts needed per wheel speed, plus friction offset.
// From `motor 3 3` (really ~3.65 V: that test assumed a 7.4 V supply):
// roughly 470-660 mm/s, the left motor being the slower one.
constexpr float FF_VOLTS_PER_MMPS = 0.0055f;
constexpr float FF_VOLTS_PER_MMPS2 = 0.0005f;
constexpr float FF_BIAS_VOLTS = 0.4f;

// PD on accumulated position error (mm) and angle error (deg).
// Too low: moves stop short. Too high: buzzing/oscillation -> raise KD or lower KP.
constexpr float FWD_KP = 0.10f;    // volts per mm
constexpr float FWD_KD = 0.002f;   // volts per mm/s
constexpr float ROT_KP = 0.20f;    // volts per deg (0.12 left turns 7-15 deg short)
constexpr float ROT_KD = 0.004f;   // volts per deg/s

// Side-wall steering: degrees of correction per mm of off-centre error, per tick.
constexpr float STEER_KP = 0.002f;
constexpr float STEER_MAX_DEG = 0.1f;

// If the position error grows beyond this the mouse is stuck: abort.
constexpr float FWD_ERROR_ABORT_MM = 40.0f;

// After a profile ends, keep controlling until the error is this small (or
// SETTLE_TIMEOUT_MS passes) so moves finish on target instead of short.
constexpr float SETTLE_FWD_MM = 2.0f;
constexpr float SETTLE_ROT_DEG = 1.5f;
constexpr int SETTLE_TIMEOUT_MS = 600;
// While settling (profile finished but not on target) add this much to get
// the wheels past static friction; otherwise small errors never move them.
constexpr float STICTION_VOLTS = 0.6f;

// Speeds (mm/s, mm/s^2, deg/s, deg/s^2). Search is gentle; run is faster.
struct SpeedSet {
  float straight;
  float accel;
  float turnRate;
  float turnAccel;
};
// Top speed is limited by MAX_MOTOR_VOLTS: at 6 V the slower (left) motor
// tops out around 850 mm/s, and the controller needs some headroom on top.
constexpr SpeedSet SEARCH_SPEEDS{550.0f, 2500.0f, 540.0f, 5000.0f};
constexpr SpeedSet RUN_SPEEDS{800.0f, 3000.0f, 600.0f, 5000.0f};

// ------------------------------------------------------------- sensors ----
// Distances are raw sensor readings in mm (not mouse-centre distances).
// CALIBRATE with `tof` with the mouse centred in a cell between two walls:
constexpr float SIDE_NOMINAL_MM = 90.0f;   // measured: L and R both read 90 when centred
// No wall = the next wall out, about 180 mm further. Halfway-ish between.
constexpr float SIDE_WALL_MM = 160.0f;     // below this = side wall present
// No wall ahead = next wall about 180 mm further (~260). Halfway-ish between.
constexpr float FRONT_WALL_MM = 170.0f;    // below this = wall ahead in this cell
// Reading to the wall ahead when the mouse is exactly at the cell centre.
constexpr float FRONT_CENTRED_MM = 85.0f;  // tuned on the maze (80 too close, 95 too far)
constexpr float FRONT_ALIGN_MAX_MM = 30.0f;  // ignore bigger corrections as glitches

constexpr uint32_t TOF_BUDGET_US = 20000;  // ~50 Hz per sensor
constexpr int WALL_SAMPLES = 3;            // readings averaged when stopped
