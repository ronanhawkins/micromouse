// Hardware test: DRI0044 motors, N20 encoders, MPU-6050 gyro, 3x VL53L0X.
//
// Serial commands (115200 baud, single keys):
//   1 / 2   run motor 1 / motor 2 forwards      b   both forwards
//   r       both in reverse                     s   stop
//   + / -   duty up / down by 10%               z   zero counts and gyro angle
//   i       I2C scan                            p   pause / resume the readout
// Motors stop by themselves after 3 s.
// One status line prints 5x a second: encoder counts, ToF distances (mm,
// "----" = nothing in range) and gyro rate / integrated angle.

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

constexpr int DIR1 = 0, PWM1 = 1;  // motor 1
constexpr int DIR2 = 6, PWM2 = 7;  // motor 2
constexpr int ENC1_A = 3, ENC1_B = 2;  // encoder 1: C1, C2
constexpr int ENC2_A = 4, ENC2_B = 5;  // encoder 2: C1, C2

constexpr int SDA_PIN = 22, SCL_PIN = 21;
constexpr int XSHUT[3] = {18, 19, 20};  // left, front, right
constexpr uint8_t TOF_ADDR[3] = {0x30, 0x31, 0x32};
constexpr uint8_t IMU_ADDR = 0x68;

// Flip a motor's direction if "forwards" turns its wheel backwards.
constexpr bool INVERT1 = true;
constexpr bool INVERT2 = false;

constexpr uint32_t PWM_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;
constexpr unsigned long RUN_MS = 3000;

volatile int32_t count1 = 0, count2 = 0;
volatile uint8_t state1 = 0, state2 = 0;

// x4 quadrature: table indexed by (previous AB << 2) | new AB.
const int8_t QDEC[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR enc1Isr() {
  uint8_t s = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  count1 += QDEC[(state1 << 2) | s];
  state1 = s;
}

void IRAM_ATTR enc2Isr() {
  uint8_t s = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);
  count2 += QDEC[(state2 << 2) | s];
  state2 = s;
}

float duty = 0.5f;
unsigned long stopAt = 0;
bool paused = false;

// ---------------------------------------------------------------- ToF ----

VL53L0X tof[3];
bool tofOk[3];
int tofMm[3] = {-1, -1, -1};  // -1 = no reading yet, 0 = nothing in range

void tofBegin() {
  // All three power up at 0x29: hold them in reset, then wake and
  // readdress one at a time.
  for (int i = 0; i < 3; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  delay(10);
  for (int i = 0; i < 3; i++) {
    digitalWrite(XSHUT[i], HIGH);
    delay(5);
    tof[i].setBus(&Wire);
    tof[i].setTimeout(100);
    tofOk[i] = tof[i].init();
    if (tofOk[i]) {
      tof[i].setAddress(TOF_ADDR[i]);
      tof[i].setMeasurementTimingBudget(33000);
      tof[i].startContinuous(0);
    }
  }
  Serial.printf("ToF left %s, front %s, right %s\n", tofOk[0] ? "ok" : "FAIL", tofOk[1] ? "ok" : "FAIL",
                tofOk[2] ? "ok" : "FAIL");
}

// Non-blocking: only read a sensor when it has a new measurement.
void tofPoll() {
  for (int i = 0; i < 3; i++) {
    if (!tofOk[i] || !(tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07)) continue;
    uint16_t mm = tof[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    tof[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    tofMm[i] = mm >= 8000 ? 0 : mm;
  }
}

// --------------------------------------------------------------- gyro ----

bool imuOk = false;
float gyroBias = 0, gyroRate = 0, gyroAngle = 0;
unsigned long gyroLastUs = 0;

void imuWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(v);
  Wire.endTransmission();
}

int16_t imuRawZ() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(IMU_ADDR, uint8_t(2)) != 2) return 0;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return int16_t((hi << 8) | lo);
}

void imuBegin() {
  Wire.beginTransmission(IMU_ADDR);
  imuOk = Wire.endTransmission() == 0;
  if (imuOk) {
    imuWrite(0x6B, 0x01);  // wake, gyro clock
    delay(10);
    imuWrite(0x1A, 0x02);  // DLPF ~94 Hz
    imuWrite(0x1B, 0x10);  // +-1000 deg/s = 32.8 LSB per deg/s
    delay(50);
    Serial.println("gyro: calibrating, keep still...");
    float sum = 0;
    for (int i = 0; i < 500; i++) {
      sum += imuRawZ();
      delay(2);
    }
    gyroBias = sum / 500;
  }
  Serial.printf("gyro %s\n", imuOk ? "ok" : "FAIL (not found at 0x68)");
  gyroLastUs = micros();
}

// Positive = turning left (anticlockwise seen from above), if mounted flat.
void imuPoll() {
  if (!imuOk) return;
  unsigned long now = micros();
  gyroRate = (imuRawZ() - gyroBias) / 32.8f;
  gyroAngle += gyroRate * (now - gyroLastUs) * 1e-6f;
  gyroLastUs = now;
}

void i2cScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X%s\n", a,
                    a == IMU_ADDR ? "  gyro" : a == 0x29 ? "  ToF at default address (XSHUT?)"
                    : a == 0x30 ? "  ToF left" : a == 0x31 ? "  ToF front" : a == 0x32 ? "  ToF right" : "");
      found++;
    }
  }
  if (!found) Serial.println("  nothing: check SDA (22) / SCL (21) and sensor power");
}

void printTof(int mm) {
  if (mm < 0) Serial.print("  ?? ");
  else if (mm == 0) Serial.print(" ----");
  else Serial.printf("%5d", mm);
}

// dir: +1 forwards, -1 reverse, 0 stop.
void drive(int dirPin, int pwmPin, bool invert, int dir) {
  digitalWrite(dirPin, (dir > 0) != invert ? HIGH : LOW);
  ledcWrite(pwmPin, dir == 0 ? 0 : uint32_t(duty * ((1 << PWM_BITS) - 1)));
}

void run(int m1, int m2) {
  drive(DIR1, PWM1, INVERT1, m1);
  drive(DIR2, PWM2, INVERT2, m2);
  stopAt = (m1 || m2) ? millis() + RUN_MS : 0;
  Serial.printf("motor1 %+d  motor2 %+d  duty %.0f%%\n", m1, m2, duty * 100);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  ledcAttach(PWM1, PWM_HZ, PWM_BITS);
  ledcAttach(PWM2, PWM_HZ, PWM_BITS);
  run(0, 0);

  for (int p : {ENC1_A, ENC1_B, ENC2_A, ENC2_B}) pinMode(p, INPUT_PULLUP);
  state1 = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  state2 = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);
  attachInterrupt(ENC1_A, enc1Isr, CHANGE);
  attachInterrupt(ENC1_B, enc1Isr, CHANGE);
  attachInterrupt(ENC2_A, enc2Isr, CHANGE);
  attachInterrupt(ENC2_B, enc2Isr, CHANGE);

  Serial.println("\nhardware test");
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.setTimeOut(10);
  i2cScan();
  tofBegin();
  imuBegin();
  Serial.println("keys: 1 2 b r s + - z i p");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': run(1, 0); break;
      case '2': run(0, 1); break;
      case 'b': run(1, 1); break;
      case 'r': run(-1, -1); break;
      case 's': run(0, 0); break;
      case '+': duty = min(duty + 0.1f, 1.0f); Serial.printf("duty %.0f%%\n", duty * 100); break;
      case '-': duty = max(duty - 0.1f, 0.1f); Serial.printf("duty %.0f%%\n", duty * 100); break;
      case 'z': count1 = 0; count2 = 0; gyroAngle = 0; break;
      case 'i': i2cScan(); break;
      case 'p': paused = !paused; break;
    }
  }
  if (stopAt && millis() > stopAt) run(0, 0);

  static unsigned long lastSensor = 0;
  if (millis() - lastSensor >= 2) {
    lastSensor = millis();
    imuPoll();
    tofPoll();
  }

  static unsigned long lastPrint = 0;
  static int32_t last1 = 0, last2 = 0;
  if (!paused && millis() - lastPrint >= 200) {
    int32_t c1 = count1, c2 = count2;
    float dt = (millis() - lastPrint) / 1000.0f;
    lastPrint = millis();
    Serial.printf("enc1 %7ld (%6.0f/s)  enc2 %7ld (%6.0f/s)  |  ToF L", long(c1), (c1 - last1) / dt, long(c2),
                  (c2 - last2) / dt);
    printTof(tofMm[0]);
    Serial.print(" F");
    printTof(tofMm[1]);
    Serial.print(" R");
    printTof(tofMm[2]);
    Serial.printf("  |  gyro %7.1f deg/s %7.1f deg\n", gyroRate, gyroAngle);
    last1 = c1;
    last2 = c2;
  }
}
