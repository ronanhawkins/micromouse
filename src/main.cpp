// Micromouse firmware entry point: hardware bring-up, serial CLI and the
// search / speed-run sequences. The maze logic itself lives in lib/maze
// and is shared with the desktop simulator.
//
// Serial CLI at 115200 baud: type `help`.
// Without a computer: short press BOOT = search (or speed run once the
// path is proven), long press (> 1.5 s) = forget the maze.

#include <Arduino.h>
#include <explorer.h>

#include "config.h"
#include "encoders.h"
#include "hw_mouse.h"
#include "i2c_bus.h"
#include "imu.h"
#include "motion.h"
#include "motors.h"
#include "polarity.h"
#include "storage.h"
#include "tof.h"
#include "ui.h"

// Explorer + flood fill + paths live on the loop task's stack.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

namespace {

mm::Maze maze;
HwMouse hw;
bool imuOk = false, tofAllOk = false, encOk = false;

// ------------------------------------------------------------- helpers ----

bool keyPressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

// Known walls as in the maze file format, unknown walls as '.', and the
// current speed-run route as '*'.
void printMaze(const mm::Path* route) {
  using namespace mm;
  bool onRoute[CELL_COUNT] = {};
  if (route && route->valid) {
    Cell c{0, 0};
    Heading h = NORTH;
    onRoute[0] = true;
    for (int i = 0; i < route->count; i++) {
      const Move& m = route->moves[i];
      if (m.type == Move::TURN_LEFT) h = leftOf(h);
      else if (m.type == Move::TURN_RIGHT) h = rightOf(h);
      else if (m.type == Move::TURN_AROUND) h = behind(h);
      else
        for (int k = 0; k < m.cells; k++) {
          c = neighbour(c, h);
          onRoute[cellIndex(c)] = true;
        }
    }
  }
  auto wall = [&](Cell c, Heading h, const char* yes, const char* no, const char* unk) {
    return !maze.isKnown(c, h) ? unk : maze.hasWall(c, h) ? yes : no;
  };
  for (int k = 0; k <= MAZE_SIZE; k++) {
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      Cell c = k < MAZE_SIZE ? Cell{x, int8_t(MAZE_SIZE - 1 - k)} : Cell{x, 0};
      Serial.print('o');
      Serial.print(wall(c, k < MAZE_SIZE ? NORTH : SOUTH, "---", "   ", " . "));
    }
    Serial.println('o');
    if (k == MAZE_SIZE) break;
    int8_t y = int8_t(MAZE_SIZE - 1 - k);
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      Cell c{x, y};
      Serial.print(wall(c, WEST, "|", " ", "."));
      Serial.print(' ');
      Serial.print(onRoute[cellIndex(c)] ? '*' : maze.isGoal(c) ? 'G' : maze.isVisited(c) ? ' ' : '?');
      Serial.print(' ');
    }
    Serial.println(wall(Cell{MAZE_SIZE - 1, y}, EAST, "|", " ", "."));
  }
}

void printStats(const mm::Explorer& ex) {
  const mm::ExploreStats& s = ex.stats();
  Serial.printf("legs %d, cells %d, turns %d, goal %s, home %s, %s%s\n", s.legs, s.cellsMoved, s.turns,
                s.reachedGoal ? "yes" : "no", s.backAtStart ? "yes" : "no",
                s.proven ? "path proven optimal" : "path NOT proven", s.aborted ? ", ABORTED" : "");
}

// Closed-loop moves need the encoders and the gyro (without the gyro a turn
// would never see itself rotating).
bool motionReady() {
  if (encOk && imuOk) return true;
  Serial.println("can't drive: encoders or IMU failed at boot (see `i2c`)");
  uiSet(UiColour::ERROR);
  return false;
}

// Searching and running also need every wall sensor: a dead ToF reads as
// "no wall" everywhere and the mouse would drive into walls.
bool mazeReady() {
  if (!motionReady()) return false;
  if (tofAllOk) return true;
  Serial.println("can't search/run: a ToF sensor failed at boot (see `i2c`)");
  uiSet(UiColour::ERROR);
  return false;
}

// Wait for the start signal, calibrate the gyro while still, motors on.
void armMotion() {
  uiWaitStart();  // LED flashes red from the start signal until we move
  imuCalibrate();
  motionEnable(true);
}

void finish(bool ok) {
  motionEnable(false);
  uiSet(ok ? UiColour::DONE : UiColour::ERROR);
  if (!ok && motionAborted()) Serial.println("motion aborted (stuck or crashed?)");
}

// ------------------------------------------------------------ sequences ----

void doSearch() {
  if (!mazeReady()) return;
  Serial.println("search: waiting for start (wave at the front sensor or press BOOT)");
  armMotion();
  uiSet(UiColour::SEARCH);
  hw.setSpeeds(SEARCH_SPEEDS);
  mm::Explorer ex(maze, hw);
  ex.setPose(mm::Cell{0, 0}, mm::NORTH);
  bool ok = hw.leaveBackWall() && ex.fullSearch() && hw.returnToBackWall();
  finish(ok);
  printStats(ex);
  mm::Path route = ex.speedRunPath();
  printMaze(&route);
}

void doRun() {
  if (!mazeReady()) return;
  mm::Explorer ex(maze, hw);
  mm::Path route = ex.speedRunPath();
  if (!route.valid) {
    Serial.println("run: no known route to the goal yet, search first");
    uiSet(UiColour::ERROR);
    return;
  }
  char buf[512];
  mm::pathToString(route, buf, sizeof buf);
  Serial.printf("run: %s\nwaiting for start\n", buf);
  armMotion();
  uiSet(UiColour::RUN);
  hw.setSpeeds(RUN_SPEEDS);
  bool ok = hw.leaveBackWall() && hw.runPath(route);
  if (ok) {
    // Back home at search speed, mapping anything new on the way.
    hw.setSpeeds(SEARCH_SPEEDS);
    ex.setPose(route.end, route.endHeading);
    uiSet(UiColour::SEARCH);
    ok = ex.searchToStart() && ex.face(mm::NORTH) && hw.returnToBackWall();
  }
  finish(ok);
}

void autoStart() {
  mm::Explorer ex(maze, hw);
  if (ex.isProvenOptimal()) {
    Serial.println("route already proven: speed run");
    doRun();
  } else {
    Serial.println("route not proven: search");
    doSearch();
  }
}

// ------------------------------------------------------ bring-up tests ----

void streamImu() {
  Serial.println("imu: gyro Z (deg/s) and integrated angle. Turn the mouse left: angle must go UP. Any key stops.");
  imuCalibrate();
  float angle = 0;
  unsigned long last = micros(), lastPrint = 0;
  while (!keyPressed()) {
    float z = imuReadZ();
    unsigned long now = micros();
    angle += z * (now - last) * 1e-6f;
    last = now;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("rate %7.1f  angle %7.1f\n", z, angle);
    }
    delay(2);
  }
}

void streamTof() {
  Serial.println("tof: left / front / right mm (9999 = nothing). Any key stops.");
  while (!keyPressed()) {
    float l = tofMm(TOF_LEFT), f = tofMm(TOF_FRONT), r = tofMm(TOF_RIGHT);
    Serial.printf("L %5.0f%s  F %5.0f%s  R %5.0f%s\n", l, l < SIDE_WALL_MM ? "|" : " ", f,
                  f < FRONT_WALL_MM ? "-" : " ", r, r < SIDE_WALL_MM ? "|" : " ");
    delay(100);
  }
}

void streamEncoders() {
  Serial.println("enc: turn each wheel FORWARDS by hand; counts must go UP.");
  Serial.println("     Turn one wheel exactly 10 revolutions: counts / 10 = COUNTS_PER_WHEEL_REV.");
  Serial.println("     Any key stops.");
  int32_t l0 = encoderLeft(), r0 = encoderRight();
  while (!keyPressed()) {
    int32_t l = encoderLeft() - l0, r = encoderRight() - r0;
    Serial.printf("L %7ld  R %7ld   (%.1f / %.1f mm)\n", long(l), long(r), l * MM_PER_COUNT, r * MM_PER_COUNT);
    delay(100);
  }
}

void openLoopMotors(float l, float r, int ms) {
  Serial.printf("motor: %.2f V / %.2f V for %d ms. Both wheels must turn FORWARDS for + volts.\n", l, r, ms);
  motionEnable(false);
  int32_t l0 = encoderLeft(), r0 = encoderRight();
  motorsSetVolts(l, r);
  delay(ms);
  motorsOff();
  Serial.printf("encoder change: L %ld  R %ld\n", long(encoderLeft() - l0), long(encoderRight() - r0));
}

void closedLoopMove(bool isTurn, float amount) {
  if (!motionReady()) return;
  uiSet(UiColour::HANDS_OFF);
  imuCalibrate();
  uiSet(UiColour::SEARCH);
  motionEnable(true);
  bool ok = isTurn ? motionTurn(amount, SEARCH_SPEEDS.turnRate, SEARCH_SPEEDS.turnAccel)
                   : motionMove(amount, SEARCH_SPEEDS.straight, 0, SEARCH_SPEEDS.accel);
  delay(200);
  Serial.printf("%s: %s  encoders %.1f mm, gyro %.1f deg, gyro read errors %lu, I2C resets %lu\n",
                isTurn ? "turn" : "fwd", ok ? "done" : "ABORTED", motionDistanceMm(), motionAngleDeg(),
                (unsigned long)imuReadErrors(), (unsigned long)i2cRecoveries());
  motionEnable(false);
  uiSet(ok ? UiColour::IDLE : UiColour::ERROR);
}

void printWalls() {
  float d[3];
  tofAverage(WALL_SAMPLES, d);
  mm::WallReading w = hw.senseWalls();
  Serial.printf("L %.0f mm %s | F %.0f mm %s | R %.0f mm %s\n", d[0], w.left ? "WALL" : "open", d[1],
                w.front ? "WALL" : "open", d[2], w.right ? "WALL" : "open");
}

// Wait for any key, ignoring leftovers (e.g. the \n after a command's \r).
void waitKey() {
  delay(50);
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(5);
  while (Serial.available()) Serial.read();
}

// Measure every direction flag instead of guessing, then save them.
void checkDirections() {
  const Polarity old = polarity;
  motionEnable(false);

  // 1. Encoders: the user moves the mouse forwards by hand.
  Serial.println("\n1/3 ENCODERS: put the mouse on the floor. Push it FORWARDS (the way the front");
  Serial.println("    sensor faces) about 10 cm by hand, then press a key.");
  polarity.encL = polarity.encR = false;
  int32_t l0 = encoderLeft(), r0 = encoderRight();
  waitKey();
  int32_t dl = encoderLeft() - l0, dr = encoderRight() - r0;
  Serial.printf("    raw counts: L %ld  R %ld\n", long(dl), long(dr));
  if (labs(dl) < 100 || labs(dr) < 100) {
    Serial.println("    a wheel barely counted: check that encoder's wiring. Aborting, nothing saved.");
    polarity = old;
    return;
  }
  polarity.encL = dl < 0;
  polarity.encR = dr < 0;

  // 2. Motors: drive each briefly and see which way its (now correct) encoder counts.
  Serial.println("2/3 MOTORS: lift the mouse so the wheels spin freely, then press a key.");
  waitKey();
  polarity.motL = polarity.motR = false;
  // Step the voltage up until the wheel clearly turns.
  auto pulse = [](bool left) -> int32_t {
    for (float v = 1.5f; v <= MAX_MOTOR_VOLTS; v += 0.5f) {
      int32_t l0 = encoderLeft(), r0 = encoderRight();
      motorsSetVolts(left ? v : 0, left ? 0 : v);
      delay(300);
      motorsOff();
      delay(200);
      int32_t dl = encoderLeft() - l0, dr = encoderRight() - r0;
      int32_t d = left ? dl : dr, other = left ? dr : dl;
      if (labs(other) >= 50 && labs(d) < labs(other)) {
        Serial.printf("    driving the %s motor turned the %s encoder: the motor pins (or encoder pins)\n"
                      "    for left and right are swapped in config.h\n",
                      left ? "left" : "right", left ? "right" : "left");
        return 0;
      }
      if (labs(d) >= 50) {
        Serial.printf("    %s wheel turned at %.1f V: %ld counts\n", left ? "left" : "right", v, long(d));
        return d;
      }
    }
    Serial.printf("    %s wheel didn't turn even at %.1f V\n", left ? "left" : "right", MAX_MOTOR_VOLTS);
    return 0;
  };
  dl = pulse(true);
  dr = pulse(false);
  if (dl == 0 || dr == 0) {
    Serial.println("    check the motor battery is connected and switched on, and that motor's wiring.");
    Serial.println("    Aborting, nothing saved.");
    polarity = old;
    return;
  }
  polarity.motL = dl < 0;
  polarity.motR = dr < 0;

  // 3. Gyro: the user turns the mouse left by hand.
  Serial.println("3/3 GYRO: put the mouse on the floor and keep it still, then press a key.");
  waitKey();
  polarity.gyro = false;
  imuCalibrate();
  Serial.println("    now turn it LEFT (anticlockwise seen from above) about 90 deg, then press a key.");
  delay(50);
  while (Serial.available()) Serial.read();
  float angle = 0;
  unsigned long last = micros();
  while (!Serial.available()) {
    unsigned long now = micros();
    angle += imuReadZ() * (now - last) * 1e-6f;
    last = now;
    delay(2);
  }
  while (Serial.available()) Serial.read();
  Serial.printf("    measured %.0f deg\n", angle);
  if (fabsf(angle) < 30) {
    Serial.println("    barely any rotation seen: check the IMU. Aborting, nothing saved.");
    polarity = old;
    return;
  }
  polarity.gyro = angle < 0;

  polaritySave();
  auto flag = [](bool v) { return v ? "invert" : "normal"; };
  Serial.printf("saved: encoder L %s R %s | motor L %s R %s | gyro %s\n", flag(polarity.encL), flag(polarity.encR),
                flag(polarity.motL), flag(polarity.motR), flag(polarity.gyro));
  imuCalibrate();
}

// Front-sensor target for "axle at the cell centre". With the mouse's back
// against a wall its axle sits a known distance behind the centre, so a
// temporary wall across the far side of that cell gives an exact reading.
void calibrateFront() {
  const float axleBehindCentre = CELL_MM / 2 - HALF_WALL_MM - BACK_TO_AXLE_MM;
  Serial.println("calfront: in the start cell, put the mouse's back against the back wall, centred,");
  Serial.println("          and put a wall across the far (open) side of the cell. Then press a key.");
  waitKey();
  float d[3];
  tofAverage(10, d);
  if (d[TOF_FRONT] >= 9000) {
    Serial.println("no front reading: is the wall there?");
    return;
  }
  Serial.printf("front reads %.0f mm here, so at the cell centre it should read %.0f mm\n", d[TOF_FRONT],
                d[TOF_FRONT] - axleBehindCentre);
  Serial.printf("-> set FRONT_CENTRED_MM = %.0f in config.h (currently %.0f)\n", d[TOF_FRONT] - axleBehindCentre,
                FRONT_CENTRED_MM);
}

void help() {
  Serial.println(
      "bring-up:  check (measure motor/encoder/gyro directions) | calfront | i2c | imu | tof | enc\n"
      "           motor <Lvolts> <Rvolts> [ms] | fwd <mm> | turn <deg> | walls\n"
      "maze:      search | run | maze | clear\n"
      "BOOT button: short press = search or run, long press = clear maze");
}

void handleCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " ");
  char* a2 = strtok(nullptr, " ");
  char* a3 = strtok(nullptr, " ");

  if (!strcmp(cmd, "help")) help();
  else if (!strcmp(cmd, "i2c")) i2cScan();
  else if (!strcmp(cmd, "imu")) streamImu();
  else if (!strcmp(cmd, "tof")) streamTof();
  else if (!strcmp(cmd, "enc")) streamEncoders();
  else if (!strcmp(cmd, "motor") && a1 && a2) openLoopMotors(atof(a1), atof(a2), a3 ? atoi(a3) : 1000);
  else if (!strcmp(cmd, "fwd") && a1) closedLoopMove(false, atof(a1));
  else if (!strcmp(cmd, "turn") && a1) closedLoopMove(true, atof(a1));
  else if (!strcmp(cmd, "walls")) printWalls();
  else if (!strcmp(cmd, "check")) checkDirections();
  else if (!strcmp(cmd, "calfront")) calibrateFront();
  else if (!strcmp(cmd, "search")) doSearch();
  else if (!strcmp(cmd, "run")) doRun();
  else if (!strcmp(cmd, "maze")) {
    mm::Explorer ex(maze, hw);
    mm::Path route = ex.speedRunPath();
    printMaze(&route);
    Serial.println(ex.isProvenOptimal() ? "route proven optimal" : "route not proven yet");
  } else if (!strcmp(cmd, "clear")) {
    maze.reset();
    storageClearMaze();
    Serial.println("maze cleared");
  } else {
    Serial.printf("unknown command '%s'\n", cmd);
    help();
  }
}

}  // namespace

const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "CRASH (panic)";
    case ESP_RST_INT_WDT: return "CRASH (interrupt watchdog)";
    case ESP_RST_TASK_WDT: return "CRASH (task watchdog: something hogged the CPU)";
    case ESP_RST_WDT: return "CRASH (watchdog)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dipped)";
    case ESP_RST_EXT: return "reset pin";
    default: return "other";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\nmicromouse (last reset: %s)\n", resetReasonText());

  polarityLoad();
  uiBegin();
  motorsBegin();
  encOk = encodersBegin();
  i2cBegin();
  imuOk = imuBegin();
  tofAllOk = tofBegin();
  if (imuOk) imuCalibrate();
  motionBegin();

  Serial.printf("encoders %s, imu %s, tof L %s F %s R %s\n", encOk ? "ok" : "FAIL", imuOk ? "ok" : "FAIL",
                tofOk(TOF_LEFT) ? "ok" : "FAIL", tofOk(TOF_FRONT) ? "ok" : "FAIL", tofOk(TOF_RIGHT) ? "ok" : "FAIL");
  if (storageLoadMaze(maze)) Serial.println("loaded saved maze (type `maze` to view, `clear` to forget)");
  uiSet(encOk && imuOk && tofAllOk ? UiColour::IDLE : UiColour::ERROR);
  help();
}

void loop() {
  static char line[64];
  static int len = 0;
  static unsigned long lastChar = 0;
  auto runLine = [&]() {
    line[len] = 0;
    Serial.println();
    handleCommand(line);
    len = 0;
  };
  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\r' || c == '\n') {
      if (len) runLine();
    } else if ((c == '\b' || c == 127) && len > 0) {
      len--;
      Serial.print("\b \b");
    } else if (len < int(sizeof line) - 1 && c >= ' ') {
      line[len++] = c;
      Serial.print(c);  // echo: most serial monitors don't show what you type
    }
  }
  // Monitors set to "No line ending" never send Enter: run after a pause.
  if (len && millis() - lastChar > 1000) runLine();

  if (uiButtonDown()) {
    unsigned long ms = uiWaitPress();
    uiBlink();  // press seen
    if (ms > UI_LONG_PRESS_MS) {
      Serial.printf("BOOT long press (%lu ms): maze cleared\n", ms);
      maze.reset();
      storageClearMaze();
      uiSet(UiColour::IDLE);
    } else {
      Serial.printf("BOOT short press (%lu ms)\n", ms);
      autoStart();
    }
  }
  delay(5);
}
