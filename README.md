# Micromouse

A classic 16×16 micromouse (UKMARS rules: 180 mm cells, start in a corner, 2×2 goal in the centre).

**Hardware:**
- ESP32-C6 DevKitC-1
- DFRobot DRI0044 motor driver
- 2× GA12-N20 geared motors with encoders, 44 mm wheels
- MPU-6050 gyro (DFRobot SEN0142)
- 3× VL53L0X ToF sensors (left, front, right)
- 2S battery → 5 V buck

The original wiring diagram is `WhatsApp Image 2026-09-26 at 12.43.39.jpeg`, but the pins have since changed: [`src/config.h`](src/config.h) is the source of truth.

## Layout

| Path | What |
|---|---|
| `lib/maze/` | Maze logic in pure C++ (no Arduino): wall map, flood fill, path planner, explorer, maze-file parser and generator. Shared by the firmware, the simulator and the tests. |
| `lib/maze/mouse_io.h` | The interface between the explorer and the world. The simulator implements it against a maze file, and the firmware (`src/hw_mouse.*`) implements it with motors and sensors. |
| `src/` | ESP32-C6 firmware: drivers, 500 Hz motion control, serial CLI. |
| `test/test_maze/` | Unit tests for `lib/maze`, which run on your computer. |
| `tools/sim/` | Desktop simulator that runs the real explorer on maze files or random mazes. |

### How the mouse searches

1. The mouse stops at every new cell centre. There it reads the left, front and right walls, updates the map, and re-plans.
2. Planning is a flood fill over (cell, heading). The cost counts cells *and* turns, so it prefers straight routes.
3. The mouse searches start → goal → start, adding round trips until the route is **proven optimal**. Proven means the route cost with unknown walls assumed open equals the cost with them assumed closed.
4. The speed run then uses only walls that have been seen.
5. The map is saved to flash after every update.

Side walls are only sampled at cell centres. That's because the side sensors would also see the posts at every cell boundary.

## Build and test

```sh
pio test -e native                      # unit tests for the maze logic
make -C tools/sim                       # build the simulator
./tools/sim/sim --random 500            # 500 random legal mazes
./tools/fetch_mazes.sh                  # download ~520 real contest mazes (not committed: no licence)
./tools/sim/sim third_party/mazefiles/classic/*.txt
./tools/sim/sim -v third_party/mazefiles/classic/uk2024-hazlemere.txt   # draw the map and route
pio run -e mouse                        # build firmware
pio run -e mouse -t upload              # flash
pio device monitor                      # serial CLI, 115200 baud, type `help`
```

The platform is pinned to pioarduino 55.03.311 (Arduino core 3.3.11), because later releases need PlatformIO Core ≥ 6.2. Plug into the DevKit's **UART** USB port. To use the native USB port instead, uncomment the `ARDUINO_USB_CDC_ON_BOOT` lines in `platformio.ini`.

## Bring-up (do these in order, mouse on a stand for the motor steps)

Everything marked `CALIBRATE` in `src/config.h` is a placeholder until you measure it.

0. **`check`**: measures which way the encoders, motors and gyro count, and saves the result to flash. It overrides the `INVERT_*` defaults in `config.h`. You push the mouse forwards, lift it while each motor is pulsed, then turn it left by hand. Run it again after any rewiring.

1. **`i2c`**: should list 0x68 (MPU-6050) and 0x30/0x31/0x32 (the ToF sensors after they're readdressed).
   - If nothing appears, check the SDA (GPIO22) / SCL (GPIO21) wiring.
   - If one ToF shows at 0x29 or is missing, check its XSHUT wire.
2. **`imu`**: the rate should sit near 0 at rest. Turn the mouse 90° left by hand and the angle should read about +90. If it reads −90, set `INVERT_GYRO`.
3. **`tof`**: put the mouse centred in a cell with walls on both sides and ahead. Set `SIDE_NOMINAL_MM` and `FRONT_CENTRED_MM` from the readings. Then check that `SIDE_WALL_MM` and `FRONT_WALL_MM` separate "wall" from "open" readings cleanly.
4. **`enc`**: turn each wheel forwards and check that its count goes **up**. If it goes down, set `INVERT_ENC_*`.
   - Mark a wheel and turn it exactly 10 revolutions. Counts ÷ 10 = `COUNTS_PER_WHEEL_REV`.
   - This also gives the gear ratio: for the usual 7-pulse N20 encoder, ratio ≈ counts per rev ÷ 28. For example, 840 means 30:1.
5. **`motor 1.5 1.5`**: both wheels should spin forwards and both encoder changes should be positive. Otherwise set `INVERT_MOTOR_*`.
   - Set `MOTOR_SUPPLY_VOLTS` to whatever feeds the driver's VM pin. If that's the 2S pack, keep `MAX_MOTOR_VOLTS` ≤ 6 so the 6 V N20s aren't over-driven.
6. **`fwd 900`** (5 cells) on the floor: measure the real distance and scale `COUNTS_PER_WHEEL_REV`.
   - **`turn 360`**: check it ends where it started. The gyro sets the angle, and `WHEELBASE_MM` only affects feedforward.
   - Tune `FF_*` first, then `FWD_KP`/`ROT_KP` (add a little `*_KD` if it oscillates).
7. **`walls`** in a few cells: check the sensed walls are right.
8. **`search`**, then **`run`**. Always start with the mouse **in the start cell, facing out, with its back touching the wall behind it**. The mouse finishes every search and run in that same position, so you can start the next one without touching it. Without a laptop:
   - Short-press BOOT to search, or to run once the route is proven.
   - Long-press BOOT to forget the maze.
   - Start either one with BOOT or by waving a hand close to the front sensor. Then hands off for 1 s while the gyro calibrates.

**New maze? Clear the map first** (long-press BOOT or type `clear`). The saved map is loaded at boot and trusted, and cells already visited are never sensed again. Without clearing, the mouse will drive the old maze's walls.

**LED:**

| LED | Meaning |
|---|---|
| Blue, steady | Idle, ready |
| White blip | Button press seen / new cell mapped |
| Amber, slow blink | Waiting for the start signal |
| **Red, fast flash** | Start seen: hands off, gyro calibrating |
| Cyan | Searching (or returning home) |
| Magenta | Speed run |
| Green | Finished OK |
| Red, steady | Error or aborted |
| White, steady | BOOT held over 1.5 s: let go to clear the map |

## Not done yet / next steps
- Continuous search (sensing walls while moving) and smooth turns for the speed run. Currently every turn is an in-place turn from a stop.
- There's no battery-voltage sensing, so feedforward drifts as the pack drains. A divider into a free ADC pin would fix this (GPIO1 is now a motor pin).
- `ukmars/mazerunner-core` (MIT) is a good reference for tuning the controller and for smooth turns.
