# Handoff: straight-line tuning (branch `steering-tuning`)
# Config.h PID values for straight movement

Last updated: 2026-11-26

## Goal

Recheck and fine-tune the robot's straight-line movement, one layer at a time, without recalibrating everything from scratch:

1. Distance: one move = exactly 25 cm.
2. Encoders: both wheels stay in step.
3. Gyro: the robot holds its heading.
4. Side walls (ultrasonic): the robot stays centred in the corridor.
5. Full maze run.

The full procedure, with pass limits and result tables, is in `Doc/straight_tuning.md`.

## Current state

| Step | Status | Result |
|---|---|---|
| 1. Distance (25 cm) | **Done** | All runs measured 24.5 cm with 1111 ticks, so changed to 1134. Rechecked: 25 cm. |
| 2. Encoders only | **Done** | Straight, but 2–3 cm drift over 1 m. The cause is wheel slip / wheel size, which encoders can't see. Accepted; the gyro fixes it. |
| 3. Gyro only | **Done** | After the end-of-move fix (below) and starting square against a board: heading within ±0.5° while driving, correct distance (`encCm` ≈ 101 for 1 m). |
| 4. Gyro + walls | **In progress** | The robot weaves in the corridor (yaw 0–7.5° within a cell). The walls steer too hard for the sensor noise. |
| 5. Maze run | Not started | |

### Current values

| Variable | File | Value | Notes |
|---|---|---|---|
| `TICKS_PER_CELL_25CM` | `include/config.h` | 1134 | Step 1 result (was 1111) |
| `kpL` / `kpR` | `maze_solver_small.cpp` | 4.0 / 4.5 | kd and ki = 0 |
| `MIN_DUTY_LEFT` / `_RIGHT` | `config.h` | 16 / 13 | Unchanged |
| `MAX_DUTY` | `include/globals.h` | **35** | Changed from 28 by the user after steps 1–3. **Not tested** with the step 1–3 values: higher speed means more coasting, so the 25 cm distance and the stop behaviour must be rechecked. |
| `HEADING_KP` | `config.h` | 2.0 | Was 1.5 in step 3. Raised to 2.0 by the user. |
| `HEADING_KD` | `config.h` | 0.0 | |
| `ENC_SYNC_KP` | `config.h` | 0.1 | Used only in step 2 |
| `WALL_YAW_GAIN` | `config.h` | 1.5 | Lowered from 3.0 for step 4. **Not tested yet.** |
| `MAX_WALL_YAW` | `config.h` | 10 | Candidate to lower to 6 |
| `WALL_CENTER_CM` | `config.h` | 3.2 | Needs rechecking (see step 4a) |

## Active fixes

1. **End-of-move spin (fixed, in `moveForwardPID`).** Each wheel used to have its own stop target. While driving, the gyro makes the wheels turn different amounts on purpose. At the end, the leading wheel stopped and the other caught up, spinning the robot about 30° left in the last 4 cm of every cell. Both wheels now use the **average** distance.
2. **Steering sign when a wheel reverses (fixed).** When a wheel overshot and drove backwards, the steering correction pushed it the wrong way. The steering is now flipped for a reversing wheel.
3. **Step 4 weave (in progress).** See Next steps.

## Changes made

**`Arduino_Test/maze_solve_test/maze_solver_small.cpp`**
- Tuning stages: a `TUNE_STAGE` switch (0 = normal maze; 1 = distance, no steering; 2 = encoders only; 3 = gyro only; 4 = gyro + walls) and `TUNE_CELLS`. In stages 1–4, each run starts with the hand signal over the front sensor and ends with a `RESULT` line.
- `stageRun()`: runs one test (gyro calibration, start readings, N cells, summary).
- Encoder steering for stage 2: `ENC_SYNC_KP × (left − right ticks)`.
- Gyro steering now includes `HEADING_KD × gyroRate()`.
- The gyro keeps updating during the 200 ms pause after each move, so coasting is included in the heading.
- Logging: the `mv` lines now include `cm=`, `LR=` and `st=`. A `move done: L= R= LR= gyro= ms=` line is printed after every move.
- The two fixes from Active fixes.
- `TUNE_STAGE` defaults to 0, so the normal maze build behaves as before except for these fixes.

**`include/config.h`**: `TICKS_PER_CELL_25CM` set to 1134; added `HEADING_KD` and `ENC_SYNC_KP`. (`HEADING_KP` and `WALL_YAW_GAIN` were changed by the user.)

**`platformio.ini`**: new envs `tune1_distance` (1 cell), `tune2_encoder`, `tune3_gyro`, `tune4_walls` (4 cells each). Each sets `TUNE_STAGE` with `-D`. Upload with, for example, `pio run -e tune4_walls -t upload`.

**`Doc/straight_tuning.md`**: the step-by-step guide.

None of this is committed yet.

## Failed or rejected methods

| Method | Why it failed |
|---|---|
| `straight_test.cpp` for the 25 cm check | It reads the encoder counts without `ATOMIC_BLOCK`. A torn read at the 1024-tick byte boundary stopped runs early (runs where a wheel shows exactly 1024). It also coasts after stopping, unlike the maze code. It was replaced by `tune1_distance`. |
| Encoder-only steering (step 2) as the final solution | Keeps the ticks equal, but the robot still drifts 2–3 cm per metre. The left wheel counts more ticks for the same distance (wheel size or slip). Only the gyro fixes this. |
| `kpL` / `kpR` = 1.0 (softer stop) | Tried by the user, then reverted to 4.0 / 4.5. |
| A friend's tuned set (`HEADING_KP` 3.0, `HEADING_KD` 0.1, `kdL` 0.5, `kdR` 0.2, `MAX_STEER_DUTY` 10, `WALL_YAW_GAIN` 0) | The user reported it didn't work. Known issues: the wheel D term is noisy (about a 1 ms loop, so 0 or 1 tick per pass) and asymmetric, and wall centring was turned off. |
| `HEADING_KP` 1.5 in step 3 before the average-distance fix | Seemed to wobble at the end of each cell. The real cause was the per-wheel stop targets, not the gain. |
| `WALL_YAW_GAIN` 3.0 in step 4 | Weaves. A 1–2 cm sensor jump changes the target heading by 3–6°, and the steering often hits its ±5 limit. In one run it scraped the right wall and the Arduino restarted (probably a battery dip from the stalled motors). |

## Known issues still open

- Each stop turns the robot about 1.5–3° to the right (up to 6° in some runs). It doesn't build up, because the next cell corrects it in about 2 cm and turns aim at the absolute heading. Revisit if the maze run shows problems.
- Side sensor noise of about 1–2 cm between readings with the robot straight.
- Start readings in step 4 had L much lower than R (2.0 vs 5.4, and 2.8 vs 5.8). It's unknown whether the robot was off-centre or one sensor is offset.
- `MAX_DUTY` 35 hasn't been checked against the step 1–3 values.

## Next steps

1. **Recheck step 1 at `MAX_DUTY` 35** (`tune1_distance`, 3–5 runs). If the distance isn't 25.0 ± 0.3 cm, set `TICKS_PER_CELL_25CM` = 1134 × 25 ÷ measured cm. Quickly repeat step 3 (`tune3_gyro`) to make sure there's no weave at the higher speed. If that's a problem, go back to 28.
2. **Step 4a, centre reading:** place the robot exactly centred with a ruler, give the hand signal, and read the `start L= R=` line (3 placements).
   - If L ≈ R but not 3.2, set `WALL_CENTER_CM` to that value.
   - If L ≠ R when centred, one sensor reads offset. This needs a correction: the wall error is `L − R` when both walls are present.
3. **Step 4b/4c with `WALL_YAW_GAIN` 1.5:** 3 runs from the centre, then 3 runs starting 1.5 cm off-centre. Pass if `rmsWallErr` < 0.5 cm, `closeFrac` = 0, and it recovers from the offset within 1 cell.
   - Still swings: set `MAX_WALL_YAW` from 10 to 6.
   - Small weave left: set `HEADING_KD` to 0.05.
   - Change one value at a time.
4. **Step 5:** upload `maze_solve_test` and check that cells end near the centre after straight moves and turns. If straight moves are fine but positions are off after turns, tune the turns (`TURN_KP`, `TICKS_PER_90_*`) instead.
5. Commit the work on `steering-tuning` once step 4 passes.

## How to run a test

1. Upload, for example `pio run -e tune4_walls -t upload`.
2. Close the serial monitor, then log with `powershell -File tools\watch_serial.ps1 -Seconds 300`. The output goes to `tools/serial_log.txt`.
3. Place the robot square (use a board as a start guide), cover the front sensor with your hand, then take it away. The robot waits 3 s.
4. Read the `move done` and `RESULT` lines, and measure the distance and drift with a tape.
