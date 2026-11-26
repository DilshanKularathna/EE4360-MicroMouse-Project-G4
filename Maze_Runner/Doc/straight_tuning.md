# Straight-line tuning guide

Tune in this order. Each stage builds on the one before, so do not move on until the stage passes.

| Stage | Firmware (env) | Steering used | Main variables |
|---|---|---|---|
| 1 Distance (25 cm) | `tune1_distance` | none | `TICKS_PER_CELL_25CM`, `kpL`, `kpR`, `MIN_DUTY_*` |
| 2 Encoders only | `tune2_encoder` | left/right tick difference | `ENC_SYNC_KP` |
| 3 Gyro only | `tune3_gyro` | gyro heading hold | `HEADING_KP`, `HEADING_KD` |
| 4 Gyro + walls | `tune4_walls` | walls set the heading target | `WALL_CENTER_CM`, `WALL_YAW_GAIN`, `MAX_WALL_YAW` |
| 5 Maze run | `maze_solve_test` | same as stage 4 + turns | check only |

Upload a stage: `pio run -e tune1_distance -t upload`

## Rules for every test

1. **One variable at a time.** Leave everything else fixed.
2. **Same conditions.** Charged battery, same floor, same start position. Use a ruler or a wall as a start jig so the robot always starts square.
3. **3 to 5 runs per value.** Use the average. If the runs disagree a lot, fix the setup before changing gains.
4. **Coarse, then fine.** Double the value until it gets worse (or wobbles), then try halfway between the last two good values.
5. **P first.** Add D only to stop wobble. Avoid I unless there is a constant offset P cannot remove.
6. **Measure, don't guess.** Tape measure for distance and drift, the serial log for the rest.

## How a run works

1. Power on and wait for `TUNE STAGE ...` on the serial monitor. It lists the current gains.
2. Place the robot on the start mark.
3. Cover the front sensor with your hand, then take it away. The robot waits 3 s and drives.
4. At the end it prints a `RESULT` line. Measure the distance and the sideways drift with a tape.
5. Repeat from step 2. No re-upload is needed between runs.

To log the runs to a file (close the serial monitor first):
`powershell -File tools\watch_serial.ps1 -Seconds 300`. The output goes to `tools/serial_log.txt`.

Log lines:
- `move done: L= R= LR= gyro= ms=`: after each cell. `LR` = left − right ticks, `gyro` = heading in degrees (right = +).
- `mv cm= LR= st= L= R= werr= desYaw= yaw=`: during the move. `st` = steering duty (+ = turning left).
- `RESULT ...`: summary of the run.

---

## Stage 1: Distance (25 cm)

**Goal:** one move = exactly 25.0 cm. There is no steering, so the robot may curve a little. That is fine here.

**Setup:** flat open floor, no walls within 15 cm in front. Mark a start line and pick one point on the robot to measure from (for example the rear edge of the chassis).

**Test:** `tune1_distance` (1 cell per run). Do 5 runs and record the distance each time.

**Tune:**
1. `TICKS_PER_CELL_25CM` (`config.h`): new value = old value × 25 ÷ average measured cm.
2. Check the `move done` line:
   - `ms=` should be well under 4000. `(TIMEOUT)` means the robot never settled at the end.
   - If the robot rocks back and forth at the end, lower `kpL`/`kpR` or `MIN_DUTY_*`.
   - If it stops short and buzzes, raise `MIN_DUTY_*` for that wheel.

**Pass:** 5 runs all within 25.0 ± 0.3 cm, and no timeouts.

| Run | ticks/cell | measured cm | L | R | ms |
|---|---|---|---|---|---|
| 1 | | | | | |

## Stage 2: Encoders only

**Goal:** the wheels stay in step, so the robot does not swing to one side. This also checks that the motors and encoders are healthy before the gyro is added.

**Setup:** open floor with a 1 m straight tape line. Start the robot on the line.

**Test:** `tune2_encoder` (4 cells = 1 m). Measure the sideways drift from the line at the end (left = −, right = +).

**Tune `ENC_SYNC_KP`** (`config.h`, start 0.1):
1. Run once with `0` to see the natural drift (baseline).
2. Try 0.05, 0.1, 0.2, 0.4 ... until the robot weaves (the `LR` value in the `mv` lines keeps flipping sign).
3. Use about 60 % of the value where it started weaving.

**What it can't fix:** if `LR` stays near 0 but the robot still curves the same way every time, the wheels are slightly different sizes or one slips. Equal ticks then do not mean equal distance. The gyro in stage 3 fixes this, so note the direction and move on.

**Pass:** `|LR|` < 10 ticks at the end, drift < 2 cm over 1 m, no weaving.

| ENC_SYNC_KP | drift cm | LR end | gyroEnd | notes |
|---|---|---|---|---|
| 0 | | | | baseline |

## Stage 3: Gyro only

**Goal:** the robot holds its start heading, whatever the wheels do.

**Setup:** same 1 m line as stage 2. Keep the robot still during the 1 s gyro calibration after the hand signal.

**Test:** `tune3_gyro` (4 cells). Record the drift, `gyroEnd`, and how much it weaves.

**Tune:**
1. `HEADING_KP` (start 0.5): try 0.5, 1.0, 1.5, 2.0, 3.0. Find where it starts to weave, and use about 60 % of that value.
2. `HEADING_KD` (start 0): add it only if it still weaves a little at the chosen KP. Try 0.02, 0.05, 0.1. D lets you use a slightly higher KP without weaving.
3. Optional check: tap the robot sideways mid-run. It should turn back to the line's heading within half a cell without weaving.

**Pass:** drift < 1 cm over 1 m, `|gyroEnd|` < 1°, no visible weave.

| HEADING_KP | HEADING_KD | drift cm | gyroEnd | weave? |
|---|---|---|---|---|
| | | | | |

## Stage 4: Gyro + side walls

**Goal:** the robot stays centred between walls without fighting the gyro.

**Setup:** a straight corridor of at least 4 cells with walls on both sides.

**Steps:**
1. **`WALL_CENTER_CM`:** place the robot exactly centred and give the hand signal. The `start L= R=` line shows the side readings. Average a few runs and use that value. If L and R differ when centred, the sensors are mounted unevenly.
2. **`WALL_THRESHOLD`:** in the `mv` lines, walls must read clearly below it and open sides clearly above it.
3. **`WALL_YAW_GAIN`:** start the robot about 1.5 cm off-centre (toward one wall). Watch `werr` in the log: it should shrink to near 0 within about 1 cell. Raise the gain if it is slow. Lower it if it overshoots to the other side or weaves.
4. **`MAX_WALL_YAW`:** the largest angle the walls may ask for. 6° is gentle, 10° corrects faster. Lower it if the robot swings hard when a wall appears or disappears.

**Pass:** `rmsWallErr` < 0.5 cm, `closeFrac` = 0, and it recovers from a 1.5 cm offset within 1 cell.

| WALL_YAW_GAIN | MAX_WALL_YAW | start offset | rmsWallErr | closeFrac | notes |
|---|---|---|---|---|---|
| | | | | | |

Optional: `AUTOTUNE_WALLS 1` in `maze_solver_small.cpp` sweeps the wall gains automatically. It also sweeps `HEADING_KP` (1.0 / 1.5), so set `TUNE_KPS` around your stage 3 value first.

## Stage 5: Maze run

Upload `maze_solve_test`. Check that every cell ends near the centre after both straight moves and turns. If straight moves are fine but the robot is off after turns, the problem is the turn tuning (`TURN_KP`, `TICKS_PER_90_*`), not these values.
