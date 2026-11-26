# Handoff: IR array, line following and the special cell (branch `steering-tuning`)

Last updated: 2026-11-26

## Goal

1. Calibrate the 8-channel IR array and check the old line-follow code and its tuning.
2. Move the working line follower into the `LineFollower` library.
3. Detect the special cell while maze solving, then switch to line-following mode.
4. Get back to maze solving after the line section (not started).

## The special cell (as described by the user)

The maze floor is **black**. Driving forward into the special cell, the IR array sees:

```
black floor -> WHITE strip ~3.5 cm -> BLACK square ~12 cm (12 cm wide) -> WHITE strip ~3.5 cm
            -> small black part -> black line starts (switch to line mode here)
```

Still unknown: the exact size of the small black part, what the line's background is (assumed white), where the line section ends, and whether the cell can be entered from another side.

## Current state

| Step | Status | Result |
|---|---|---|
| IR pins | **Done** | The pins in `ir_callibration.cpp` are correct: A7..A0, enable on pin 52. `config.h` was wrong (A15..A8, 49) and has been fixed. |
| IR calibration | **Done** | New values measured with ir_02 `c` and copied into `config.h` |
| Line follow in `ir_02.cpp` | **Done, works** | The user confirmed it follows well with BASE_PWM 95 |
| `LineFollower` library | **Written, builds** | **Not tested on the robot** |
| Special-cell detector | **Written, builds** | **Not tested on the robot.** Based on the band pattern above |
| Maze solver hook | **Written, builds, OFF** | `LINE_MODE 0` in `maze_solver_small.cpp` |
| Re-align after line mode | Not started | TODO in `navigateStep()` |

### Current values (`include/config.h`)

| Name | Value | Notes |
|---|---|---|
| `IR_PINS` | {A7, A6, A5, A4, A3, A2, A1, A0} | [0] = right most, [7] = left most |
| `PIN_IR_EN` | 52 | |
| `WHITE_VALUES` | {805, 722, 731, 733, 720, 727, 743, 740} | Raw reads HIGH on black |
| `BLACK_VALUES` | {1022, 1017, 1020, 1019, 1017, 1016, 1018, 1017} | |
| `LINE_BASE_PWM` / `LINE_MAX_PWM` | 95 / 100 | **0-255 scale** (as in ir_02); the library converts to duty % |
| `LINE_KP` / `LINE_KI` / `LINE_KD` | 0.025 / 0 / 0.053 | Unchanged from ir_02 |
| `LINE_SENSOR_EXPONENT` | 2.5 | |
| `LINE_LOST_SUM` | 200 | |
| `LINE_IS_WHITE` | 0 | 1 = white line on black |
| `MARKER_FULL_SENSORS` | 6 | Sensors that must agree for a full-width band |
| `MARKER_W_MIN_CM` / `_MAX_CM` | 2.0 / 6.0 | White strip (nominal 3.5) |
| `MARKER_B_MIN_CM` / `_MAX_CM` | 8.0 / 16.0 | Black square (nominal 12) |
| `MARKER_W2_CONFIRM_CM` | 1.5 | Second strip seen this long = FOUND |
| `MARKER_FINISH_MAX_CM` | 30 | Driving limit to finish a pattern cut off by the end of a move |
| `LINE_SEARCH_DUTY` | 20 % | Straight drive while looking for the line |
| `LINE_SEARCH_MAX_CM` | 60 | |
| `LINE_FOUND_CM` | 1.0 | 1-4 sensors black for this long = line found |
| `LINE_END_SENSORS` / `LINE_END_CM` | 6 / 3.0 | Back on the black floor = end of the line section |
| `LINE_LOST_MS` | 400 | |

## Active fixes / things to watch

1. **Motor PWM in the library.** ir_02 drives the motors with `analogWrite` (0-255, about 490 Hz). The library must use `setMotors()` (duty %, 20 kHz), because once `setPWM` has set up timers 3 and 4, `analogWrite` on those pins gives the wrong duty. Speeds are converted: 95 → 37 %, 100 → 39 %. The algorithm and gains are unchanged, but the PWM frequency is different. **Check that `g` in `line_mode_test` follows as well as ir_02 did.** If it's weaker, raise `LINE_BASE_PWM` a little.
2. **Pattern across cell moves.** The array can still be in the black square when a 25 cm move ends. The maze solver keeps a running distance (`mazeOdoCm`) and does not reset the detector between moves. If the last move ended mid-pattern (`MARKER_STARTED`), `navigateStep()` calls `runLineMode(true)`, which first drives straight to finish the pattern. If the pattern breaks, it returns `LINE_NO_MARKER` and maze solving continues.
3. **The detector resets on every turn** (`turnBy()`).

## Changes made

**`include/config.h`**
- IR pins fixed (A7..A0, enable on 52).
- New `WHITE_VALUES` / `BLACK_VALUES`.
- Removed the unused `DEFAULT_IR_THRESHOLDS` and `DEFAULT_BASE_PWM/MAX_PWM/KP/KI/KD` (old values 100/120/0.030/0/0.100).
- Added all `LINE_*` and `MARKER_*` values.

**`Arduino_Test/irarray_test/ir_02.cpp`**
- Pins now come from `config.h`.
- Removed the old pin definitions: IR A7..A0 / EN 52, which were the correct pins, and motor RPWM 4 / 9, which were outdated (now 5 / 7).
- Removed the unused encoder ISRs.
- Tuning and algorithm unchanged. The user later set `BASE_PWM` to 95 and pasted in the new calibration.
- Added a serial menu: `c` calibrate (6 s sweep, compares with the stored values), `r` live view (motors off), `g` follow, `o` original values, `p` print.

**`lib/LineFollower/`**
- Renamed `LineFollwer.cpp` to `LineFollower.cpp`.
- Replaced the old class, which used a `MotorDriver` class that no longer exists, with free functions:
  - Sensors: `lineBegin`, `lineReadSensors`, `lineRaw`/`lineNorm`/`lineIsBlack`, `lineBlackCount`/`lineWhiteCount`.
  - PID: `lineReset`, `linePosition`, `lineFollowStep` (same maths as ir_02 `followLinePID`).
  - Odometry: `lineOdoReset`, `lineOdoCm`, `lineDriveStraight` (encoder L/R sync with `ENC_SYNC_KP`, `RIGHT_DUTY_TRIM`).
  - Marker: `markerReset`, `markerUpdate(cm)`, `markerState`, `markerStepName`, `markerVerbose`. Steps: floor → white1 → black → white2 → FOUND (plus wait-black after a long white area). Prints `marker: <step> <len> cm -> <next>` on each change.
  - Line mode: `runLineMode(finishMarker, abortCheck)`. Phase 0 finishes the pattern if needed, phase 1 drives straight until the line pattern appears, phase 2 runs PID until the black floor returns (end) or the line is lost.

**`Arduino_Test/irarray_test/line_mode_test.cpp`** (new) + **`platformio.ini`** env `line_mode_test`
- `v` view, motors off: push by hand; shows the bar, distance, step and band lengths.
- `g` plain follow.
- `m` drive until the marker is found.
- `l` line mode only (start on the second white strip).
- `f` full sequence: marker, then line mode.
- Any key stops a test.

**`Arduino_Test/maze_solve_test/maze_solver_small.cpp`**
- `#include "LineFollower.h"`; `LINE_MODE` switch (default 0, can also be set with `-DLINE_MODE=1`); `lineModePending`, `mazeOdoCm`.
- `moveForwardPID()`: calls `markerUpdate()` every 3 ms. On FOUND it stops the move and sets `lineModePending`.
- After each move, `mazeOdoCm` is increased by the distance driven.
- `navigateStep()`: if pending or `MARKER_STARTED`, it calls `runLineMode(true)`, then `markerReset()`.
- `turnBy()`: `markerReset()`.
- `setup()`: `lineBegin()` when `LINE_MODE`.

Builds checked: `irarray_test`, `line_mode_test`, `maze_solve_test` (with `LINE_MODE` 0 and 1). None of this is committed yet.

## Failed or rejected methods

| Method | Why it failed |
|---|---|
| IR pins A15..A8 / EN 49 in `config.h` | Wrong pins. The correct ones are in `ir_callibration.cpp` (A7..A0 / 52). |
| First rewrite of ir_02 with tuning converted to duty % (base 25, Kp 0.010, Kd 0.020, trim, steer sign, lost-line stop) | Rejected by the user: tuning and algorithm must not change. Reverted. |
| `analogWrite` inside the library | Can't be mixed with `setPWM` (timers 3 and 4 reconfigured, so the duty is wrong). The library converts to duty % instead. |
| Old `LineFollower` class | Referenced a `MotorDriver` class and `DEFAULT_*` values that no longer existed; used `map()` without the 2.5 curve. |
| First marker detector: "5 or more white sensors for 1.5 cm = white square" | Based on a wrong description of the cell. Replaced by the white–black–white band detector. |
| Old calibration in ir_02 (white ~730-800, black ~975) and in `config.h` (white 158-516) | Outdated. Replaced by the new `c` values. |
| Running `pio` directly in PowerShell | Not on the PATH. Use `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"`. |

## Next steps

1. **Upload `line_mode_test`** (`pio run -e line_mode_test -t upload`, then monitor at 115200 baud).
2. **`v` by hand through the special cell**, 3 times, pushing forward only. Check the printed lengths: `white1` ≈ 3.5 cm, `black` ≈ 12 cm, then FOUND.
   - Lengths scaled wrong: check `TICKS_PER_CELL_25CM`, or widen the `MARKER_*_MIN/MAX` limits.
   - Stuck on one step: check the bar and `black=` per band; lower `MARKER_FULL_SENSORS` to 5 if a strip shows only 4-5 white sensors.
   - False-trigger check: push along a normal black corridor and over the line section. It must never reach FOUND.
3. **`g`**: compare with ir_02 (see Active fix 1).
4. **`m`** 3 times: it should stop on the second white strip.
5. **`l`** from the second white strip: line found, followed, `line ended (black floor)`. If the small black part is taken for the line or for the end, raise `LINE_FOUND_CM` or `LINE_END_CM`.
6. **`f`** 3 runs in a row.
7. **Maze solver**: set `LINE_MODE 1` and upload `maze_solve_test`.
   - Start 1-2 cells before the special cell. Expect `marker: ... -> FOUND`, `Special cell found - stopping the move`, `Special cell -> line mode`, `Line mode done: ...`.
   - Run a normal path without the special cell. There must be no FOUND, and moves must behave as before.
8. **Re-align after line mode** (TODO in `navigateStep()`). Needs from the user: what is at the end of the line section (wall, cell entrance, exit), and whether the special cell can be entered from another side.
9. Commit on `steering-tuning` once steps 2-7 pass.
10. The straight-line tuning work is a separate track: see `handoff/handoff.md` (step 4, gyro + walls, still in progress).

## How to run a test

1. `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e line_mode_test -t upload`
2. `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor` (115200 baud)
3. Type a command letter and press ENTER. Any key stops a running test. Keep a hand ready whenever the motors are on.
4. To log to a file instead, close the monitor and run `powershell -File tools\watch_serial.ps1 -Seconds 300`; the output goes to `tools/serial_log.txt`.
