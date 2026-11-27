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

From the user's photo (2026-11-27): the cell is a **white square frame on all 4 sides** (~3.5 cm wide) around a 12 cm black square, in a 1-cell corridor with wooden walls left and right. A strip of black tape (~2 cm wide, glossy at one end) runs down the middle of the black square, along the direction of travel; it reads blacker (~1000) than the card (~950). Because the frame is on all sides, the robot also sees the white-black-white pattern when it comes from the line side.

Still unknown: the exact size of the small black part, what the line's background is (assumed white), where the line section ends, and whether the cell can be entered from another side.

## Current state

| Step | Status | Result |
|---|---|---|
| IR pins | **Done** | The pins in `ir_callibration.cpp` are correct: A7..A0, enable on pin 52. `config.h` was wrong (A15..A8, 49) and has been fixed. |
| IR calibration | **Done** | New values measured with ir_02 `c` and copied into `config.h` |
| Line follow in `ir_02.cpp` | **Done, works** | The user confirmed it follows well with BASE_PWM 95 |
| `LineFollower` library | **Written, builds** | **Not tested on the robot** |
| Special-cell detector | **Hand test (`v`) passed 3/3** (2026-11-27) | white1 3.5 / 3.4 / 3.5 cm; black 11.0 / 9.6 / 10.8 cm (nominal 12, reads short by hand); FOUND each time on white2; small black ~2.4-3.2 cm; line starts ~5 cm after FOUND on a white background (~750-880). Run 3: a **3.1 cm all-black patch ON the line section** (~4 cm after the line starts, raw 900-990). Black floor raw 930-970 vs thresholds 869-913 |
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
| `MARKER_B_MIN_CM` / `_MAX_CM` | 8.5 / 16.0 | Black square (nominal 12). Motor runs read 11.5-11.6; one run saw a false 5.3 cm black band. 8.5 sits between the two |
| `MARKER_W2_CONFIRM_CM` | 1.5 | Second strip seen this long = FOUND |
| `MARKER_FINISH_MAX_CM` | 30 | Driving limit to finish a pattern cut off by the end of a move |
| `LINE_SEARCH_DUTY` | 20 % | Straight drive while looking for the line |
| `LINE_SEARCH_MAX_CM` | 60 | |
| `LINE_FOUND_CM` | 1.0 | 1-4 sensors black for this long = line found |
| `LINE_END_SENSORS` / `LINE_END_CM` | 6 / 6.0 | Back on the black floor = end of the line section. Raised from 3 because of the 3.1 cm patch on the line section |
| `LINE_END_MAX_CONTRAST` | 60 | End also needs max-min raw < 60 (black floor 20-50; grey line background + line 70-130) |
| `LINE_MIN_FOLLOW_CM` | 20 | No 'line ended' check for the first 20 cm of following (ramp at the start of the line section) |
| `LINE_LOST_MS` | 2000 | Raised from 400 by the user |

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

## Session 2 (2026-11-27)

- 3 hand runs of `v`: the pattern was found every time (see Current state).
- Changed `MARKER_B_MIN_CM` 8 → 7 and `LINE_END_CM` 3 → 6.
- Fixed a log bug: the first `marker: floor` line after `v` was restarted printed a negative length (`-18.4 cm`), because `markerReset()` didn't reset the band start. It now does (`mFresh`). Detection was not affected.
- Open question: what is the all-black patch about 4 cm after the line starts (a cross line, a joint, a darker piece of board)? If it can be longer than 6 cm, line mode would end early there.
- Not done yet: the false-trigger check (`v` along a normal corridor and over the line section).

- `m` (motors) 3 runs: runs 2 and 3 perfect (white1 3.5/3.6, black 11.6/11.5, FOUND at 27.4/27.5 cm). Run 1 first saw a different sequence: white 3.5, **black 5.3** (rejected as too short), white 4.1, black 16 (rejected as floor). It then found the real pattern (white 3.5, black 11.6) at 56.6 cm. **Unknown what that first sequence was**: ask where the robot started and what is on the floor there.
- Changed `MARKER_B_MIN_CM` 7 → 8.5, halfway between the false 5.3 and the real 11.5.

- Added `@<cm> <bar>` to every `marker:` line, so an unexpected sequence (like `m` run 1) can be traced to a spot on the floor.

- Motor tests: `m` 3/3 (white1 3.5-3.6, black 11.4-11.5, FOUND 26.3-27.8 cm). `f` 3/3 marker found. `l`/`f`: the line was found 6.4-6.7 cm after FOUND each time.
- **Problem:** in all 6 `l`/`f` runs line mode ended `line ended (black floor)` only **8.4-10.1 cm** after the line was found, with black=8 from about 4 cm in. The hand logs show the line section's background reading grey (870-940, which is 'black' by the midpoint) while the line itself reads 980-1000. So this is most likely a false end, not the real end of the line. **Ask the user where the robot physically stopped.**
- Fix: the end also needs `lineContrast()` (max-min raw) < `LINE_END_MAX_CONTRAST` 60. On real black floor the readings are even (20-50); on grey background with the line, 70-130. The follow log now prints every 50 ms with `c=` and the bar, and `v` shows `c=`.
- The user had not uploaded the build with the `@cm bar` marker logging when running these tests.

- The user found a **small ramp at the start of the line section**. On it the IR array lifts off the surface, so every sensor reads dark and even, which looks exactly like the black floor and ended line mode early. Added `LINE_MIN_FOLLOW_CM` 20: no end check until 20 cm of following. The lost check stays active. To do: measure the ramp length and set this to (line start → top of ramp) + about 5 cm.

- `l`/`f` with `LINE_MIN_FOLLOW_CM` 20 (4 runs): marker found (white 3.5-3.6, black 11.5), line found 6.3 cm after FOUND, ramp crossed (`######## c=6-47` from ~3.5 to 13.6 cm, no false end), line followed on top from ~15 to 29 cm. **The test track ends at the top of the ramp** (the rest of the line section isn't connected yet), so everything after ~30 cm in those logs (line lost, spinning, up to 100 cm on plain white, end on black floor) is the unfinished track, not a bug.
- Known weakness for when the full track is built: on a plain white surface with no line, `lost` (normalised sum < `LINE_LOST_SUM` 200) didn't fire, because that white reads slightly greyer than the calibration white, and `LINE_LOST_MS` 2000 = 8-14 cm of spinning at this speed. Check both on the full track.

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
