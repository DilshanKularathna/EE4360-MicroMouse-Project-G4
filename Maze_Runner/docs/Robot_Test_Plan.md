# Robot Test Plan, Round 1: Hardware Bring-up

Purpose: check each layer on the real robot, in order (sensors, then motion, then line following, then exploration), before a full run. When something fails, we then know which layer to fix.

## Tools

| Tool | How | Use |
|---|---|---|
| Bench sketch | `pio run -e bench -t upload`, then `pio device monitor -e bench` (115200 baud, type a letter and press Enter) | Single commands through the same code the robot runs |
| Main firmware | `pio run -e megaatmega2560 -t upload` | Full exploration. Start it by covering the front sensor with a hand, then taking it away |
| Serial log | Close the monitor, then `powershell -File tools\watch_serial.ps1 -Port COMx -Seconds 120` | Saves the output to `tools/serial_log.txt` |
| Simulator | `pio test -e native` (14 tests) and `pio run -e native -t exec` (report) | Explorer logic on the PC |

Bench commands:

| Key | Action |
|---|---|
| `s` | Stream F/L/R cm, wall booleans, 8 raw IR values, floor tile, yaw (any key stops) |
| `w` | One `readWalls()` |
| `t` | One `readFloorTile()` |
| `f` | Move one cell |
| `r` / `l` / `u` | Turn right / left / 180 |
| `a` | Align on the front wall |
| `g` | Gyro drift over 60 s |
| `c` | IR calibration sweep (5 s) |
| `L` | Line-follow until bridge exit (15 s max) |

With `LOG_ADAPTER 1` in `config.h`, the adapter prints one line per action, in both the bench sketch and the main firmware:
```
WALLS F=4.3 L=3.1 R=27.0 -> F1 L1 R0
FLOOR black
MOVE ok 24.8cm 1420ms
TURN +1 err 0.8deg 980ms
```

**Do not run `Arduino_Test/irarray_test/ir_02.cpp` with the motors powered.** It still uses the old motor pins (4/6/9/8), but the wiring is now 5/6/7/8, so it would drive the wrong outputs.

## Part 1: PC checks (no robot)
| # | Command | Pass |
|---|---|---|
| P1 | `pio run` | Firmware builds |
| P2 | `pio test -e native` | 14/14 tests pass |
| P3 | `pio run -e native -t exec` | 100% on every line with perfect sensors |

## Part 2: On the robot, in this order

### A. Sensors (robot still)
| # | Test | How | Report |
|---|---|---|---|
| A1 | Wall readings | `s`, robot centred in a cell, in 4 setups: walls L+R, F only, none, all three. 20 s each | Typical F/L/R cm for each setup, and how many lines had a wrong wall boolean |
| A2 | Opening reading | `w`, facing an open gap with nothing within ~1.5 m | `F=` value (expect 100 = no echo) |
| A3 | Floor tiles | `s`, axle on the tile centre: black floor, white tile, marker tile (9 cm white square, 3 cm black centre), bridge tile (white with 3 cm black line). Repeat under a second light source | 8 IR values and the floor result for each. Expect the marker NOT to be detected while stopped |
| A4 | IR calibration | `c` over black floor and white tile | The printed values compared with `config.h` |
| A5 | Gyro drift | `g` | Drift over 60 s (fine if under about 2°) |

### B. Motion
| # | Test | How | Report |
|---|---|---|---|
| B1 | One cell | `f` ×10 down a walled corridor, measured with tape | cm error per move, ms per move, wall touches |
| B2 | Turns | `r` ×4, `l` ×4, `u` ×2 | Heading error after each set, ms per turn |
| B3 | Square | `r`, `f` around a 2×2 square of cells, ×2 laps | Distance from the start mark at the end |
| B4 | Front align | Stop short of a wall, then `a` | Final gap to the wall |

The B1 and B2 timings go into `T_MOVE` / `T_TURN90` / `T_TURN180` in `simulation/arena_sim.h`.

### C. Line following
| # | Test | How | Report |
|---|---|---|---|
| C1 | Straight line | White board with a 3 cm black line, `L` | Tracks? Wobble? (Gains in `config.h` are untuned) |
| C2 | Bend | A 45–90° bend | Holds the line through the bend? |
| C3 | Exit detection | A line board ending onto a black floor | "bridge exit reached" on the black floor, not earlier or later? |
| C4 | Ramp | Mock ramp under 35° | Climbs? Stalls? Slips on the way down? |

### D. Exploration (main firmware)
| # | Test | Layout | Report |
|---|---|---|---|
| D1 | Start scan | Robot in one cell | Two `WALLS` lines and one 90° turn |
| D2 | Section A | A 4×4 (or 2×2) mock-up with a gap at a corner for the bridge | Log, a photo, and the printed Section A bitmasks. Does it stop on the bridge tile facing the gap? |
| D3 | Full run | Full arena | Log and video |

## Report template
```
Test: B1   Date / battery voltage:
Setup: (photo if layout matters)
Result: numbers, pass/fail
Odd behaviour:
Log: tools/serial_log.txt section
```

## Interface notes for the work in progress
- **DIP switches:** suggested pins 38, 40, 42, 44 (INPUT_PULLUP, switch to GND). Pins 2, 3, 5–8, 18–21, 22–35 and 49 are in use.
- **Bridge detection:** the explorer needs `readFloorTile()` to return `TILE_BRIDGE_MARKER` when the robot is stopped on the marker tile. Sample the IR array inside the forward move, latch the marker, and report it at the next `readFloorTile()`.
- **Speed-run planner:** input is `ExplorationResult` in `lib/MazeExplorer/maze_types.h`.
  - Section A's NORTH is the robot's heading on START; Section B's NORTH is its heading when it enters B.
  - On speed runs the robot may start facing a different way, so first match the START tile's walls against `mazeA` to find its heading.
