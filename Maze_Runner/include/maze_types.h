#ifndef MAZE_TYPES_H
#define MAZE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/**
 * ============================================================================
 * Direction Representation
 * ----------------------------------------------------------------------------
 * NOTE ON HEADING:
 * The robot starts at an unknown orientation in the real world.
 * "DIR_NORTH" is the LOCAL RELATIVE REFERENCE FRAME defined at t=0.
 * Whichever direction the robot faces when placed at the start tile is DIR_NORTH.
 * All wall detection, coordinate updates, and turns are computed relative to this frame:
 *   DIR_NORTH (0): Robot initial forward
 *   DIR_EAST  (1): +90° turn clockwise (Right)
 *   DIR_SOUTH (2): +180° turn (U-turn)
 *   DIR_WEST  (3): +270° / -90° turn counter-clockwise (Left)
 * ============================================================================
 */
enum Direction : uint8_t {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
};

// Bitmask flags for walls and cell state
#define WALL_NORTH    (1 << 0) // 0x01
#define WALL_EAST     (1 << 1) // 0x02
#define WALL_SOUTH    (1 << 2) // 0x04
#define WALL_WEST     (1 << 3) // 0x08
#define CELL_VISITED  (1 << 4) // 0x10

// Dimensions according to EE4360 2026 Challenge Specification
#define MAZE_A_SIZE   4  // Section A: 4x4 cells
#define MAZE_B_SIZE   9  // Section B: 9x9 cells

// Floor tile types detected by IR array / sensors
enum TileType : uint8_t {
    TILE_NORMAL_BLACK = 0,
    TILE_WHITE,           // Full white tile: START in Sec A or FINISH in Sec B
    TILE_BRIDGE_MARKER    // 9cm x 9cm white square with 3cm x 3cm black center
};

// Coordinate representation
struct CellCoord {
    int8_t x;
    int8_t y;

    bool equals(int8_t ox, int8_t oy) const {
        return (x == ox && y == oy);
    }
};

/**
 * ============================================================================
 * Exploration Result (Phase 1 Output -> Phase 2 Input)
 * ----------------------------------------------------------------------------
 * Contains the complete mapped environment across both sections,
 * bridge connection coordinates, and goal location.
 * Phase 2 speed run solver consumes this struct directly.
 * ============================================================================
 */
struct ExplorationResult {
    uint8_t   mazeA[MAZE_A_SIZE][MAZE_A_SIZE]; // Section A wall bitmasks (4x4)
    uint8_t   mazeB[MAZE_B_SIZE][MAZE_B_SIZE]; // Section B wall bitmasks (9x9)
    CellCoord startA;                          // Resolved start tile in mazeA
    CellCoord bridgeEntryA;                    // Bridge ramp tile in mazeA
    CellCoord bridgeExitB;                     // Bridge ramp / entrance in mazeB
    CellCoord goalB;                           // Full-white goal tile in mazeB
    bool      goalFound;                       // True if goal discovered
    bool      completed;                       // True if full exploration finished
};

#endif // MAZE_TYPES_H
