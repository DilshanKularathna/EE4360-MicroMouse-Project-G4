#ifndef MAZE_EXPLORER_H
#define MAZE_EXPLORER_H

#include "maze_types.h"
#include "robot_hardware_interface.h"

/**
 * ============================================================================
 * Exploration State Machine States
 * ============================================================================
 */
enum ExplorationState : uint8_t {
    STATE_INIT = 0,
    STATE_EXPLORE_SECTION_A,
    STATE_APPROACH_BRIDGE,
    STATE_CROSSING_BRIDGE,
    STATE_ENTER_SECTION_B,
    STATE_EXPLORE_SECTION_B,
    STATE_COMPLETED,
    STATE_FAILED
};

/**
 * ============================================================================
 * Stack for DFS Backtracking
 * ============================================================================
 */
#define MAX_STACK_SIZE 100

class CoordStack {
private:
    CellCoord items[MAX_STACK_SIZE];
    int8_t topIndex;

public:
    CoordStack() : topIndex(-1) {}

    void clear() { topIndex = -1; }
    bool isEmpty() const { return topIndex < 0; }
    bool isFull() const { return topIndex >= (MAX_STACK_SIZE - 1); }

    bool push(CellCoord c) {
        if (isFull()) return false;
        items[++topIndex] = c;
        return true;
    }

    bool pop(CellCoord& out) {
        if (isEmpty()) return false;
        out = items[topIndex--];
        return true;
    }

    bool peek(CellCoord& out) const {
        if (isEmpty()) return false;
        out = items[topIndex];
        return true;
    }
};

/**
 * ============================================================================
 * MazeExplorer Class
 * ----------------------------------------------------------------------------
 * Handles Phase 1 Exploration:
 * 1. Explores Section A (4x4) from unknown start (a, b) and relative heading.
 * 2. Detects bridge approach marker (9x9cm white + 3x3cm black center).
 * 3. Hands off control to pluggable ILineFollower to cross bridge.
 * 4. Enters Section B (9x9) and explores to discover the white goal tile.
 * 5. Returns normalized ExplorationResult ready for Phase 2 Speed Run.
 * ============================================================================
 */
class MazeExplorer {
private:
    IRobotPlatform&  robot;
    ILineFollower*   lineFollower;

    ExplorationState state;
    Direction        currentHeading;

    // Working grid for Section A (centered 7x7 to absorb unknown start offset)
    #define GRID_A_WORK_SIZE 7
    uint8_t workGridA[GRID_A_WORK_SIZE][GRID_A_WORK_SIZE];
    int8_t  currA_x, currA_y;
    int8_t  minA_x, maxA_x, minA_y, maxA_y;
    CellCoord bridgeRampA; // Cell where bridge ramp is located

    // Working grid for Section B (9x9)
    uint8_t gridB[MAZE_B_SIZE][MAZE_B_SIZE];
    int8_t  currB_x, currB_y;
    CellCoord bridgeExitB;
    CellCoord goalB;
    bool      goalFound;

    CoordStack dfsStack;
    ExplorationResult finalResult;

    // Internal navigation helpers
    void senseAndMapCurrentCell(bool inSectionA);
    void updateSymmetricWall(int8_t x, int8_t y, Direction dir, bool inSectionA);
    bool hasWall(int8_t x, int8_t y, Direction dir, bool inSectionA) const;
    bool isVisited(int8_t x, int8_t y, bool inSectionA) const;
    void getNeighborCoord(int8_t x, int8_t y, Direction dir, int8_t& nx, int8_t& ny) const;
    bool isWithinBounds(int8_t x, int8_t y, bool inSectionA) const;
    
    void turnToHeading(Direction targetHeading);
    Direction getTurnDirection(int8_t fromX, int8_t fromY, int8_t toX, int8_t toY) const;
    
    bool stepDFS(bool inSectionA);
    void normalizeSectionA();

public:
    MazeExplorer(IRobotPlatform& platform, ILineFollower* follower = nullptr);

    void setLineFollower(ILineFollower* follower) { lineFollower = follower; }

    void begin();
    
    /**
     * Executes the full Phase 1 exploration sequence.
     * Blocks until completion or failure.
     * @return ExplorationResult containing both mapped mazes and coordinates.
     */
    ExplorationResult runExploration();

    /**
     * Non-blocking single step execution (for use in Arduino loop() if desired).
     * @return true if exploration is still in progress, false if finished.
     */
    bool loopStep();

    ExplorationState getState() const { return state; }
    const ExplorationResult& getResult() const { return finalResult; }
};

#endif // MAZE_EXPLORER_H
