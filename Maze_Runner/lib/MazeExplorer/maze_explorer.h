#ifndef MAZE_EXPLORER_H
#define MAZE_EXPLORER_H

#include "maze_types.h"
#include "robot_hardware_interface.h"
#include "section_map.h"

/**
 * ============================================================================
 * Exploration State Machine States
 * ============================================================================
 */
enum ExplorationState : uint8_t {
    STATE_INIT = 0,
    STATE_EXPLORE_SECTION_A,   // map every reachable cell of Section A
    STATE_APPROACH_BRIDGE,     // drive to the bridge-out tile and face the bridge
    STATE_CROSSING_BRIDGE,     // line follower has control
    STATE_ENTER_SECTION_B,     // start a new map on the bridge-in tile
    STATE_EXPLORE_SECTION_B,   // find FINISH, then prove the shortest path
    STATE_COMPLETED,
    STATE_FAILED
};

/**
 * ============================================================================
 * MazeExplorer - Phase 1 exploration (Section A -> bridge -> Section B)
 * ----------------------------------------------------------------------------
 * 1. START tile: scan the walls twice (turning 90 deg in between) so all four
 *    sides are known. Timing only starts when the robot leaves the tile.
 * 2. Section A: visit every reachable cell, always going to the nearest
 *    unexplored cell. The bridge-out tile is a corner; its opening is found from
 *    the approach marker + bridge tile, or from the gap in the outer wall.
 * 3. Drive to the bridge-out tile, face the opening, hand over to the line
 *    follower.
 * 4. Section B: new map starting on the bridge-in tile (a corner, opening
 *    behind). Explore until FINISH is found, then keep exploring only until
 *    the shortest bridge-in -> FINISH path is proven (no unknown cells on it).
 * 5. Results are normalised to 4x4 / 9x9 arrays for the Phase 2 speed run.
 * ============================================================================
 */
class MazeExplorer {
public:
    MazeExplorer(IRobotPlatform& platform, ILineFollower* follower = nullptr);

    void setLineFollower(ILineFollower* follower) { lineFollower = follower; }

    void begin();

    /**
     * Executes the full Phase 1 exploration sequence.
     * Blocks until completion or failure.
     */
    ExplorationResult runExploration();

    /**
     * Non-blocking single step (one cell move or one state change).
     * @return true while exploration is still in progress.
     */
    bool loopStep();

    ExplorationState getState() const { return state; }
    const ExplorationResult& getResult() const { return finalResult; }

private:
    static const uint16_t MAX_CELLS = SectionMap::MAX_WORK * SectionMap::MAX_WORK;
    static const uint8_t  NO_DIR = 0xFF;

    enum SearchMode : uint8_t {
        SEARCH_FRONTIER,     // nearest unvisited cell that may be inside the section
        SEARCH_PATH_CELLS,   // nearest unvisited cell on the optimistic shortest path
        SEARCH_TARGET        // a given visited cell
    };

    IRobotPlatform& robot;
    ILineFollower*  lineFollower;

    ExplorationState state;
    Direction        heading;
    int8_t           posX, posY;
    SectionMap       map;

    // Section A
    CellCoord startCell;
    CellCoord markerA;     bool markerAKnown;
    CellCoord bridgeTile;  bool bridgeTileKnown;
    Direction openingDir;  bool openingKnown;

    // Section B
    CellCoord bridgeInB;
    CellCoord goalCell;    bool goalFound;

    ExplorationResult finalResult;

    // Search buffers (shared by all searches)
    uint8_t  parentDir[MAX_CELLS];
    uint16_t queue[MAX_CELLS];
    uint8_t  onPath[(MAX_CELLS + 7) / 8];
    CellCoord searchTarget;

    // Sensing / moving
    void scanWalls();
    void arriveAtCell(bool inSectionA);
    bool moveToward(Direction d);
    void turnToHeading(Direction target);

    // Search
    static uint16_t idx(int8_t x, int8_t y) { return (uint16_t)y * SectionMap::MAX_WORK + x; }
    bool isTarget(SearchMode mode, int8_t fromX, int8_t fromY, int8_t x, int8_t y) const;
    uint8_t firstStep(SearchMode mode);
    bool markOptimisticPath();

    // Section A bridge
    void tryResolveOpening();
    void setOpening(int8_t x, int8_t y, Direction d);
    bool identifyBridge();
    bool probeOpening();

    // Results
    void normalizeSectionA();
    void normalizeSectionB();

    bool stepExploreA();
    bool stepApproachBridge();
    bool stepCrossBridge();
    bool stepEnterSectionB();
    bool stepExploreB();
};

#endif // MAZE_EXPLORER_H
