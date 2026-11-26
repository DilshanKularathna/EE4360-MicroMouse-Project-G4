#include "maze_explorer.h"

MazeExplorer::MazeExplorer(IRobotPlatform& platform, ILineFollower* follower)
    : robot(platform),
      lineFollower(follower),
      state(STATE_INIT),
      heading(DIR_NORTH),
      posX(0), posY(0),
      startCell({0, 0}),
      markerA({0, 0}), markerAKnown(false),
      bridgeTile({0, 0}), bridgeTileKnown(false),
      openingDir(DIR_NORTH), openingKnown(false),
      bridgeInB({0, 0}),
      goalCell({0, 0}), goalFound(false),
      searchTarget({0, 0})
{
    finalResult = ExplorationResult();
}

// ============================================================================
// Setup
// ============================================================================

void MazeExplorer::begin() {
    finalResult = ExplorationResult();
    finalResult.bridgeExitDirA = DIR_NORTH;

    map.reset(MAZE_A_SIZE);
    posX = posY = map.origin();
    heading = DIR_NORTH;              // the robot's heading on START defines NORTH
    startCell = {posX, posY};
    markerAKnown = bridgeTileKnown = openingKnown = false;
    goalFound = false;

    robot.begin();

    // Scan the START tile from two headings so all four sides are known (the
    // robot has no rear sensor). Turning on START is free: the race clock only
    // starts when the robot leaves the tile.
    scanWalls();
    turnToHeading(DIR_EAST);
    scanWalls();
    map.markVisited(posX, posY);

    state = STATE_EXPLORE_SECTION_A;
}

ExplorationResult MazeExplorer::runExploration() {
    begin();
    while (loopStep()) {
    }
    return finalResult;
}

bool MazeExplorer::loopStep() {
    switch (state) {
        case STATE_INIT:              begin(); return true;
        case STATE_EXPLORE_SECTION_A: return stepExploreA();
        case STATE_APPROACH_BRIDGE:   return stepApproachBridge();
        case STATE_CROSSING_BRIDGE:   return stepCrossBridge();
        case STATE_ENTER_SECTION_B:   return stepEnterSectionB();
        case STATE_EXPLORE_SECTION_B: return stepExploreB();
        case STATE_COMPLETED:
        case STATE_FAILED:
        default:
            robot.stop();
            return false;
    }
}

// ============================================================================
// Sensing and moving
// ============================================================================

void MazeExplorer::scanWalls() {
    WallSensors ws = robot.readWalls();
    if (ws.wallFront) map.setWall(posX, posY, heading);
    if (ws.wallRight) map.setWall(posX, posY, (Direction)((heading + 1) & 3));
    if (ws.wallLeft)  map.setWall(posX, posY, (Direction)((heading + 3) & 3));
}

void MazeExplorer::arriveAtCell(bool inSectionA) {
    if (!map.isVisited(posX, posY)) {
        scanWalls();                       // the side behind is open: we came through it
        map.markVisited(posX, posY);
    }

    // The floor is read on every visit, so one missed reading is not final.
    TileType tile = robot.readFloorTile();
    if (inSectionA) {
        if (tile == TILE_BRIDGE_MARKER) {
            markerA = {posX, posY};
            markerAKnown = true;
            tryResolveOpening();
        } else if (tile == TILE_WHITE && !startCell.equals(posX, posY)) {
            // White but not START: the bridge-out tile (white with a black line)
            bridgeTile = {posX, posY};
            bridgeTileKnown = true;
            map.requireCorner(posX, posY);
            tryResolveOpening();
        }
    } else if (tile == TILE_WHITE && !bridgeInB.equals(posX, posY) && !goalFound) {
        goalCell = {posX, posY};
        goalFound = true;
    }
}

// Turns and drives one cell. A failed move means there is a wall after all.
bool MazeExplorer::moveToward(Direction d) {
    turnToHeading(d);
    if (robot.moveForwardOneCell()) {
        posX += SectionMap::stepX(d);
        posY += SectionMap::stepY(d);
        return true;
    }
    map.setWall(posX, posY, d);
    return false;
}

void MazeExplorer::turnToHeading(Direction target) {
    int diff = ((int)target - (int)heading + 4) & 3;
    if (diff == 1)      robot.turn(1);
    else if (diff == 3) robot.turn(-1);
    else if (diff == 2) robot.turn(2);
    heading = target;
}

// ============================================================================
// Search (breadth-first over the known map)
// ============================================================================

bool MazeExplorer::isTarget(SearchMode mode, int8_t fromX, int8_t fromY, int8_t x, int8_t y) const {
    switch (mode) {
        case SEARCH_FRONTIER:
            if (!map.canBeInside(x, y)) return false;
            // Until the bridge opening is known, only leave the bridge tile
            // towards cells that are certainly inside Section A.
            if (state == STATE_EXPLORE_SECTION_A && bridgeTileKnown && !openingKnown &&
                bridgeTile.equals(fromX, fromY) && !map.surelyInside(x, y))
                return false;
            return true;
        case SEARCH_PATH_CELLS: {
            uint16_t i = idx(x, y);
            return (onPath[i >> 3] & (1 << (i & 7))) != 0;
        }
        case SEARCH_TARGET:
        default:
            return false;
    }
}

// Direction of the first move on the shortest route to the nearest target,
// or NO_DIR if there is none. Routes only use visited cells (all sides known).
uint8_t MazeExplorer::firstStep(SearchMode mode) {
    for (uint16_t i = 0; i < MAX_CELLS; i++) parentDir[i] = NO_DIR;

    const uint16_t start = idx(posX, posY);
    parentDir[start] = 4;   // marks the start
    uint16_t head = 0, tail = 0;
    queue[tail++] = start;

    int8_t   foundX = -1, foundY = -1;
    while (head < tail && foundX < 0) {
        uint16_t cur = queue[head++];
        int8_t cx = (int8_t)(cur % SectionMap::MAX_WORK);
        int8_t cy = (int8_t)(cur / SectionMap::MAX_WORK);

        // Straight on first, then right, left, back: fewer turns on ties
        for (uint8_t k = 0; k < 4 && foundX < 0; k++) {
            static const uint8_t ORDER[4] = {0, 1, 3, 2};
            Direction d = (Direction)((heading + ORDER[k]) & 3);
            if (map.hasWall(cx, cy, d)) continue;
            int8_t nx = cx + SectionMap::stepX(d), ny = cy + SectionMap::stepY(d);
            if (!map.inWork(nx, ny)) continue;
            uint16_t ni = idx(nx, ny);
            if (parentDir[ni] != NO_DIR) continue;

            if (map.isVisited(nx, ny)) {
                parentDir[ni] = (uint8_t)d;
                if (mode == SEARCH_TARGET && searchTarget.equals(nx, ny)) { foundX = nx; foundY = ny; }
                else queue[tail++] = ni;
            } else if (isTarget(mode, cx, cy, nx, ny)) {
                parentDir[ni] = (uint8_t)d;
                foundX = nx; foundY = ny;
            }
        }
    }
    if (foundX < 0) return NO_DIR;

    // Walk back to the start; the last direction seen is the first move
    int8_t x = foundX, y = foundY;
    uint8_t d = parentDir[idx(x, y)];
    while (true) {
        int8_t px = x - SectionMap::stepX((Direction)d);
        int8_t py = y - SectionMap::stepY((Direction)d);
        if (idx(px, py) == start) return d;
        x = px; y = py;
        d = parentDir[idx(x, y)];
    }
}

// Shortest bridge-in -> FINISH route assuming unknown walls are open. Marks
// its cells in onPath. Returns true if some of them are still unvisited
// (the route is not proven yet).
bool MazeExplorer::markOptimisticPath() {
    for (uint16_t i = 0; i < sizeof(onPath); i++) onPath[i] = 0;
    for (uint16_t i = 0; i < MAX_CELLS; i++) parentDir[i] = NO_DIR;

    const uint16_t start = idx(bridgeInB.x, bridgeInB.y);
    parentDir[start] = 4;
    uint16_t head = 0, tail = 0;
    queue[tail++] = start;
    bool found = false;
    while (head < tail && !found) {
        uint16_t cur = queue[head++];
        int8_t cx = (int8_t)(cur % SectionMap::MAX_WORK);
        int8_t cy = (int8_t)(cur / SectionMap::MAX_WORK);
        for (uint8_t d = 0; d < 4; d++) {
            if (map.hasWall(cx, cy, (Direction)d)) continue;
            int8_t nx = cx + SectionMap::stepX((Direction)d), ny = cy + SectionMap::stepY((Direction)d);
            if (!map.canBeInside(nx, ny)) continue;
            uint16_t ni = idx(nx, ny);
            if (parentDir[ni] != NO_DIR) continue;
            parentDir[ni] = d;
            if (goalCell.equals(nx, ny)) { found = true; break; }
            queue[tail++] = ni;
        }
    }
    if (!found) return false;

    bool unknownOnPath = false;
    int8_t x = goalCell.x, y = goalCell.y;
    while (true) {
        uint16_t i = idx(x, y);
        onPath[i >> 3] |= (uint8_t)(1 << (i & 7));
        if (!map.isVisited(x, y)) unknownOnPath = true;
        if (i == start) break;
        Direction d = (Direction)parentDir[i];
        x -= SectionMap::stepX(d);
        y -= SectionMap::stepY(d);
    }
    return unknownOnPath;
}

// ============================================================================
// Section A bridge
// ============================================================================

void MazeExplorer::setOpening(int8_t x, int8_t y, Direction d) {
    bridgeTile = {x, y};
    bridgeTileKnown = true;
    openingDir = d;
    openingKnown = true;
    markerA = {(int8_t)(x - SectionMap::stepX(d)), (int8_t)(y - SectionMap::stepY(d))};
    markerAKnown = true;
    map.setWall(x, y, d);                 // never drive off the bridge without the line follower
    map.requireCorner(x, y);
    map.requireOuterSide(x, y, d);
}

// Marker tile, bridge tile and opening are in one straight line.
void MazeExplorer::tryResolveOpening() {
    if (openingKnown || !markerAKnown || !bridgeTileKnown) return;
    for (uint8_t k = 0; k < 4; k++) {
        Direction d = (Direction)k;
        if (bridgeTile.equals(markerA.x + SectionMap::stepX(d), markerA.y + SectionMap::stepY(d)) &&
            !map.hasWall(markerA.x, markerA.y, d)) {
            setOpening(bridgeTile.x, bridgeTile.y, d);
            return;
        }
    }
}

// Called when Section A is fully explored and the opening is still unknown.
bool MazeExplorer::identifyBridge() {
    if (openingKnown) return true;

    // 1. An open side of a visited cell that leads outside every possible
    //    placement of the 4x4 section: that gap is the bridge opening.
    int8_t cx = -1, cy = -1; Direction cd = DIR_NORTH; uint8_t count = 0;
    for (int8_t y = 0; y < map.workSize(); y++) {
        for (int8_t x = 0; x < map.workSize(); x++) {
            if (!map.isVisited(x, y)) continue;
            for (uint8_t k = 0; k < 4; k++) {
                Direction d = (Direction)k;
                if (map.hasWall(x, y, d)) continue;
                if (map.canBeInside(x + SectionMap::stepX(d), y + SectionMap::stepY(d))) continue;
                cx = x; cy = y; cd = d; count++;
            }
        }
    }
    if (count == 1) { setOpening(cx, cy, cd); return true; }

    // 2. Bridge tile seen (white) but not the marker: its open side that may lead outside
    if (bridgeTileKnown) {
        count = 0;
        for (uint8_t k = 0; k < 4; k++) {
            Direction d = (Direction)k;
            if (map.hasWall(bridgeTile.x, bridgeTile.y, d)) continue;
            if (map.surelyInside(bridgeTile.x + SectionMap::stepX(d), bridgeTile.y + SectionMap::stepY(d))) continue;
            cd = d; count++;
        }
        if (count == 1) { setOpening(bridgeTile.x, bridgeTile.y, cd); return true; }
    }

    // 3. Marker seen but not the bridge tile: the visited neighbour in line with a possible opening
    if (markerAKnown) {
        count = 0;
        for (uint8_t k = 0; k < 4; k++) {
            Direction d = (Direction)k;
            int8_t bx = markerA.x + SectionMap::stepX(d), by = markerA.y + SectionMap::stepY(d);
            if (map.hasWall(markerA.x, markerA.y, d) || !map.inWork(bx, by) || !map.isVisited(bx, by)) continue;
            if (map.hasWall(bx, by, d)) continue;
            if (map.surelyInside(bx + SectionMap::stepX(d), by + SectionMap::stepY(d))) continue;
            cx = bx; cy = by; cd = d; count++;
        }
        if (count == 1) { setOpening(cx, cy, cd); return true; }
    }
    return false;
}

// ============================================================================
// Results
// ============================================================================

void MazeExplorer::normalizeSectionA() {
    int8_t bx = 0, by = 0;
    map.placement(bx, by);
    for (int8_t y = 0; y < MAZE_A_SIZE; y++)
        for (int8_t x = 0; x < MAZE_A_SIZE; x++)
            finalResult.mazeA[y][x] = map.raw(bx + x, by + y);
    finalResult.startA         = {(int8_t)(startCell.x - bx), (int8_t)(startCell.y - by)};
    finalResult.bridgeEntryA   = {(int8_t)(bridgeTile.x - bx), (int8_t)(bridgeTile.y - by)};
    finalResult.bridgeExitDirA = openingDir;
}

void MazeExplorer::normalizeSectionB() {
    int8_t bx = 0, by = 0;
    map.placement(bx, by);
    for (int8_t y = 0; y < MAZE_B_SIZE; y++)
        for (int8_t x = 0; x < MAZE_B_SIZE; x++)
            finalResult.mazeB[y][x] = map.raw(bx + x, by + y);
    finalResult.bridgeExitB = {(int8_t)(bridgeInB.x - bx), (int8_t)(bridgeInB.y - by)};
    finalResult.goalFound   = goalFound;
    if (goalFound) finalResult.goalB = {(int8_t)(goalCell.x - bx), (int8_t)(goalCell.y - by)};
}

// ============================================================================
// States
// ============================================================================

bool MazeExplorer::stepExploreA() {
    uint8_t d = firstStep(SEARCH_FRONTIER);
    if (d == NO_DIR) {
        // Section A fully explored. If the walls do not settle where the
        // opening is, it is checked with distances on the bridge tile.
        if (!identifyBridge() && !bridgeTileKnown) {
            robot.stop();
            state = STATE_FAILED;
            return false;
        }
        state = STATE_APPROACH_BRIDGE;
        return true;
    }
    if (moveToward((Direction)d)) arriveAtCell(true);
    return true;
}

// On the bridge tile with two open sides that could each be the opening (one
// leads to the marker tile, the other off the maze). Looking along Section A
// the sensor must see a wall within (N-1) cells; through the opening there is
// no maze wall, so that side reads further.
bool MazeExplorer::probeOpening() {
    const float insideMaxCM = (map.sectionSize() - 1) * MAZE_CELL_CM + MAZE_CELL_CM / 2.0f;
    Direction best = DIR_NORTH, beyond = DIR_NORTH;
    float bestCM = -1.0f;
    uint8_t candidates = 0, beyondCount = 0;

    for (uint8_t k = 0; k < 4; k++) {
        Direction d = (Direction)((heading + k) & 3);
        if (map.hasWall(posX, posY, d)) continue;
        if (map.surelyInside(posX + SectionMap::stepX(d), posY + SectionMap::stepY(d))) continue;
        turnToHeading(d);
        float cm = robot.readWalls().frontCM;
        candidates++;
        if (cm > bestCM) { bestCM = cm; best = d; }
        if (cm > insideMaxCM) { beyond = d; beyondCount++; }
    }
    if (candidates == 0) return false;
    // Exactly one side reads past the section: that is the opening.
    // Otherwise take the longer reading (better than giving up the run).
    setOpening(posX, posY, beyondCount == 1 ? beyond : best);
    return true;
}

bool MazeExplorer::stepApproachBridge() {
    if (bridgeTile.equals(posX, posY)) {
        if (!openingKnown && !probeOpening()) {
            robot.stop();
            state = STATE_FAILED;
            return false;
        }
        normalizeSectionA();
        turnToHeading(openingDir);   // on the line, facing the bridge
        state = STATE_CROSSING_BRIDGE;
        return true;
    }
    searchTarget = bridgeTile;
    uint8_t d = firstStep(SEARCH_TARGET);
    if (d == NO_DIR) {
        robot.stop();
        state = STATE_FAILED;
        return false;
    }
    if (moveToward((Direction)d)) arriveAtCell(true);
    return true;
}

bool MazeExplorer::stepCrossBridge() {
    if (lineFollower != nullptr) {
        lineFollower->begin();
        while (!lineFollower->step()) {
        }
        lineFollower->stop();
    }
    state = STATE_ENTER_SECTION_B;
    return true;
}

bool MazeExplorer::stepEnterSectionB() {
    // The line follower leaves the robot on the bridge-in tile facing into B.
    // That heading is NORTH of the Section B map.
    map.reset(MAZE_B_SIZE);
    posX = posY = map.origin();
    heading = DIR_NORTH;
    bridgeInB = {posX, posY};
    map.setWall(posX, posY, DIR_SOUTH);          // the opening behind us
    map.requireCorner(posX, posY);
    map.requireOuterSide(posX, posY, DIR_SOUTH);
    arriveAtCell(false);
    state = STATE_EXPLORE_SECTION_B;
    return true;
}

bool MazeExplorer::stepExploreB() {
    SearchMode mode = SEARCH_FRONTIER;
    bool searching = true;
    if (goalFound) {
        // Keep exploring only while the shortest route still crosses unknown cells
        searching = markOptimisticPath();
        mode = SEARCH_PATH_CELLS;
    }
    uint8_t d = searching ? firstStep(mode) : NO_DIR;
    if (d == NO_DIR) {
        normalizeSectionB();
        finalResult.completed = goalFound;
        robot.stop();
        state = goalFound ? STATE_COMPLETED : STATE_FAILED;
        return false;
    }
    if (moveToward((Direction)d)) arriveAtCell(false);
    return true;
}
