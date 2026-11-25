#include "maze_explorer.h"

MazeExplorer::MazeExplorer(IRobotPlatform& platform, ILineFollower* follower)
    : robot(platform),
      lineFollower(follower),
      state(STATE_INIT),
      currentHeading(DIR_NORTH),
      currA_x(3), currA_y(3),
      minA_x(3), maxA_x(3), minA_y(3), maxA_y(3),
      bridgeRampA({0, 0}),
      currB_x(0), currB_y(0),
      bridgeExitB({0, 0}),
      goalB({0, 0}),
      goalFound(false)
{
    finalResult.startA = {0, 0};
    finalResult.bridgeEntryA = {0, 0};
    finalResult.bridgeExitB = {0, 0};
    finalResult.goalB = {0, 0};
    finalResult.goalFound = false;
    finalResult.completed = false;
}

void MazeExplorer::begin() {
    // Clear Section A working grid (7x7 centered at 3,3)
    for (int y = 0; y < GRID_A_WORK_SIZE; y++) {
        for (int x = 0; x < GRID_A_WORK_SIZE; x++) {
            workGridA[y][x] = 0;
        }
    }

    // Clear Section B grid (9x9)
    for (int y = 0; y < MAZE_B_SIZE; y++) {
        for (int x = 0; x < MAZE_B_SIZE; x++) {
            gridB[y][x] = 0;
            finalResult.mazeB[y][x] = 0;
        }
    }

    for (int y = 0; y < MAZE_A_SIZE; y++) {
        for (int x = 0; x < MAZE_A_SIZE; x++) {
            finalResult.mazeA[y][x] = 0;
        }
    }

    // Initialize relative origin for Section A
    currA_x = 3;
    currA_y = 3;
    minA_x = maxA_x = 3;
    minA_y = maxA_y = 3;

    // Robot starts facing arbitrary physical direction -> defined as DIR_NORTH
    currentHeading = DIR_NORTH;
    dfsStack.clear();

    goalFound = false;
    finalResult.goalFound = false;
    finalResult.completed = false;

    robot.begin();
    state = STATE_EXPLORE_SECTION_A;
}

void MazeExplorer::getNeighborCoord(int8_t x, int8_t y, Direction dir, int8_t& nx, int8_t& ny) const {
    nx = x;
    ny = y;
    switch (dir) {
        case DIR_NORTH: ny = y + 1; break;
        case DIR_EAST:  nx = x + 1; break;
        case DIR_SOUTH: ny = y - 1; break;
        case DIR_WEST:  nx = x - 1; break;
    }
}

bool MazeExplorer::isWithinBounds(int8_t x, int8_t y, bool inSectionA) const {
    if (inSectionA) {
        // Must stay inside 7x7 working buffer
        if (x < 0 || x >= GRID_A_WORK_SIZE || y < 0 || y >= GRID_A_WORK_SIZE) {
            return false;
        }
        // Section A is strictly 4x4. Check if adding (x, y) exceeds 4x4 bounding box
        int8_t tMinX = (x < minA_x) ? x : minA_x;
        int8_t tMaxX = (x > maxA_x) ? x : maxA_x;
        int8_t tMinY = (y < minA_y) ? y : minA_y;
        int8_t tMaxY = (y > maxA_y) ? y : maxA_y;

        if ((tMaxX - tMinX + 1) > MAZE_A_SIZE || (tMaxY - tMinY + 1) > MAZE_A_SIZE) {
            return false; // Beyond 4x4 physical arena perimeter
        }
        return true;
    } else {
        // Section B is strictly 9x9 (0..8)
        return (x >= 0 && x < MAZE_B_SIZE && y >= 0 && y < MAZE_B_SIZE);
    }
}

bool MazeExplorer::hasWall(int8_t x, int8_t y, Direction dir, bool inSectionA) const {
    uint8_t cellValue = inSectionA ? workGridA[y][x] : gridB[y][x];
    uint8_t mask = (1 << dir);
    return (cellValue & mask) != 0;
}

bool MazeExplorer::isVisited(int8_t x, int8_t y, bool inSectionA) const {
    uint8_t cellValue = inSectionA ? workGridA[y][x] : gridB[y][x];
    return (cellValue & CELL_VISITED) != 0;
}

void MazeExplorer::updateSymmetricWall(int8_t x, int8_t y, Direction dir, bool inSectionA) {
    int8_t nx, ny;
    getNeighborCoord(x, y, dir, nx, ny);

    if (inSectionA) {
        if (nx >= 0 && nx < GRID_A_WORK_SIZE && ny >= 0 && ny < GRID_A_WORK_SIZE) {
            Direction opposite = (Direction)((dir + 2) % 4);
            workGridA[ny][nx] |= (1 << opposite);
        }
    } else {
        if (nx >= 0 && nx < MAZE_B_SIZE && ny >= 0 && ny < MAZE_B_SIZE) {
            Direction opposite = (Direction)((dir + 2) % 4);
            gridB[ny][nx] |= (1 << opposite);
        }
    }
}

void MazeExplorer::senseAndMapCurrentCell(bool inSectionA) {
    int8_t cx = inSectionA ? currA_x : currB_x;
    int8_t cy = inSectionA ? currA_y : currB_y;

    // Mark current cell as visited
    if (inSectionA) {
        workGridA[cy][cx] |= CELL_VISITED;
        if (cx < minA_x) minA_x = cx;
        if (cx > maxA_x) maxA_x = cx;
        if (cy < minA_y) minA_y = cy;
        if (cy > maxA_y) maxA_y = cy;
    } else {
        gridB[cy][cx] |= CELL_VISITED;
    }

    // Read wall sensors
    WallSensors ws = robot.readWalls();

    // Convert robot-relative sensor readings to grid directions
    Direction frontDir = currentHeading;
    Direction rightDir = (Direction)((currentHeading + 1) % 4);
    Direction leftDir  = (Direction)((currentHeading + 3) % 4);

    if (ws.wallFront) {
        if (inSectionA) workGridA[cy][cx] |= (1 << frontDir);
        else gridB[cy][cx] |= (1 << frontDir);
        updateSymmetricWall(cx, cy, frontDir, inSectionA);
    }
    if (ws.wallRight) {
        if (inSectionA) workGridA[cy][cx] |= (1 << rightDir);
        else gridB[cy][cx] |= (1 << rightDir);
        updateSymmetricWall(cx, cy, rightDir, inSectionA);
    }
    if (ws.wallLeft) {
        if (inSectionA) workGridA[cy][cx] |= (1 << leftDir);
        else gridB[cy][cx] |= (1 << leftDir);
        updateSymmetricWall(cx, cy, leftDir, inSectionA);
    }

    // If a neighbor would exceed 4x4 bounds in Section A, mark boundary wall
    if (inSectionA) {
        for (uint8_t d = 0; d < 4; d++) {
            Direction dir = (Direction)d;
            int8_t nx, ny;
            getNeighborCoord(cx, cy, dir, nx, ny);
            if (!isWithinBounds(nx, ny, true)) {
                workGridA[cy][cx] |= (1 << dir);
            }
        }
    }
}

void MazeExplorer::turnToHeading(Direction targetHeading) {
    int diff = (int)targetHeading - (int)currentHeading;
    if (diff == 1 || diff == -3) {
        robot.turn(1);  // Right turn 90°
    } else if (diff == -1 || diff == 3) {
        robot.turn(-1); // Left turn 90°
    } else if (diff == 2 || diff == -2) {
        robot.turn(2);  // 180° turn
    }
    currentHeading = targetHeading;
}

Direction MazeExplorer::getTurnDirection(int8_t fromX, int8_t fromY, int8_t toX, int8_t toY) const {
    if (toY > fromY) return DIR_NORTH;
    if (toX > fromX) return DIR_EAST;
    if (toY < fromY) return DIR_SOUTH;
    return DIR_WEST;
}

bool MazeExplorer::stepDFS(bool inSectionA) {
    int8_t cx = inSectionA ? currA_x : currB_x;
    int8_t cy = inSectionA ? currA_y : currB_y;

    // Check open, unvisited neighbors in preference: Straight -> Right -> Left -> Back
    const Direction checkOrder[4] = {
        currentHeading,
        (Direction)((currentHeading + 1) % 4),
        (Direction)((currentHeading + 3) % 4),
        (Direction)((currentHeading + 2) % 4)
    };

    for (uint8_t i = 0; i < 4; i++) {
        Direction dir = checkOrder[i];
        if (!hasWall(cx, cy, dir, inSectionA)) {
            int8_t nx, ny;
            getNeighborCoord(cx, cy, dir, nx, ny);

            if (isWithinBounds(nx, ny, inSectionA) && !isVisited(nx, ny, inSectionA)) {
                // Push current position to backtrack stack
                dfsStack.push({cx, cy});

                // Orient and drive
                turnToHeading(dir);
                if (robot.moveForwardOneCell()) {
                    if (inSectionA) {
                        currA_x = nx;
                        currA_y = ny;
                    } else {
                        currB_x = nx;
                        currB_y = ny;
                    }
                    return true;
                } else {
                    // Physical collision or stall -> restore wall flag and remain
                    if (inSectionA) workGridA[cy][cx] |= (1 << dir);
                    else gridB[cy][cx] |= (1 << dir);
                    return false;
                }
            }
        }
    }

    // No unvisited open neighbor found -> Backtrack
    if (dfsStack.isEmpty()) {
        // Backtracked all the way to start of this section
        return false;
    }

    CellCoord prev;
    dfsStack.pop(prev);

    Direction backDir = getTurnDirection(cx, cy, prev.x, prev.y);
    turnToHeading(backDir);
    if (robot.moveForwardOneCell()) {
        if (inSectionA) {
            currA_x = prev.x;
            currA_y = prev.y;
        } else {
            currB_x = prev.x;
            currB_y = prev.y;
        }
        return true;
    }

    return false;
}

void MazeExplorer::normalizeSectionA() {
    int8_t offsetX = minA_x;
    int8_t offsetY = minA_y;

    // Shift working grid into canonical 4x4 array
    for (int y = 0; y < MAZE_A_SIZE; y++) {
        for (int x = 0; x < MAZE_A_SIZE; x++) {
            int8_t wx = x + offsetX;
            int8_t wy = y + offsetY;
            if (wx >= 0 && wx < GRID_A_WORK_SIZE && wy >= 0 && wy < GRID_A_WORK_SIZE) {
                finalResult.mazeA[y][x] = workGridA[wy][wx];
            } else {
                finalResult.mazeA[y][x] = 0x0F; // Solid wall if outside
            }
        }
    }

    // Canonical coordinates of start tile (started at 3, 3)
    finalResult.startA.x = 3 - offsetX;
    finalResult.startA.y = 3 - offsetY;

    // Canonical coordinates of bridge ramp tile
    finalResult.bridgeEntryA.x = bridgeRampA.x - offsetX;
    finalResult.bridgeEntryA.y = bridgeRampA.y - offsetY;
}

bool MazeExplorer::loopStep() {
    switch (state) {
        case STATE_INIT:
            begin();
            return true;

        case STATE_EXPLORE_SECTION_A: {
            senseAndMapCurrentCell(true);

            // Check floor tile
            TileType tile = robot.readFloorTile();
            if (tile == TILE_BRIDGE_MARKER) {
                // Detected 9cm white square with 3cm black center!
                // The tile immediately ahead is the bridge ramp
                int8_t rampX, rampY;
                getNeighborCoord(currA_x, currA_y, currentHeading, rampX, rampY);
                bridgeRampA.x = rampX;
                bridgeRampA.y = rampY;

                state = STATE_APPROACH_BRIDGE;
                return true;
            }

            // Step DFS exploration
            bool canContinue = stepDFS(true);
            if (!canContinue) {
                // Section A fully mapped without finding bridge marker on unvisited cells
                // (e.g., if marker was already sighted or reachable set exhausted)
                state = STATE_APPROACH_BRIDGE;
            }
            return true;
        }

        case STATE_APPROACH_BRIDGE: {
            // Drive forward 1 cell onto the bridge ramp tile
            robot.moveForwardOneCell();
            normalizeSectionA();
            state = STATE_CROSSING_BRIDGE;
            return true;
        }

        case STATE_CROSSING_BRIDGE: {
            // Hand control over to pluggable line follower
            if (lineFollower != nullptr) {
                lineFollower->begin();
                while (!lineFollower->step()) {
                    // Running line follower loop across bridge
                }
                lineFollower->stop();
            }
            state = STATE_ENTER_SECTION_B;
            return true;
        }

        case STATE_ENTER_SECTION_B: {
            // Drive forward 1 cell off the bridge corridor into Section B
            robot.moveForwardOneCell();
            currB_x = 0;
            currB_y = 0;
            currentHeading = DIR_NORTH;
            bridgeExitB = {0, 0};
            finalResult.bridgeExitB = bridgeExitB;

            dfsStack.clear();
            state = STATE_EXPLORE_SECTION_B;
            return true;
        }

        case STATE_EXPLORE_SECTION_B: {
            senseAndMapCurrentCell(false);

            // Check if current tile is the white goal tile
            TileType tile = robot.readFloorTile();
            if (tile == TILE_WHITE) {
                goalB = {currB_x, currB_y};
                goalFound = true;
                finalResult.goalB = goalB;
                finalResult.goalFound = true;
                finalResult.completed = true;

                // Copy mapped mazeB into final result
                for (int y = 0; y < MAZE_B_SIZE; y++) {
                    for (int x = 0; x < MAZE_B_SIZE; x++) {
                        finalResult.mazeB[y][x] = gridB[y][x];
                    }
                }

                robot.stop();
                state = STATE_COMPLETED;
                return false; // Done!
            }

            // Continue DFS in Section B
            bool canContinue = stepDFS(false);
            if (!canContinue) {
                // Explored all reachable cells in Section B
                for (int y = 0; y < MAZE_B_SIZE; y++) {
                    for (int x = 0; x < MAZE_B_SIZE; x++) {
                        finalResult.mazeB[y][x] = gridB[y][x];
                    }
                }
                finalResult.completed = goalFound;
                robot.stop();
                state = goalFound ? STATE_COMPLETED : STATE_FAILED;
                return false;
            }
            return true;
        }

        case STATE_COMPLETED:
        case STATE_FAILED:
        default:
            robot.stop();
            return false;
    }
}

ExplorationResult MazeExplorer::runExploration() {
    begin();
    while (loopStep()) {
        // Running state machine
    }
    return finalResult;
}
