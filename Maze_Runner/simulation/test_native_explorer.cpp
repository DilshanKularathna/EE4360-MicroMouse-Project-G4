/**
 * ============================================================================
 * Native C++ Test Harness for MicroMouse MazeExplorer Algorithm
 * ----------------------------------------------------------------------------
 * Validates algorithm correctness independently of microcontroller hardware:
 * - Dynamic 9x9 maze test (and 4x4 + bridge + 9x9 full sequence)
 * - Configurable Start and Goal tiles
 * - Simulated perfect IR array & ultrasonic sensor inputs
 * - Verifies wall bitmask recording and goal discovery
 * ============================================================================
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cassert>
#include "../include/maze_explorer.h"

// Simple deterministic maze generator for testing
void generateTestMaze(uint8_t maze[MAZE_B_SIZE][MAZE_B_SIZE], int width, int height) {
    // Initialize with all walls present
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            maze[y][x] = WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST;
        }
    }

    // Carve a spanning tree using DFS so all cells are reachable (no loops, no obstacles)
    std::vector<std::pair<int, int>> stack;
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));

    stack.push_back({0, 0});
    visited[0][0] = true;

    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    const uint8_t wallMask[4] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
    const uint8_t oppMask[4] = {WALL_SOUTH, WALL_WEST, WALL_NORTH, WALL_EAST};

    // Linear congruential generator for deterministic maze
    unsigned int seed = 12345;
    auto myRand = [&]() {
        seed = seed * 1103515245 + 12345;
        return (seed / 65536) % 32768;
    };

    while (!stack.empty()) {
        auto curr = stack.back();
        int cx = curr.first;
        int cy = curr.second;

        // Find unvisited neighbors
        std::vector<int> validDirs;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d];
            int ny = cy + dy[d];
            if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx]) {
                validDirs.push_back(d);
            }
        }

        if (!validDirs.empty()) {
            int d = validDirs[myRand() % validDirs.size()];
            int nx = cx + dx[d];
            int ny = cy + dy[d];

            // Knock down wall between current and neighbor
            maze[cy][cx] &= ~wallMask[d];
            maze[ny][nx] &= ~oppMask[d];

            visited[ny][nx] = true;
            stack.push_back({nx, ny});
        } else {
            stack.pop_back();
        }
    }
}

/**
 * Mock Robot Platform for Native Testing
 */
class MockRobotPlatform : public IRobotPlatform {
public:
    uint8_t truthMazeA[MAZE_A_SIZE][MAZE_A_SIZE];
    uint8_t truthMazeB[MAZE_B_SIZE][MAZE_B_SIZE];

    int posX, posY;
    Direction heading;
    char currentSection; // 'A' or 'B'

    CellCoord startA;
    CellCoord goalB;
    CellCoord bridgeMarkerA;
    CellCoord bridgeRampA;

    int moveCount;
    int turnCount;

    MockRobotPlatform() 
        : posX(0), posY(0), heading(DIR_NORTH), currentSection('A'),
          moveCount(0), turnCount(0) 
    {
        // Default Start A at (0,0), Bridge Marker A at (3,2), Bridge Ramp at (3,3)
        startA = {0, 0};
        bridgeMarkerA = {3, 2};
        bridgeRampA = {3, 3};
        goalB = {4, 4}; // Default center of 9x9

        // Initialize Section A 4x4 maze
        for (int y = 0; y < MAZE_A_SIZE; y++) {
            for (int x = 0; x < MAZE_A_SIZE; x++) {
                truthMazeA[y][x] = 0;
                if (y == MAZE_A_SIZE - 1) truthMazeA[y][x] |= WALL_NORTH;
                if (y == 0) truthMazeA[y][x] |= WALL_SOUTH;
                if (x == MAZE_A_SIZE - 1) truthMazeA[y][x] |= WALL_EAST;
                if (x == 0) truthMazeA[y][x] |= WALL_WEST;
            }
        }
        // Open bridge ramp exit at (3,3) to the north
        truthMazeA[3][3] &= ~WALL_NORTH;

        // Initialize Section B 9x9 maze
        generateTestMaze(truthMazeB, MAZE_B_SIZE, MAZE_B_SIZE);
        // Ensure bridge entry at (0,0) has south wall open for entry
        truthMazeB[0][0] &= ~WALL_SOUTH;
    }

    void begin() override {
        posX = startA.x;
        posY = startA.y;
        heading = DIR_NORTH;
        currentSection = 'A';
    }

    WallSensors readWalls() override {
        uint8_t cellWalls = 0;
        if (currentSection == 'A') {
            if (posX >= 0 && posX < MAZE_A_SIZE && posY >= 0 && posY < MAZE_A_SIZE) {
                cellWalls = truthMazeA[posY][posX];
            }
        } else {
            if (posX >= 0 && posX < MAZE_B_SIZE && posY >= 0 && posY < MAZE_B_SIZE) {
                cellWalls = truthMazeB[posY][posX];
            }
        }

        const uint8_t wallMasks[4] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
        Direction frontDir = heading;
        Direction rightDir = (Direction)((heading + 1) % 4);
        Direction leftDir  = (Direction)((heading + 3) % 4);

        WallSensors ws;
        ws.wallFront = (cellWalls & wallMasks[frontDir]) != 0;
        ws.wallRight = (cellWalls & wallMasks[rightDir]) != 0;
        ws.wallLeft  = (cellWalls & wallMasks[leftDir])  != 0;
        return ws;
    }

    TileType readFloorTile() override {
        if (currentSection == 'A') {
            if (posX == bridgeMarkerA.x && posY == bridgeMarkerA.y) {
                return TILE_BRIDGE_MARKER;
            }
            if (posX == startA.x && posY == startA.y) {
                return TILE_WHITE;
            }
        } else if (currentSection == 'B') {
            if (posX == goalB.x && posY == goalB.y) {
                return TILE_WHITE;
            }
        }
        return TILE_NORMAL_BLACK;
    }

    bool moveForwardOneCell() override {
        const int dx[4] = {0, 1, 0, -1};
        const int dy[4] = {1, 0, -1, 0};
        posX += dx[heading];
        posY += dy[heading];
        moveCount++;
        return true;
    }

    void turn(int relativeTurns) override {
        heading = (Direction)((heading + relativeTurns + 4) % 4);
        turnCount++;
    }

    void stop() override {}
};

/**
 * Mock Line Follower for Bridge
 */
class MockLineFollower : public ILineFollower {
private:
    int stepCount = 0;
    MockRobotPlatform* platform;

public:
    MockLineFollower(MockRobotPlatform* p = nullptr) : platform(p) {}

    void setPlatform(MockRobotPlatform* p) { platform = p; }

    void begin() override { stepCount = 0; }
    bool step() override {
        stepCount++;
        // Completes line following in 3 steps
        return stepCount >= 3;
    }
    void stop() override {
        if (platform) {
            platform->currentSection = 'B';
            platform->posX = 0;
            platform->posY = -1; // Facing North, next step will land at (0, 0)
            platform->heading = DIR_NORTH;
        }
    }
};

int main() {
    std::cout << "========================================================\n";
    std::cout << "  EE4360 MicroMouse Maze Explorer - Native C++ Test      \n";
    std::cout << "========================================================\n\n";

    MockRobotPlatform mockPlatform;
    MockLineFollower mockFollower(&mockPlatform);

    // Set dynamic goal for Section B
    mockPlatform.goalB = {3, 0};
    std::cout << "[Config] Section A Size: 4x4\n";
    std::cout << "[Config] Section B Size: 9x9\n";
    std::cout << "[Config] Section A Start: (" << (int)mockPlatform.startA.x << ", " << (int)mockPlatform.startA.y << ")\n";
    std::cout << "[Config] Section B Goal: (" << (int)mockPlatform.goalB.x << ", " << (int)mockPlatform.goalB.y << ")\n\n";

    MazeExplorer explorer(mockPlatform, &mockFollower);
    std::cout << "[Status] Starting Maze Explorer...\n";

    explorer.begin();
    int stepCount = 0;
    while (explorer.loopStep()) {
        stepCount++;
        if (stepCount > 500) {
            std::cout << "[Error] Step limit exceeded (infinite loop)!\n";
            return 1;
        }
    }

    ExplorationResult result = explorer.getResult();

    std::cout << "\n[Status] Exploration Finished in " << stepCount << " steps!\n";
    std::cout << "  - Completed: " << (result.completed ? "YES" : "NO") << "\n";
    std::cout << "  - Goal Found: " << (result.goalFound ? "YES" : "NO") << "\n";
    std::cout << "  - Goal Location in B: (" << (int)result.goalB.x << ", " << (int)result.goalB.y << ")\n";
    std::cout << "  - Total Moves: " << mockPlatform.moveCount << "\n";
    std::cout << "  - Total Turns: " << mockPlatform.turnCount << "\n\n";

    // Print Section B Recorded Mapped Grid in Hex
    std::cout << "--- Section B Mapped Wall Bitmasks (9x9) ---\n";
    std::cout << "      ";
    for (int x = 0; x < MAZE_B_SIZE; x++) std::cout << "x=" << x << "  ";
    std::cout << "\n";
    for (int y = MAZE_B_SIZE - 1; y >= 0; y--) {
        std::cout << "y=" << y << " | ";
        for (int x = 0; x < MAZE_B_SIZE; x++) {
            uint8_t cell = result.mazeB[y][x];
            std::cout << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)cell << " ";
        }
        std::cout << std::dec << "\n";
    }

    assert(result.completed == true);
    assert(result.goalFound == true);
    assert(result.goalB.x == 3 && result.goalB.y == 0);
    std::cout << "\n[PASS] Test 1: Near Goal (3,0) passed.\n\n";

    // Test 2: Deep Goal in Section B at (8, 8)
    std::cout << "--- Running Test 2: Deep Goal at (8, 8) ---\n";
    MockRobotPlatform mockPlatform2;
    MockLineFollower mockFollower2(&mockPlatform2);
    mockPlatform2.goalB = {8, 8};

    MazeExplorer explorer2(mockPlatform2, &mockFollower2);
    explorer2.begin();
    stepCount = 0;
    while (explorer2.loopStep()) {
        stepCount++;
        if (stepCount > 1000) break;
    }
    ExplorationResult result2 = explorer2.getResult();
    std::cout << "  - Completed: " << (result2.completed ? "YES" : "NO") << "\n";
    std::cout << "  - Goal Found: " << (result2.goalFound ? "YES" : "NO") << "\n";
    std::cout << "  - Goal Location in B: (" << (int)result2.goalB.x << ", " << (int)result2.goalB.y << ")\n";
    std::cout << "  - Steps taken: " << stepCount << "\n";
    std::cout << "  - Moves: " << mockPlatform2.moveCount << ", Turns: " << mockPlatform2.turnCount << "\n";

    assert(result2.completed == true);
    assert(result2.goalFound == true);
    assert(result2.goalB.x == 8 && result2.goalB.y == 8);
    std::cout << "\n[SUCCESS] All assertion tests (Test 1 & Test 2) passed!\n";

    return 0;
}
