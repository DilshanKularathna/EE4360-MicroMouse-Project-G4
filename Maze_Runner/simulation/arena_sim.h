#ifndef ARENA_SIM_H
#define ARENA_SIM_H

/**
 * ============================================================================
 * Competition-arena simulator for the maze explorer (host / native build only)
 * ----------------------------------------------------------------------------
 * Generates random arenas that follow the EE4360 2026 rules and runs the real
 * MazeExplorer against a simulated robot, then scores the run.
 *
 * Arena rules modelled (Maze_Solver.pdf, "The Field"):
 *  - Section A is 4x4, Section B is 9x9, 25 cm tiles, fully enclosed except at
 *    the bridge opening.
 *  - START is a white tile anywhere in A; the robot may face any direction.
 *  - The bridge leaves from a corner tile of A (bridge-out tile) and enters a
 *    corner tile of B (bridge-in tile). Corners and directions vary.
 *  - The tile just before the bridge-out / bridge-in tile has the approach
 *    marker. Marker tile, bridge tile and wall opening are in a straight line
 *    (Figure 2).
 *  - Bridge tiles have a white background with a black line.
 *  - FINISH is a white tile anywhere in B. B has loops (multiple paths).
 *
 * Interface contract assumed for the hardware adapter (what the real robot
 * code must provide for these results to hold):
 *  - readWalls()   : wall / no wall on the front, left and right of the cell.
 *  - readFloorTile(): what is under the robot at the cell centre. Marker tile
 *                    -> TILE_BRIDGE_MARKER, START/FINISH -> TILE_WHITE, bridge
 *                    tiles -> SimConfig::lineTileReads (default TILE_WHITE,
 *                    because they are mostly white).
 *  - moveForwardOneCell(): returns false (robot does not move) if a wall is ahead.
 *  - ILineFollower: begin() must be called on the bridge-out tile facing the
 *    opening. When it finishes, the robot is on the bridge-in tile facing into B.
 * ============================================================================
 */

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "maze_types.h"
#include "robot_hardware_interface.h"
#include "maze_explorer.h"

namespace sim {

// ---------------------------------------------------------------------------
// Time model (seconds). Rough estimates - replace with values measured on the
// robot (time 10 cell moves, 10 turns, etc.) to make the time check meaningful.
// ---------------------------------------------------------------------------
constexpr double T_SENSE     = 0.45;   // readWalls: 3 sensors x 3 pings + gyro bias refresh
constexpr double T_MOVE      = 1.50;   // one 25 cm cell incl. stop and front-wall alignment
constexpr double T_TURN90    = 1.20;
constexpr double T_TURN180   = 2.00;
constexpr double T_BRIDGE    = 25.0;   // ramp + span + line corridor
constexpr double RUN_LIMIT_S = 480.0;  // 8 minute run limit

constexpr int DX[4] = {0, 1, 0, -1};   // N, E, S, W  (same order as Direction)
constexpr int DY[4] = {1, 0, -1, 0};

inline uint8_t bitOf(int d) { return (uint8_t)(1u << d); }
inline int     opp(int d)   { return (d + 2) & 3; }
inline const char* dirName(int d) {
    static const char* n[4] = {"N", "E", "S", "W"};
    return n[d & 3];
}

struct Cell {
    int x, y;
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Cell& o) const { return !(*this == o); }
};

// Small deterministic RNG (same results on every machine).
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x9E3779B9u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    int      below(int n) { return (int)(next() % (uint32_t)n); }
    double   unit() { return next() / 4294967296.0; }
};

// ---------------------------------------------------------------------------
// Grid of wall bitmasks (WALL_NORTH.. bits, same layout as maze_types.h)
// ---------------------------------------------------------------------------
struct Grid {
    int     n = 0;
    uint8_t w[MAZE_B_SIZE][MAZE_B_SIZE] = {};

    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < n && y < n; }
    bool wall(int x, int y, int d) const { return (w[y][x] & bitOf(d)) != 0; }

    // Sets or clears a wall on both sides.
    void setWall(int x, int y, int d, bool on) {
        if (on) w[y][x] |= bitOf(d); else w[y][x] &= (uint8_t)~bitOf(d);
        int nx = x + DX[d], ny = y + DY[d];
        if (inside(nx, ny)) {
            if (on) w[ny][nx] |= bitOf(opp(d)); else w[ny][nx] &= (uint8_t)~bitOf(opp(d));
        }
    }
};

struct Arena {
    uint32_t seed = 0;
    Grid a, b;
    Cell startA{};      int startHeading = 0;
    Cell bridgeOutA{};  int outDir = 0;     // opening side of the bridge-out tile
    Cell markerA{};
    Cell bridgeInB{};   int inDir = 0;      // heading of a robot entering B
    Cell markerB{};
    Cell goalB{};
};

// Random spanning tree (every cell reachable), then extra openings for loops.
inline void carveMaze(Grid& g, Rng& rng, int extraOpenings) {
    for (int y = 0; y < g.n; y++)
        for (int x = 0; x < g.n; x++)
            g.w[y][x] = WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST;

    std::vector<Cell> stack;
    bool seen[MAZE_B_SIZE][MAZE_B_SIZE] = {};
    Cell s{rng.below(g.n), rng.below(g.n)};
    stack.push_back(s);
    seen[s.y][s.x] = true;
    while (!stack.empty()) {
        Cell c = stack.back();
        int dirs[4], k = 0;
        for (int d = 0; d < 4; d++) {
            int nx = c.x + DX[d], ny = c.y + DY[d];
            if (g.inside(nx, ny) && !seen[ny][nx]) dirs[k++] = d;
        }
        if (k == 0) { stack.pop_back(); continue; }
        int d = dirs[rng.below(k)];
        g.setWall(c.x, c.y, d, false);
        seen[c.y + DY[d]][c.x + DX[d]] = true;
        stack.push_back({c.x + DX[d], c.y + DY[d]});
    }

    for (int tries = 0; extraOpenings > 0 && tries < 1000; tries++) {
        int x = rng.below(g.n), y = rng.below(g.n), d = rng.below(4);
        if (!g.inside(x + DX[d], y + DY[d]) || !g.wall(x, y, d)) continue;
        g.setWall(x, y, d, false);
        extraOpenings--;
    }
}

inline Cell randomCorner(int n, Rng& rng) {
    return Cell{rng.below(2) ? n - 1 : 0, rng.below(2) ? n - 1 : 0};
}

// One of the two outer sides of a corner cell.
inline int randomOuterSide(const Cell& c, int n, Rng& rng) {
    int sides[2] = {c.x == 0 ? (int)DIR_WEST : (int)DIR_EAST,
                    c.y == 0 ? (int)DIR_SOUTH : (int)DIR_NORTH};
    (void)n;
    return sides[rng.below(2)];
}

inline Arena generateArena(uint32_t seed) {
    Rng rng(seed * 2654435761u + 1u);
    Arena ar;
    ar.seed = seed;
    ar.a.n = MAZE_A_SIZE;
    ar.b.n = MAZE_B_SIZE;
    carveMaze(ar.a, rng, 1 + rng.below(2));
    carveMaze(ar.b, rng, 6 + rng.below(7));

    // Section A: bridge-out corner, opening, marker in line with them
    ar.bridgeOutA = randomCorner(MAZE_A_SIZE, rng);
    ar.outDir     = randomOuterSide(ar.bridgeOutA, MAZE_A_SIZE, rng);
    ar.markerA    = {ar.bridgeOutA.x - DX[ar.outDir], ar.bridgeOutA.y - DY[ar.outDir]};
    ar.a.setWall(ar.markerA.x, ar.markerA.y, ar.outDir, false);    // marker -> bridge tile open
    ar.a.w[ar.bridgeOutA.y][ar.bridgeOutA.x] &= (uint8_t)~bitOf(ar.outDir);  // the opening

    // Section B: bridge-in corner, opening behind the entering robot, marker ahead
    ar.bridgeInB = randomCorner(MAZE_B_SIZE, rng);
    int openSide = randomOuterSide(ar.bridgeInB, MAZE_B_SIZE, rng);
    ar.inDir     = opp(openSide);
    ar.markerB   = {ar.bridgeInB.x + DX[ar.inDir], ar.bridgeInB.y + DY[ar.inDir]};
    ar.b.setWall(ar.bridgeInB.x, ar.bridgeInB.y, ar.inDir, false);
    ar.b.w[ar.bridgeInB.y][ar.bridgeInB.x] &= (uint8_t)~bitOf(openSide);

    do {
        ar.startA = {rng.below(MAZE_A_SIZE), rng.below(MAZE_A_SIZE)};
    } while (ar.startA == ar.bridgeOutA || ar.startA == ar.markerA);
    ar.startHeading = rng.below(4);

    do {
        ar.goalB = {rng.below(MAZE_B_SIZE), rng.below(MAZE_B_SIZE)};
    } while (ar.goalB == ar.bridgeInB || ar.goalB == ar.markerB);
    return ar;
}

// Checks one grid against the rules. Returns "" if fine, otherwise the problem.
inline std::string checkGrid(const Grid& g, const Cell& openCell, int openDir) {
    char buf[96];
    for (int y = 0; y < g.n; y++) {
        for (int x = 0; x < g.n; x++) {
            for (int d = 0; d < 4; d++) {
                int nx = x + DX[d], ny = y + DY[d];
                if (g.inside(nx, ny)) {
                    if (g.wall(x, y, d) != g.wall(nx, ny, opp(d))) {
                        snprintf(buf, sizeof buf, "wall (%d,%d)%s not mirrored", x, y, dirName(d));
                        return buf;
                    }
                } else {
                    bool isOpening = (Cell{x, y} == openCell && d == openDir);
                    if (g.wall(x, y, d) == isOpening) {
                        snprintf(buf, sizeof buf, "outer wall wrong at (%d,%d)%s", x, y, dirName(d));
                        return buf;
                    }
                }
            }
        }
    }
    // Every cell reachable
    bool seen[MAZE_B_SIZE][MAZE_B_SIZE] = {};
    std::vector<Cell> q{{0, 0}};
    seen[0][0] = true;
    int count = 0;
    while (!q.empty()) {
        Cell c = q.back(); q.pop_back(); count++;
        for (int d = 0; d < 4; d++) {
            int nx = c.x + DX[d], ny = c.y + DY[d];
            if (g.inside(nx, ny) && !g.wall(c.x, c.y, d) && !seen[ny][nx]) {
                seen[ny][nx] = true;
                q.push_back({nx, ny});
            }
        }
    }
    if (count != g.n * g.n) return "not every cell is reachable";
    return "";
}

inline int openInteriorPassages(const Grid& g) {
    int open = 0;
    for (int y = 0; y < g.n; y++)
        for (int x = 0; x < g.n; x++)
            for (int d = 0; d < 2; d++) {   // N and E only, so each passage counts once
                int nx = x + DX[d], ny = y + DY[d];
                if (g.inside(nx, ny) && !g.wall(x, y, d)) open++;
            }
    return open;
}

// ---------------------------------------------------------------------------
// Simulated robot
// ---------------------------------------------------------------------------
struct SimConfig {
    double   wallNoise     = 0.0;          // chance each wall reading is flipped
    double   floorMiss     = 0.0;          // chance a marker / white tile reads as black
    TileType lineTileReads = TILE_WHITE;   // what a bridge tile (white + black line) reads as
    float    outsideEchoCM = 100.0f;       // reading through the bridge opening (100 = Ultrasonic
                                           // lib "no echo"); lower it to model objects outside
    int      maxSteps      = 20000;        // explorer loop steps before giving up
};

class SimRobot : public IRobotPlatform {
public:
    const Arena& ar;
    SimConfig    cfg;
    Rng          rng;

    char section = 'A';      // 'A' or 'B'
    int  x = 0, y = 0, h = 0;

    int  moves = 0, turns90 = 0, turns180 = 0, wallReads = 0, bridgeCrossings = 0;
    int  blockedMoves = 0, startReentries = 0;
    bool visitedGoal = false;
    double secondsAtGoal = 0.0;
    bool lost = false;       // run cannot continue (off the maze, bad bridge hand-off)
    std::string failure;

    SimRobot(const Arena& arena, const SimConfig& c)
        : ar(arena), cfg(c), rng(arena.seed ^ 0xA5A5A5A5u),
          x(arena.startA.x), y(arena.startA.y), h(arena.startHeading) {}

    const Grid& grid() const { return section == 'A' ? ar.a : ar.b; }

    double seconds() const {
        return moves * T_MOVE + turns90 * T_TURN90 + turns180 * T_TURN180 +
               wallReads * T_SENSE + bridgeCrossings * T_BRIDGE;
    }

    void fail(const std::string& why) {
        if (!lost) failure = why;
        lost = true;
    }

    void begin() override {}

    // Distance a sensor at the cell centre would read looking in direction d:
    // to the first wall, or outsideEchoCM if the ray leaves through the opening.
    float rayCM(int d) const {
        const Grid& g = grid();
        int cx = x, cy = y, cells = 0;
        while (!g.wall(cx, cy, d)) {
            cx += DX[d]; cy += DY[d];
            if (!g.inside(cx, cy)) return cfg.outsideEchoCM;
            cells++;
        }
        float cm = cells * 25.0f + 12.5f - 9.0f;   // sensor ~9 cm ahead of the centre
        return cm > 100.0f ? 100.0f : cm;            // past ~1 m the lib reports 100 (no echo)
    }

    WallSensors readWalls() override {
        wallReads++;
        const Grid& g = grid();
        WallSensors ws;
        ws.wallFront = g.wall(x, y, h);
        ws.wallRight = g.wall(x, y, (h + 1) & 3);
        ws.wallLeft  = g.wall(x, y, (h + 3) & 3);
        ws.frontCM   = rayCM(h);
        ws.rightCM   = rayCM((h + 1) & 3);
        ws.leftCM    = rayCM((h + 3) & 3);
        if (cfg.wallNoise > 0) {
            if (rng.unit() < cfg.wallNoise) ws.wallFront = !ws.wallFront;
            if (rng.unit() < cfg.wallNoise) ws.wallRight = !ws.wallRight;
            if (rng.unit() < cfg.wallNoise) ws.wallLeft  = !ws.wallLeft;
        }
        return ws;
    }

    TileType readFloorTile() override {
        Cell c{x, y};
        TileType t = TILE_NORMAL_BLACK;
        if (section == 'A') {
            if (c == ar.markerA)         t = TILE_BRIDGE_MARKER;
            else if (c == ar.bridgeOutA) t = cfg.lineTileReads;
            else if (c == ar.startA)     t = TILE_WHITE;
        } else {
            if (c == ar.markerB)         t = TILE_BRIDGE_MARKER;
            else if (c == ar.bridgeInB)  t = cfg.lineTileReads;
            else if (c == ar.goalB)      t = TILE_WHITE;
        }
        if (t != TILE_NORMAL_BLACK && cfg.floorMiss > 0 && rng.unit() < cfg.floorMiss)
            t = TILE_NORMAL_BLACK;
        return t;
    }

    bool moveForwardOneCell() override {
        if (lost) return false;
        const Grid& g = grid();
        if (g.wall(x, y, h)) {     // the real robot stops at the wall
            blockedMoves++;
            return false;
        }
        int nx = x + DX[h], ny = y + DY[h];
        if (!g.inside(nx, ny)) {
            char buf[120];
            snprintf(buf, sizeof buf, "drove out of section %c through the bridge opening at (%d,%d)%s "
                     "without the line follower", section, x, y, dirName(h));
            fail(buf);
            return false;
        }
        x = nx; y = ny;
        moves++;
        if (section == 'A' && Cell{x, y} == ar.startA) startReentries++;
        if (section == 'B' && Cell{x, y} == ar.goalB && !visitedGoal) {
            visitedGoal   = true;
            secondsAtGoal = seconds();
        }
        return true;
    }

    void turn(int relativeTurns) override {
        int r = ((relativeTurns % 4) + 4) % 4;
        if (r == 0) return;
        if (r == 2) turns180++; else turns90++;
        h = (h + r) & 3;
    }

    void stop() override {}
};

class SimLineFollower : public ILineFollower {
public:
    SimRobot& robot;
    int  steps = 0;
    bool handoffOk = false;

    explicit SimLineFollower(SimRobot& r) : robot(r) {}

    void begin() override {
        steps = 0;
        handoffOk = false;
        if (robot.lost) return;
        const Arena& ar = robot.ar;
        if (robot.section == 'A' && Cell{robot.x, robot.y} == ar.bridgeOutA && robot.h == ar.outDir) {
            handoffOk = true;
            return;
        }
        char buf[160];
        snprintf(buf, sizeof buf, "line follower started at %c(%d,%d) facing %s; the bridge is at "
                 "A(%d,%d) facing %s", robot.section, robot.x, robot.y, dirName(robot.h),
                 ar.bridgeOutA.x, ar.bridgeOutA.y, dirName(ar.outDir));
        robot.fail(buf);
    }

    bool step() override { return ++steps >= 3; }

    void stop() override {
        if (!handoffOk) return;
        robot.section = 'B';
        robot.x = robot.ar.bridgeInB.x;
        robot.y = robot.ar.bridgeInB.y;
        robot.h = robot.ar.inDir;
        robot.bridgeCrossings++;
        handoffOk = false;
    }
};

// ---------------------------------------------------------------------------
// Map scoring. The explorer's frames are fixed by contract (maze_types.h):
// Section A NORTH = robot heading on START, Section B NORTH = heading when
// entering B. So the explorer's map is aligned with the real maze using the
// rotation and one known cell (START / bridge-in), then compared wall by wall.
// ---------------------------------------------------------------------------
struct MapMatch {
    int visited = 0;        // cells the explorer marked visited
    int mismatches = 0;     // wall bits that differ on visited cells
    int rot = 0, tx = 0, ty = 0;

    // Explorer cell -> real cell
    Cell toTrue(int ex, int ey) const {
        return Cell{ex * DX[(rot + 1) & 3] + ey * DX[rot] + tx,
                    ex * DY[(rot + 1) & 3] + ey * DY[rot] + ty};
    }
};

// rot: real direction of the explorer's NORTH. anchorE (explorer) is anchorT (real).
template <int N>
MapMatch matchMap(const uint8_t (&m)[N][N], const Grid& truth, int rot, Cell anchorE, Cell anchorT,
                  Cell ignoreCell, int ignoreDir) {
    MapMatch mm;
    mm.rot = rot;
    Cell z = MapMatch{0, 0, rot, 0, 0}.toTrue(anchorE.x, anchorE.y);
    mm.tx = anchorT.x - z.x;
    mm.ty = anchorT.y - z.y;
    for (int y = 0; y < N; y++) {
        for (int x = 0; x < N; x++) {
            if (!(m[y][x] & CELL_VISITED)) continue;
            mm.visited++;
            Cell t = mm.toTrue(x, y);
            if (!truth.inside(t.x, t.y)) { mm.mismatches += 4; continue; }
            for (int d = 0; d < 4; d++) {
                int td = (d + rot) & 3;
                if (t == ignoreCell && td == ignoreDir) continue;
                bool e = (m[y][x] & bitOf(d)) != 0;
                if (e != truth.wall(t.x, t.y, td)) mm.mismatches++;
            }
        }
    }
    return mm;
}

// ---------------------------------------------------------------------------
// Shortest paths (in cells). The speed run can only use cells the explorer
// visited, so the explorer's map must hold a route as short as the real one.
// ---------------------------------------------------------------------------
inline int trueDistance(const Grid& g, Cell from, Cell to) {
    int dist[MAZE_B_SIZE][MAZE_B_SIZE];
    for (auto& row : dist) for (int& v : row) v = -1;
    std::vector<Cell> q{from};
    dist[from.y][from.x] = 0;
    for (size_t i = 0; i < q.size(); i++) {
        Cell c = q[i];
        if (c == to) return dist[c.y][c.x];
        for (int d = 0; d < 4; d++) {
            int nx = c.x + DX[d], ny = c.y + DY[d];
            if (!g.inside(nx, ny) || g.wall(c.x, c.y, d) || dist[ny][nx] >= 0) continue;
            dist[ny][nx] = dist[c.y][c.x] + 1;
            q.push_back({nx, ny});
        }
    }
    return -1;
}

// Same, on the explorer's map, through visited cells only.
template <int N>
int knownDistance(const uint8_t (&m)[N][N], Cell from, Cell to) {
    auto ok = [&](int x, int y) { return x >= 0 && y >= 0 && x < N && y < N && (m[y][x] & CELL_VISITED); };
    if (!ok(from.x, from.y) || !ok(to.x, to.y)) return -1;
    int dist[N][N];
    for (auto& row : dist) for (int& v : row) v = -1;
    std::vector<Cell> q{from};
    dist[from.y][from.x] = 0;
    for (size_t i = 0; i < q.size(); i++) {
        Cell c = q[i];
        if (c == to) return dist[c.y][c.x];
        for (int d = 0; d < 4; d++) {
            int nx = c.x + DX[d], ny = c.y + DY[d];
            if (!ok(nx, ny) || (m[c.y][c.x] & bitOf(d)) || dist[ny][nx] >= 0) continue;
            dist[ny][nx] = dist[c.y][c.x] + 1;
            q.push_back({nx, ny});
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------
// One full run
// ---------------------------------------------------------------------------
struct RunReport {
    uint32_t seed = 0;
    bool   crossedBridge = false;
    bool   reachedFinish = false;     // robot physically entered the FINISH tile
    bool   reportedGoal = false;      // explorer says it found the goal
    bool   falseFinish = false;       // explorer reported the goal somewhere else
    bool   stepLimit = false;
    int    blockedMoves = 0, startReentries = 0;
    int    moves = 0, turns90 = 0, turns180 = 0;
    double secondsToFinish = 0.0;     // estimated, from START to first entering FINISH
    double secondsTotal = 0.0;
    MapMatch mapA, mapB;
    bool   bridgeEntryCorrect = false;
    bool   startCorrect = false;
    bool   shortestA = false;         // explorer's map holds a shortest START -> bridge-out route
    bool   shortestB = false;         // explorer's map holds a shortest bridge-in -> FINISH route
    ExplorationState finalState = STATE_INIT;
    std::string failure;              // first reason the run failed, "" if it reached FINISH

    bool overTime() const { return reachedFinish && secondsToFinish > RUN_LIMIT_S; }
};

inline const char* stateName(ExplorationState s) {
    switch (s) {
        case STATE_INIT:              return "INIT";
        case STATE_EXPLORE_SECTION_A: return "EXPLORE_A";
        case STATE_APPROACH_BRIDGE:   return "APPROACH_BRIDGE";
        case STATE_CROSSING_BRIDGE:   return "CROSSING_BRIDGE";
        case STATE_ENTER_SECTION_B:   return "ENTER_B";
        case STATE_EXPLORE_SECTION_B: return "EXPLORE_B";
        case STATE_COMPLETED:         return "COMPLETED";
        case STATE_FAILED:            return "FAILED";
    }
    return "?";
}

inline RunReport runArena(const Arena& ar, const SimConfig& cfg = SimConfig()) {
    SimRobot        robot(ar, cfg);
    SimLineFollower follower(robot);
    MazeExplorer    explorer(robot, &follower);

    RunReport rep;
    rep.seed = ar.seed;

    explorer.begin();
    int steps = 0;
    while (explorer.loopStep()) {
        if (robot.lost) break;
        if (++steps >= cfg.maxSteps) { rep.stepLimit = true; break; }
    }

    const ExplorationResult& res = explorer.getResult();
    rep.finalState      = explorer.getState();
    rep.crossedBridge   = robot.bridgeCrossings > 0;
    rep.reachedFinish   = robot.visitedGoal;
    rep.reportedGoal    = res.goalFound;
    rep.blockedMoves    = robot.blockedMoves;
    rep.startReentries  = robot.startReentries;
    rep.moves           = robot.moves;
    rep.turns90         = robot.turns90;
    rep.turns180        = robot.turns180;
    rep.secondsToFinish = robot.secondsAtGoal;
    rep.secondsTotal    = robot.seconds();

    // Section A: anchored on the reported START, so a wrong START shows up as wrong walls
    rep.mapA = matchMap(res.mazeA, ar.a, ar.startHeading, Cell{res.startA.x, res.startA.y}, ar.startA,
                        ar.bridgeOutA, ar.outDir);
    rep.mapB = matchMap(res.mazeB, ar.b, ar.inDir, Cell{res.bridgeExitB.x, res.bridgeExitB.y}, ar.bridgeInB,
                        ar.bridgeInB, opp(ar.inDir));
    if (rep.mapA.visited > 0) {
        rep.bridgeEntryCorrect = rep.mapA.toTrue(res.bridgeEntryA.x, res.bridgeEntryA.y) == ar.bridgeOutA &&
                                 ((res.bridgeExitDirA + ar.startHeading) & 3) == ar.outDir;
        rep.startCorrect       = rep.mapA.mismatches == 0;
    }
    // The goal the explorer reports, located in the real maze
    rep.falseFinish = res.goalFound &&
                      (rep.mapB.visited == 0 || rep.mapB.toTrue(res.goalB.x, res.goalB.y) != ar.goalB);

    // Proven shortest routes (only meaningful when the map itself is right)
    if (rep.bridgeEntryCorrect && rep.mapA.mismatches == 0) {
        int known = knownDistance(res.mazeA, Cell{res.startA.x, res.startA.y},
                                  Cell{res.bridgeEntryA.x, res.bridgeEntryA.y});
        rep.shortestA = known >= 0 && known == trueDistance(ar.a, ar.startA, ar.bridgeOutA);
    }
    if (res.goalFound && !rep.falseFinish && rep.mapB.mismatches == 0) {
        int known = knownDistance(res.mazeB, Cell{res.bridgeExitB.x, res.bridgeExitB.y},
                                  Cell{res.goalB.x, res.goalB.y});
        rep.shortestB = known >= 0 && known == trueDistance(ar.b, ar.bridgeInB, ar.goalB);
    }

    if (robot.lost)              rep.failure = robot.failure;
    else if (rep.stepLimit)      rep.failure = "explorer still running after the step limit";
    else if (!rep.crossedBridge) rep.failure = std::string("never crossed the bridge (ended in ") +
                                               stateName(rep.finalState) + ")";
    else if (!rep.reachedFinish) rep.failure = std::string("never entered the FINISH tile (ended in ") +
                                               stateName(rep.finalState) + ")";
    else if (rep.falseFinish)    rep.failure = "reported the goal on the wrong tile";
    return rep;
}

// ---------------------------------------------------------------------------
// ASCII drawing of an arena (for the runner's --seed view)
// ---------------------------------------------------------------------------
inline void printGrid(const Grid& g, const char* title,
                      const std::vector<std::pair<Cell, char>>& marks) {
    printf("%s\n", title);
    for (int y = g.n - 1; y >= 0; y--) {
        std::string top = "+", mid = "";
        for (int x = 0; x < g.n; x++) {
            top += g.wall(x, y, DIR_NORTH) ? "---+" : "   +";
            mid += g.wall(x, y, DIR_WEST) ? "|" : " ";
            char c = ' ';
            for (const auto& m : marks) if (m.first == Cell{x, y}) c = m.second;
            mid += " "; mid += c; mid += " ";
        }
        mid += g.wall(g.n - 1, y, DIR_EAST) ? "|" : " ";
        printf("  %s\n  %s   y=%d\n", top.c_str(), mid.c_str(), y);
    }
    std::string bot = "+";
    for (int x = 0; x < g.n; x++) bot += g.wall(x, 0, DIR_SOUTH) ? "---+" : "   +";
    printf("  %s\n", bot.c_str());
}

inline void printArena(const Arena& ar) {
    printf("Seed %u\n", ar.seed);
    printf("  Start A(%d,%d) facing %s | bridge-out A(%d,%d) exits %s | marker A(%d,%d)\n",
           ar.startA.x, ar.startA.y, dirName(ar.startHeading), ar.bridgeOutA.x, ar.bridgeOutA.y,
           dirName(ar.outDir), ar.markerA.x, ar.markerA.y);
    printf("  Bridge-in B(%d,%d) entering %s | marker B(%d,%d) | finish B(%d,%d)\n",
           ar.bridgeInB.x, ar.bridgeInB.y, dirName(ar.inDir), ar.markerB.x, ar.markerB.y,
           ar.goalB.x, ar.goalB.y);
    printGrid(ar.a, "Section A  (S start, M marker, B bridge-out)",
              {{ar.startA, 'S'}, {ar.markerA, 'M'}, {ar.bridgeOutA, 'B'}});
    printGrid(ar.b, "Section B  (B bridge-in, M marker, G finish)",
              {{ar.bridgeInB, 'B'}, {ar.markerB, 'M'}, {ar.goalB, 'G'}});
}

}  // namespace sim

#endif  // ARENA_SIM_H
