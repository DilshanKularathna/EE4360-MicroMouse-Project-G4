/**
 * ============================================================================
 * Competition-spec tests for the maze explorer (runs on the PC)
 *
 *   pio test -e native
 *
 * 1. Simulator tests: the arena generator follows the competition rules and
 *    the simulated robot senses and moves correctly.
 * 2. Explorer acceptance tests: the real MazeExplorer must succeed on every
 *    randomly generated arena. These are the targets for the explorer work.
 * ============================================================================
 */

#include <unity.h>
#include <cstdio>
#include <string>

#include "../../simulation/arena_sim.h"

using namespace sim;

static const int ARENAS = 300;   // arenas per explorer test

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Simulator: arena generator
// ---------------------------------------------------------------------------

void test_generated_arenas_follow_the_rules() {
    for (uint32_t s = 1; s <= 1000; s++) {
        Arena ar = generateArena(s);
        char msg[160];

        std::string e = checkGrid(ar.a, ar.bridgeOutA, ar.outDir);
        snprintf(msg, sizeof msg, "seed %u, section A: %s", s, e.c_str());
        TEST_ASSERT_TRUE_MESSAGE(e.empty(), msg);

        e = checkGrid(ar.b, ar.bridgeInB, opp(ar.inDir));
        snprintf(msg, sizeof msg, "seed %u, section B: %s", s, e.c_str());
        TEST_ASSERT_TRUE_MESSAGE(e.empty(), msg);

        snprintf(msg, sizeof msg, "seed %u: bridge tiles must be corners", s);
        TEST_ASSERT_TRUE_MESSAGE((ar.bridgeOutA.x == 0 || ar.bridgeOutA.x == 3) &&
                                 (ar.bridgeOutA.y == 0 || ar.bridgeOutA.y == 3), msg);
        TEST_ASSERT_TRUE_MESSAGE((ar.bridgeInB.x == 0 || ar.bridgeInB.x == 8) &&
                                 (ar.bridgeInB.y == 0 || ar.bridgeInB.y == 8), msg);

        snprintf(msg, sizeof msg, "seed %u: marker, bridge tile and opening must be in line", s);
        TEST_ASSERT_TRUE_MESSAGE(ar.markerA.x + DX[ar.outDir] == ar.bridgeOutA.x &&
                                 ar.markerA.y + DY[ar.outDir] == ar.bridgeOutA.y, msg);
        TEST_ASSERT_FALSE_MESSAGE(ar.a.wall(ar.markerA.x, ar.markerA.y, ar.outDir), msg);
        TEST_ASSERT_TRUE_MESSAGE(ar.bridgeInB.x + DX[ar.inDir] == ar.markerB.x &&
                                 ar.bridgeInB.y + DY[ar.inDir] == ar.markerB.y, msg);
        TEST_ASSERT_FALSE_MESSAGE(ar.b.wall(ar.bridgeInB.x, ar.bridgeInB.y, ar.inDir), msg);

        snprintf(msg, sizeof msg, "seed %u: start / finish placed on a bridge or marker tile", s);
        TEST_ASSERT_TRUE_MESSAGE(ar.startA != ar.bridgeOutA && ar.startA != ar.markerA, msg);
        TEST_ASSERT_TRUE_MESSAGE(ar.goalB != ar.bridgeInB && ar.goalB != ar.markerB, msg);

        // Rules: "multiple paths to the finish" -> B must have loops
        snprintf(msg, sizeof msg, "seed %u: section B has no loops", s);
        TEST_ASSERT_TRUE_MESSAGE(openInteriorPassages(ar.b) > 81 - 1, msg);
    }
}

void test_generator_covers_all_corners_and_directions() {
    bool outCornerDir[4][4] = {};   // [corner][dir]
    bool inCorner[4] = {}, heading[4] = {};
    for (uint32_t s = 1; s <= 1000; s++) {
        Arena ar = generateArena(s);
        int ca = (ar.bridgeOutA.x ? 1 : 0) + (ar.bridgeOutA.y ? 2 : 0);
        int cb = (ar.bridgeInB.x ? 1 : 0) + (ar.bridgeInB.y ? 2 : 0);
        outCornerDir[ca][ar.outDir] = true;
        inCorner[cb] = true;
        heading[ar.startHeading] = true;
    }
    int combos = 0;
    for (int c = 0; c < 4; c++)
        for (int d = 0; d < 4; d++) combos += outCornerDir[c][d];
    TEST_ASSERT_EQUAL_INT_MESSAGE(8, combos, "each A corner should exit through both outer sides");
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(inCorner[i]);
        TEST_ASSERT_TRUE(heading[i]);
    }
}

// ---------------------------------------------------------------------------
// Simulator: robot
// ---------------------------------------------------------------------------

void test_sim_robot_reads_walls_relative_to_heading() {
    Arena ar = generateArena(7);
    SimRobot r(ar, SimConfig());
    for (int h = 0; h < 4; h++) {
        r.h = h;
        WallSensors ws = r.readWalls();
        TEST_ASSERT_EQUAL(ar.a.wall(r.x, r.y, h), ws.wallFront);
        TEST_ASSERT_EQUAL(ar.a.wall(r.x, r.y, (h + 1) & 3), ws.wallRight);
        TEST_ASSERT_EQUAL(ar.a.wall(r.x, r.y, (h + 3) & 3), ws.wallLeft);
    }
}

void test_sim_robot_cannot_drive_through_walls() {
    Arena ar = generateArena(3);
    SimRobot r(ar, SimConfig());
    for (int h = 0; h < 4; h++) {
        r.x = ar.startA.x; r.y = ar.startA.y; r.h = h;
        bool wall = ar.a.wall(r.x, r.y, h);
        bool outside = !ar.a.inside(r.x + DX[h], r.y + DY[h]);
        if (outside) continue;   // only the bridge opening leads outside, tested below
        TEST_ASSERT_EQUAL(!wall, r.moveForwardOneCell());
    }
}

void test_sim_bridge_needs_the_line_follower_from_the_right_tile() {
    Arena ar = generateArena(11);

    // Correct hand-off: on the bridge-out tile facing the opening
    SimRobot ok(ar, SimConfig());
    SimLineFollower f1(ok);
    ok.x = ar.bridgeOutA.x; ok.y = ar.bridgeOutA.y; ok.h = ar.outDir;
    f1.begin();
    while (!f1.step()) {}
    f1.stop();
    TEST_ASSERT_FALSE(ok.lost);
    TEST_ASSERT_EQUAL_CHAR('B', ok.section);
    TEST_ASSERT_EQUAL_INT(ar.bridgeInB.x, ok.x);
    TEST_ASSERT_EQUAL_INT(ar.bridgeInB.y, ok.y);
    TEST_ASSERT_EQUAL_INT(ar.inDir, ok.h);

    // Wrong hand-off: facing the wrong way
    SimRobot bad(ar, SimConfig());
    SimLineFollower f2(bad);
    bad.x = ar.bridgeOutA.x; bad.y = ar.bridgeOutA.y; bad.h = opp(ar.outDir);
    f2.begin();
    TEST_ASSERT_TRUE(bad.lost);

    // Driving out of the opening without the line follower is a failure
    SimRobot off(ar, SimConfig());
    off.x = ar.bridgeOutA.x; off.y = ar.bridgeOutA.y; off.h = ar.outDir;
    TEST_ASSERT_FALSE(off.moveForwardOneCell());
    TEST_ASSERT_TRUE(off.lost);
}

void test_map_matcher_scores_rotated_maps() {
    Arena ar = generateArena(5);
    // The map an explorer would make if its NORTH were the real EAST
    uint8_t m[MAZE_A_SIZE][MAZE_A_SIZE] = {};
    const int rot = 1;
    MapMatch frame{0, 0, rot, 0, 3};   // rot 1 maps (ex,ey) -> (ey, 3 - ex)
    for (int ey = 0; ey < 4; ey++) {
        for (int ex = 0; ex < 4; ex++) {
            Cell t = frame.toTrue(ex, ey);
            uint8_t v = CELL_VISITED;
            for (int d = 0; d < 4; d++)
                if (ar.a.wall(t.x, t.y, (d + rot) & 3)) v |= bitOf(d);
            m[ey][ex] = v;
        }
    }
    // Anchor: explorer (0,0) is real (0,3)
    MapMatch mm = matchMap(m, ar.a, rot, Cell{0, 0}, Cell{0, 3}, Cell{-1, -1}, -1);
    TEST_ASSERT_EQUAL_INT(16, mm.visited);
    TEST_ASSERT_EQUAL_INT(0, mm.mismatches);

    // A wrong anchor must show up as mismatches
    MapMatch bad = matchMap(m, ar.a, rot, Cell{0, 0}, Cell{1, 3}, Cell{-1, -1}, -1);
    TEST_ASSERT_TRUE(bad.mismatches > 0);
}

void test_sim_distance_sees_through_the_opening() {
    Arena ar = generateArena(9);
    SimRobot r(ar, SimConfig());
    r.x = ar.bridgeOutA.x; r.y = ar.bridgeOutA.y; r.h = ar.outDir;
    TEST_ASSERT_TRUE(r.readWalls().frontCM >= 100.0f);
    // Any ray that stays inside the 4x4 ends at a wall within 3 cells
    for (int d = 0; d < 4; d++) {
        if (d == ar.outDir) continue;
        TEST_ASSERT_TRUE(r.rayCM(d) < 3 * 25.0f + 12.5f);
    }
}

// ---------------------------------------------------------------------------
// Explorer acceptance tests (perfect sensors)
// ---------------------------------------------------------------------------

struct Batch {
    int crossed = 0, finished = 0, falseFinish = 0, overTime = 0, blocked = 0, shortA = 0, shortB = 0;
    std::string firstBridgeFail, firstFinishFail, firstFalse, firstBlocked, firstSlow, firstA, firstB;
};

static const Batch& perfectBatch() {
    static Batch b;
    static bool done = false;
    if (done) return b;
    done = true;
    char buf[300];
    for (int i = 1; i <= ARENAS; i++) {
        RunReport r = runArena(generateArena((uint32_t)i));
        snprintf(buf, sizeof buf, "e.g. seed %u: %s", r.seed,
                 r.failure.empty() ? "-" : r.failure.c_str());
        b.crossed  += r.crossedBridge;
        b.finished += r.reachedFinish;
        if (!r.crossedBridge && b.firstBridgeFail.empty()) b.firstBridgeFail = buf;
        if (!r.reachedFinish && b.firstFinishFail.empty()) b.firstFinishFail = buf;
        if (r.falseFinish) { b.falseFinish++; if (b.firstFalse.empty()) b.firstFalse = buf; }
        if (r.blockedMoves) { b.blocked++; if (b.firstBlocked.empty()) b.firstBlocked = buf; }
        if (r.overTime()) {
            b.overTime++;
            if (b.firstSlow.empty()) {
                snprintf(buf, sizeof buf, "e.g. seed %u: %.0f s", r.seed, r.secondsToFinish);
                b.firstSlow = buf;
            }
        }
        b.shortA += r.shortestA;
        b.shortB += r.shortestB;
        if (!r.shortestA && b.firstA.empty()) {
            snprintf(buf, sizeof buf, "e.g. seed %u: %d wrong walls, bridge tile %s", r.seed,
                     r.mapA.mismatches, r.bridgeEntryCorrect ? "ok" : "wrong");
            b.firstA = buf;
        }
        if (!r.shortestB && b.firstB.empty()) {
            snprintf(buf, sizeof buf, "e.g. seed %u: %d/81 visited, %d wrong walls", r.seed,
                     r.mapB.visited, r.mapB.mismatches);
            b.firstB = buf;
        }
    }
    printf("\n[spec] %d arenas: crossed bridge %d, reached finish %d, shortest A %d, shortest B %d, "
           "false finish %d, blocked %d, over time %d\n",
           ARENAS, b.crossed, b.finished, b.shortA, b.shortB, b.falseFinish, b.blocked, b.overTime);
    return b;
}

static void expectAll(int got, const std::string& example, const char* what) {
    char msg[400];
    snprintf(msg, sizeof msg, "%s in %d of %d arenas; %s", what, got, ARENAS, example.c_str());
    TEST_ASSERT_EQUAL_INT_MESSAGE(ARENAS, got, msg);
}

static void expectNone(int got, const std::string& example, const char* what) {
    char msg[400];
    snprintf(msg, sizeof msg, "%s in %d of %d arenas; %s", what, got, ARENAS, example.c_str());
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, got, msg);
}

void test_explorer_crosses_the_bridge_in_every_arena() {
    const Batch& b = perfectBatch();
    expectAll(b.crossed, b.firstBridgeFail, "crossed the bridge");
}

void test_explorer_proves_shortest_route_to_the_bridge() {
    const Batch& b = perfectBatch();
    expectAll(b.shortA, b.firstA, "map holds a shortest START -> bridge route");
}

void test_explorer_proves_shortest_route_to_the_finish() {
    const Batch& b = perfectBatch();
    expectAll(b.shortB, b.firstB, "map holds a shortest bridge -> FINISH route");
}

void test_explorer_reaches_the_finish_in_every_arena() {
    const Batch& b = perfectBatch();
    expectAll(b.finished, b.firstFinishFail, "reached the finish");
}

void test_explorer_never_reports_a_false_finish() {
    const Batch& b = perfectBatch();
    expectNone(b.falseFinish, b.firstFalse, "goal reported on the wrong tile");
}

void test_explorer_never_drives_into_a_known_wall() {
    const Batch& b = perfectBatch();
    expectNone(b.blocked, b.firstBlocked, "tried to drive into a wall");
}

void test_explorer_finishes_within_8_minutes() {
    const Batch& b = perfectBatch();
    expectNone(b.overTime, b.firstSlow, "over the 8 minute limit");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_generated_arenas_follow_the_rules);
    RUN_TEST(test_generator_covers_all_corners_and_directions);
    RUN_TEST(test_sim_robot_reads_walls_relative_to_heading);
    RUN_TEST(test_sim_robot_cannot_drive_through_walls);
    RUN_TEST(test_sim_bridge_needs_the_line_follower_from_the_right_tile);
    RUN_TEST(test_map_matcher_scores_rotated_maps);
    RUN_TEST(test_sim_distance_sees_through_the_opening);
    RUN_TEST(test_explorer_crosses_the_bridge_in_every_arena);
    RUN_TEST(test_explorer_proves_shortest_route_to_the_bridge);
    RUN_TEST(test_explorer_proves_shortest_route_to_the_finish);
    RUN_TEST(test_explorer_reaches_the_finish_in_every_arena);
    RUN_TEST(test_explorer_never_reports_a_false_finish);
    RUN_TEST(test_explorer_never_drives_into_a_known_wall);
    RUN_TEST(test_explorer_finishes_within_8_minutes);
    return UNITY_END();
}
