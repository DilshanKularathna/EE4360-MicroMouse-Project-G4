/**
 * ============================================================================
 * Spec stress runner: runs the MazeExplorer on many random competition arenas
 * and prints how often it succeeds.
 *
 *   pio run -e native -t exec                      -> summary report
 *   .pio/build/native/program.exe --seed 42        -> draw one arena and its run
 *   .pio/build/native/program.exe --count 2000     -> more arenas
 * ============================================================================
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "arena_sim.h"

using namespace sim;

static void printRun(const RunReport& r) {
    printf("  Result         : %s\n", r.failure.empty() ? "REACHED FINISH" : r.failure.c_str());
    printf("  Final state    : %s\n", stateName(r.finalState));
    printf("  Moves %d, 90-turns %d, 180-turns %d, blocked moves %d, START re-entries %d\n",
           r.moves, r.turns90, r.turns180, r.blockedMoves, r.startReentries);
    if (r.reachedFinish)
        printf("  Est. time to FINISH: %.0f s (limit %.0f s)\n", r.secondsToFinish, RUN_LIMIT_S);
    printf("  Section A map  : %d/16 cells visited, %d wrong wall bits, bridge tile %s, start %s\n",
           r.mapA.visited, r.mapA.mismatches, r.bridgeEntryCorrect ? "correct" : "WRONG",
           r.startCorrect ? "correct" : "WRONG");
    printf("  Section B map  : %d/81 cells visited, %d wrong wall bits\n",
           r.mapB.visited, r.mapB.mismatches);
    printf("  Shortest route proven: A %s, B %s\n", r.shortestA ? "yes" : "NO", r.shortestB ? "yes" : "NO");
}

static void summary(const char* title, const SimConfig& cfg, int count) {
    int crossed = 0, finished = 0, falseFinish = 0, overTime = 0, blocked = 0, shortA = 0, shortB = 0;
    double sumT = 0, maxT = 0;
    std::map<std::string, int> reasons;
    std::vector<uint32_t> failedSeeds;

    for (int i = 1; i <= count; i++) {
        RunReport r = runArena(generateArena((uint32_t)i), cfg);
        crossed  += r.crossedBridge;
        finished += r.reachedFinish;
        falseFinish += r.falseFinish;
        overTime += r.overTime();
        blocked  += r.blockedMoves > 0;
        shortA   += r.shortestA;
        shortB   += r.shortestB;
        if (r.reachedFinish) {
            sumT += r.secondsToFinish;
            if (r.secondsToFinish > maxT) maxT = r.secondsToFinish;
        }
        if (!r.failure.empty()) {
            // Group by the text before the first '(' or digit so similar failures merge
            std::string key = r.failure;
            size_t cut = key.find_first_of("(0123456789");
            if (cut != std::string::npos) key = key.substr(0, cut);
            reasons[key]++;
            if (failedSeeds.size() < 8) failedSeeds.push_back(r.seed);
        }
    }

    auto pct = [count](int n) { return 100.0 * n / count; };
    printf("\n=== %s (%d arenas) ===\n", title, count);
    printf("  Crossed the bridge             : %4d  (%5.1f%%)\n", crossed, pct(crossed));
    printf("  Reached FINISH                 : %4d  (%5.1f%%)\n", finished, pct(finished));
    printf("  Shortest START->bridge proven  : %4d  (%5.1f%%)\n", shortA, pct(shortA));
    printf("  Shortest bridge->FINISH proven : %4d  (%5.1f%%)\n", shortB, pct(shortB));
    printf("  Reported goal on wrong tile    : %4d\n", falseFinish);
    printf("  Tried to drive into a wall     : %4d arenas\n", blocked);
    printf("  Over the 8 min limit (est.)    : %4d\n", overTime);
    if (finished)
        printf("  Est. time to FINISH            : avg %.0f s, max %.0f s\n", sumT / finished, maxT);
    if (!reasons.empty()) {
        printf("  Failure reasons:\n");
        for (const auto& kv : reasons) printf("    %4d  %s\n", kv.second, kv.first.c_str());
        printf("  Some failing seeds (view with --seed N):");
        for (uint32_t s : failedSeeds) printf(" %u", s);
        printf("\n");
    }
}

int main(int argc, char** argv) {
    int count = 500;
    long seed = -1;
    double noise = 0.0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--seed") && i + 1 < argc)  seed  = atol(argv[++i]);
        else if (!strcmp(argv[i], "--count") && i + 1 < argc) count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--noise") && i + 1 < argc) noise = atof(argv[++i]);
    }

    if (seed >= 0) {
        Arena ar = generateArena((uint32_t)seed);
        printArena(ar);
        SimConfig cfg;
        cfg.wallNoise = noise;
        printf("\n");
        printRun(runArena(ar, cfg));
        return 0;
    }

    printf("Time model: move %.2f s, 90-turn %.2f s, 180-turn %.2f s, wall read %.2f s, bridge %.0f s\n",
           T_MOVE, T_TURN90, T_TURN180, T_SENSE, T_BRIDGE);
    printf("(estimates - measure on the robot and update arena_sim.h)\n");

    SimConfig perfect;
    summary("Perfect sensors", perfect, count);

    SimConfig noisy;
    noisy.wallNoise = 0.02;
    noisy.floorMiss = 0.05;
    summary("2% wall-reading errors, 5% floor-tile misses", noisy, count);
    return 0;
}
