#ifndef SECTION_MAP_H
#define SECTION_MAP_H

#include <stdint.h>
#include <stdbool.h>
#include "maze_types.h"

/**
 * ============================================================================
 * SectionMap - wall map of one maze section with an unknown position
 * ----------------------------------------------------------------------------
 * The robot does not know where it is inside a section when it starts mapping
 * it (START can be any tile of A, and the bridge can enter any corner of B).
 * So the map is a working grid of (2N-1) x (2N-1) cells with the robot starting
 * in the middle: whichever cell of the N x N section it is really on, the whole
 * section fits in the working grid.
 *
 * "Placement" = where the real N x N section lies in the working grid. It must
 * contain every visited cell and satisfy the known facts (e.g. "this cell is a
 * corner", "this side is on the outer wall"). The X and Y placement are
 * independent, so each is kept as a bitmask of possible start columns / rows.
 *
 * Visited cells have all four sides known: three are sensed, and the side the
 * robot came in through is open (the start cell is scanned twice instead).
 * ============================================================================
 */
class SectionMap {
public:
    static const uint8_t MAX_WORK = 2 * MAZE_B_SIZE - 1;   // 17
    static const uint8_t MAX_FACTS = 4;

    void reset(uint8_t sectionSize);

    uint8_t sectionSize() const { return n; }
    uint8_t workSize() const { return w; }
    int8_t  origin() const { return (int8_t)(n - 1); }     // robot's first cell (both axes)

    bool inWork(int8_t x, int8_t y) const { return x >= 0 && y >= 0 && x < w && y < w; }
    bool isVisited(int8_t x, int8_t y) const { return (cells[y][x] & CELL_VISITED) != 0; }
    bool hasWall(int8_t x, int8_t y, Direction d) const { return (cells[y][x] & (1 << d)) != 0; }
    uint8_t raw(int8_t x, int8_t y) const { return cells[y][x]; }

    void markVisited(int8_t x, int8_t y);
    void setWall(int8_t x, int8_t y, Direction d);           // sets both sides

    // Known facts about the placement
    void requireCorner(int8_t x, int8_t y);                   // cell is a corner of the section
    void requireOuterSide(int8_t x, int8_t y, Direction d);   // that side is the outer wall

    bool canBeInside(int8_t x, int8_t y) const;               // inside some possible placement
    bool surelyInside(int8_t x, int8_t y) const;              // inside every possible placement
    bool placementUnique() const;
    // First possible placement: section cell (sx, sy) = working cell (bx + sx, by + sy).
    bool placement(int8_t& bx, int8_t& by) const;
    uint16_t placementsX() const { return feasible(true); }
    uint16_t placementsY() const { return feasible(false); }

    static int8_t stepX(Direction d) { return d == DIR_EAST ? 1 : (d == DIR_WEST ? -1 : 0); }
    static int8_t stepY(Direction d) { return d == DIR_NORTH ? 1 : (d == DIR_SOUTH ? -1 : 0); }

private:
    struct Fact {
        int8_t x, y;
        int8_t dir;   // -1 = corner, otherwise the outer side
    };

    uint8_t n = MAZE_A_SIZE;
    uint8_t w = 2 * MAZE_A_SIZE - 1;
    uint8_t cells[MAX_WORK][MAX_WORK];
    int8_t  minX, maxX, minY, maxY;
    bool    anyVisited;
    Fact    facts[MAX_FACTS];
    uint8_t factCount;

    uint16_t feasible(bool xAxis) const;
    void addFact(int8_t x, int8_t y, int8_t dir);
};

#endif // SECTION_MAP_H
