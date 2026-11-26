#include "section_map.h"

void SectionMap::reset(uint8_t sectionSize) {
    n = sectionSize;
    w = (uint8_t)(2 * sectionSize - 1);
    for (uint8_t y = 0; y < MAX_WORK; y++)
        for (uint8_t x = 0; x < MAX_WORK; x++)
            cells[y][x] = 0;
    minX = maxX = minY = maxY = origin();
    anyVisited = false;
    factCount = 0;
}

void SectionMap::markVisited(int8_t x, int8_t y) {
    cells[y][x] |= CELL_VISITED;
    if (!anyVisited) {
        minX = maxX = x;
        minY = maxY = y;
        anyVisited = true;
        return;
    }
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
}

void SectionMap::setWall(int8_t x, int8_t y, Direction d) {
    cells[y][x] |= (uint8_t)(1 << d);
    int8_t nx = x + stepX(d), ny = y + stepY(d);
    if (inWork(nx, ny)) cells[ny][nx] |= (uint8_t)(1 << ((d + 2) & 3));
}

void SectionMap::addFact(int8_t x, int8_t y, int8_t dir) {
    for (uint8_t i = 0; i < factCount; i++)
        if (facts[i].x == x && facts[i].y == y && facts[i].dir == dir) return;
    if (factCount < MAX_FACTS) facts[factCount++] = {x, y, dir};
}

void SectionMap::requireCorner(int8_t x, int8_t y) { addFact(x, y, -1); }
void SectionMap::requireOuterSide(int8_t x, int8_t y, Direction d) { addFact(x, y, (int8_t)d); }

// Bit b set = the section can start at working column (or row) b.
uint16_t SectionMap::feasible(bool xAxis) const {
    int8_t last = (int8_t)(n - 1);
    int8_t lo = (int8_t)((xAxis ? maxX : maxY) - last);
    int8_t hi = xAxis ? minX : minY;
    if (lo < 0) lo = 0;
    if (hi > (int8_t)(w - n)) hi = (int8_t)(w - n);

    uint16_t mask = 0;
    for (int8_t b = lo; b <= hi; b++) {
        bool ok = true;
        for (uint8_t i = 0; i < factCount && ok; i++) {
            const Fact& f = facts[i];
            int8_t v = xAxis ? f.x : f.y;
            if (f.dir < 0) {
                ok = (v == b || v == b + last);
            } else if (xAxis && f.dir == DIR_WEST) {
                ok = (v == b);
            } else if (xAxis && f.dir == DIR_EAST) {
                ok = (v == b + last);
            } else if (!xAxis && f.dir == DIR_SOUTH) {
                ok = (v == b);
            } else if (!xAxis && f.dir == DIR_NORTH) {
                ok = (v == b + last);
            }
        }
        if (ok) mask |= (uint16_t)(1u << b);
    }
    return mask;
}

static bool coveredBy(uint16_t mask, int8_t v, uint8_t n, bool all) {
    if (mask == 0) return false;
    for (int8_t b = 0; b < 16; b++) {
        if (!(mask & (1u << b))) continue;
        bool in = (v >= b && v < b + (int8_t)n);
        if (all && !in) return false;
        if (!all && in) return true;
    }
    return all;
}

bool SectionMap::canBeInside(int8_t x, int8_t y) const {
    return inWork(x, y) && coveredBy(feasible(true), x, n, false) && coveredBy(feasible(false), y, n, false);
}

bool SectionMap::surelyInside(int8_t x, int8_t y) const {
    return inWork(x, y) && coveredBy(feasible(true), x, n, true) && coveredBy(feasible(false), y, n, true);
}

static bool singleBit(uint16_t m) { return m != 0 && (m & (m - 1)) == 0; }

bool SectionMap::placementUnique() const {
    return singleBit(feasible(true)) && singleBit(feasible(false));
}

bool SectionMap::placement(int8_t& bx, int8_t& by) const {
    uint16_t mx = feasible(true), my = feasible(false);
    if (!mx || !my) return false;
    bx = 0;
    while (!(mx & (1u << bx))) bx++;
    by = 0;
    while (!(my & (1u << by))) by++;
    return true;
}
