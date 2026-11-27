#ifndef MODE_SELECT_H
#define MODE_SELECT_H

#include <Arduino.h>

// Run mode from the 4-way DIP switch. Value = SW4 SW3 SW2 SW1 (SW1 = bit 0).
// SW4 on = test modes. Pins and polarity are in include/config.h.
enum RobotMode : uint8_t {
    MODE_IDLE          = 0,    // off off off off : print sensors, motors off
    MODE_EXPLORE_FULL  = 1,    // SW1
    MODE_SPEED_RUN     = 2,    // SW2
    MODE_EXPLORE_4X4   = 3,    // SW1 SW2
    MODE_EXPLORE_9X9   = 4,    // SW3
    MODE_TEST_STRAIGHT = 8,    // SW4
    MODE_TEST_TURN     = 9,    // SW4 SW1
    MODE_TEST_LINE     = 10,   // SW4 SW2
    MODE_ERASE_MAP     = 15    // all on
};

void        modeBegin();                   // configure the switch pins
uint8_t     modeReadRaw();                 // 0..15, read once
uint8_t     modeReadStable();              // waits until the value holds for 200 ms
const char *modeName(uint8_t mode);        // "unused" for values with no mode
void        modePrint(uint8_t mode);       // "SW1=on SW2=off ... -> 1 (explore full)"

#endif // MODE_SELECT_H
