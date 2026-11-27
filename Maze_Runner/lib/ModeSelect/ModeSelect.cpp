#include "ModeSelect.h"
#include "config.h"

void modeBegin() {
    for (uint8_t i = 0; i < 4; i++) pinMode(DIP_PINS[i], INPUT);   // external pull-down resistors
    delay(10);
}

uint8_t modeReadRaw() {
    uint8_t value = 0;
    for (uint8_t i = 0; i < 4; i++)
        if (digitalRead(DIP_PINS[i]) == DIP_ON_LEVEL) value |= (1 << i);
    return value;
}

uint8_t modeReadStable() {
    uint8_t value = modeReadRaw();
    unsigned long since = millis();
    while (millis() - since < 200) {
        uint8_t now = modeReadRaw();
        if (now != value) {
            value = now;
            since = millis();
        }
        delay(5);
    }
    return value;
}

const char *modeName(uint8_t mode) {
    switch (mode) {
        case MODE_IDLE:          return "idle";
        case MODE_EXPLORE_FULL:  return "explore full";
        case MODE_SPEED_RUN:     return "speed run";
        case MODE_EXPLORE_4X4:   return "explore 4x4";
        case MODE_EXPLORE_9X9:   return "explore 9x9";
        case MODE_TEST_STRAIGHT: return "straight test";
        case MODE_TEST_TURN:     return "turn test";
        case MODE_TEST_LINE:     return "line test";
        case MODE_ERASE_MAP:     return "erase map";
        default:                 return "unused";
    }
}

void modePrint(uint8_t mode) {
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print("SW");
        Serial.print(i + 1);
        Serial.print((mode & (1 << i)) ? "=on  " : "=off ");
    }
    Serial.print("-> ");
    Serial.print(mode);
    Serial.print(" (");
    Serial.print(modeName(mode));
    Serial.println(")");
}
