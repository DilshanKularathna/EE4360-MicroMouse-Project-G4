// DIP switch test. Motors are never touched.
//   pio run -e dip_test -t upload
// Open the serial monitor (115200) and flip the switches one at a time.

#include <Arduino.h>
#include "config.h"
#include "ModeSelect.h"

static uint8_t       lastValue = 0xFF;
static unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);
    modeBegin();
    Serial.println("DIP TEST - start with all switches OFF, expect value 0");
}

void loop() {
    uint8_t value = modeReadStable();
    if (value == lastValue && millis() - lastPrint < 2000) return;

    Serial.print("raw pins ");
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(DIP_PINS[i]);
        Serial.print(digitalRead(DIP_PINS[i]) == HIGH ? ":HIGH " : ":LOW  ");
    }
    Serial.print("| ");
    modePrint(value);

    lastValue = value;
    lastPrint = millis();
}
