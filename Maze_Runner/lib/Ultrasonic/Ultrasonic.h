#ifndef ULTRASONICS_H
#define ULTRASONICS_H

#include <Arduino.h>

class Ultrasonics {
private:
    uint8_t trigPin;
    uint8_t echoPin;

public:
    Ultrasonics(uint8_t trig, uint8_t echo);

    void begin();
    
    // Returns measured distance in cm. Returns -1 if out of range/timeout.
    float getDistanceCM();
};

#endif // ULTRASONICS_H