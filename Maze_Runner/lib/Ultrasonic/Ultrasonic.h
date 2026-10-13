#ifndef ULTRASONICS_H
#define ULTRASONICS_H

#include <Arduino.h>

extern float frontDist;
extern float leftDist;
extern float rightDist;

bool isPathClearFront();
bool isWallOnRight();
bool isWallOnLeft();
float readUltrasonic(int trigPin, int echoPin);
float calibrateDistance(float rawDist, int trigPin);
void readAllSensors();

#endif // ULTRASONICS_H