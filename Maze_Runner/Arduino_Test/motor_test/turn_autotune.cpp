// Turn autotune: tries TURN_KP / TURN_MIN_DUTY / max duty combinations on 90 degree
// turns (right then left, so the robot stays in place) and scores each with a cost.
// Place the robot on the floor with space to spin. Takes about 4 minutes (24 sets).

#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "globals.h"
#include "Gyro.h"

const float KP_LIST[]  = {0.3f, 0.5f, 0.8f, 1.2f};
const int   MIN_LIST[] = {20, 22, 25};
const int   MAX_LIST[] = {30, 35};
const int   TURNS_PER_SET = 4;   // right, left, right, left

// Cost (lower is better): accuracy first, then overshoot, then speed.
// 1 deg final error = 10, 1 deg overshoot = 3, 1 second = 1, timeout = 1000.
float turnCost(const TurnResult &r) {
    if (r.timedOut) return 1000.0f;
    return 10.0f * fabs(r.finalError) + 3.0f * r.overshoot + r.timeMs / 1000.0f;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    initEncoders();
    stopMotors();

    if (!gyroBegin()) {
        Serial.println("MPU-6050 NOT FOUND: autotune needs the gyro");
        while (1) {}
    }
    Serial.println("Turn autotune starting in 3 s. Keep clear of the robot.");
    delay(3000);
    Serial.println("kp,minDuty,maxDuty,avgErr,avgOver,avgTimeMs,timeouts,cost");

    float bestCost = 1e9f;
    float bestKp = 0.0f;
    int bestMin = 0, bestMax = 0;

    for (float kp : KP_LIST) {
        for (int minD : MIN_LIST) {
            for (int maxD : MAX_LIST) {
                if (!gyroCalibrate()) Serial.println("# warning: moved during calibration");

                float sumErr = 0, sumOver = 0, sumCost = 0;
                unsigned long sumTime = 0;
                int timeouts = 0;

                for (int i = 0; i < TURNS_PER_SET; i++) {
                    float angle = (i % 2 == 0) ? 90.0f : -90.0f;
                    TurnResult r = turnDegreesWith(angle, kp, minD, maxD);
                    sumErr  += fabs(r.finalError);
                    sumOver += r.overshoot;
                    sumTime += r.timeMs;
                    sumCost += turnCost(r);
                    if (r.timedOut) timeouts++;
                    delay(300);
                }

                float cost = sumCost / TURNS_PER_SET;
                Serial.print(kp, 2);                          Serial.print(',');
                Serial.print(minD);                           Serial.print(',');
                Serial.print(maxD);                           Serial.print(',');
                Serial.print(sumErr / TURNS_PER_SET, 2);      Serial.print(',');
                Serial.print(sumOver / TURNS_PER_SET, 2);     Serial.print(',');
                Serial.print(sumTime / TURNS_PER_SET);        Serial.print(',');
                Serial.print(timeouts);                       Serial.print(',');
                Serial.println(cost, 2);

                if (cost < bestCost) {
                    bestCost = cost;
                    bestKp = kp;
                    bestMin = minD;
                    bestMax = maxD;
                }
            }
        }
    }

    Serial.println("===== BEST =====");
    Serial.print("#define TURN_KP                  "); Serial.print(bestKp, 2); Serial.println("f");
    Serial.print("#define TURN_MIN_DUTY            "); Serial.println(bestMin);
    Serial.print("// max duty (TURN_SPEED in globals.h): "); Serial.println(bestMax);
    Serial.print("// cost = "); Serial.println(bestCost, 2);
}

void loop() {}
