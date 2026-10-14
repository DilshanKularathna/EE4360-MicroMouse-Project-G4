#include <Arduino.h>
#include <math.h>

// Measure the distance from the drive-axle midpoint to the farthest chassis
// end, and the full outside width (including anything that can hit a wall).
const float AXLE_TO_FARTHEST_END_CM = 12.6f;
const float CHASSIS_WIDTH_CM = 18.0f;
const float EXPECTED_SWEEP_RADIUS_CM = 15.4842f; // sqrt(12.6^2 + 9^2)
const float GEOMETRY_TEST_TOLERANCE_CM = 0.05f;

void setup() {
    Serial.begin(115200);
    delay(500);

    const float halfWidth = CHASSIS_WIDTH_CM * 0.5f;
    const float sweepRadiusCm = sqrtf(
        AXLE_TO_FARTHEST_END_CM * AXLE_TO_FARTHEST_END_CM +
        halfWidth * halfWidth);
    const float sweepDiameterCm = 2.0f * sweepRadiusCm;
    const bool calculationPassed =
        fabsf(sweepRadiusCm - EXPECTED_SWEEP_RADIUS_CM) <= GEOMETRY_TEST_TOLERANCE_CM;

    Serial.println(F("=== Turn sweep geometry test ==="));
    Serial.println(F("Motors are not controlled by this test."));
    Serial.print(F("Axle to farthest chassis end: "));
    Serial.print(AXLE_TO_FARTHEST_END_CM, 1);
    Serial.println(F(" cm"));
    Serial.print(F("Overall chassis width: "));
    Serial.print(CHASSIS_WIDTH_CM, 1);
    Serial.println(F(" cm"));
    Serial.print(F("Farthest-corner sweep radius: "));
    Serial.print(sweepRadiusCm, 2);
    Serial.println(F(" cm"));
    Serial.print(F("Full sweep-circle diameter: "));
    Serial.print(sweepDiameterCm, 2);
    Serial.println(F(" cm"));
    Serial.println(calculationPassed ? F("Geometry calculation: PASS")
                                     : F("Geometry calculation: FAIL"));
    Serial.println(F("Compare the radius with available clearance around the axle."));
    Serial.println(F("This calculation alone does not confirm a physical turn is clear."));
}

void loop() {}
