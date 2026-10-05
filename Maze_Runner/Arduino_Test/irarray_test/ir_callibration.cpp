#include <Arduino.h>

#define IR0 A15
#define IR1 A14
#define IR2 A13
#define IR3 A12
#define IR4 A11
#define IR5 A10
#define IR6 A9
#define IR7 A8
#define IR_EN 49

#define NUM_SENSORS 8
#define CALIBRATION_TIME 5000

int sensorValues[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS];

void readIRSensors()
{
    sensorValues[0] = analogRead(IR0);
    sensorValues[1] = analogRead(IR1);
    sensorValues[2] = analogRead(IR2);
    sensorValues[3] = analogRead(IR3);
    sensorValues[4] = analogRead(IR4);
    sensorValues[5] = analogRead(IR5);
    sensorValues[6] = analogRead(IR6);
    sensorValues[7] = analogRead(IR7);
}

// Helper function: Pauses code until user presses ENTER in Serial Monitor
void waitForUserPrompt()
{
    while (Serial.available() > 0)
    {
        Serial.read(); // Clear buffer
    }
    while (Serial.available() == 0)
    {
        delay(10); // Wait for input
    }
    while (Serial.available() > 0)
    {
        Serial.read(); // Clear buffer after keypress
    }
}

void calibrateSensors()
{
    Serial.println("========================================");
    Serial.println("    IR SENSOR CALIBRATION TEST          ");
    Serial.println("========================================");
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
    }

    // --- PHASE 1: WHITE SURFACE ---
    Serial.println("PHASE 1: WHITE SURFACE CALIBRATION");
    Serial.println("Place ALL sensors on the WHITE surface.");
    Serial.println("--> Type any key in Serial Monitor and press ENTER to start <--");
    waitForUserPrompt();

    Serial.println("\nCalibrating WHITE surface for 5 seconds...");
    unsigned long startTime = millis();
    int sampleCount = 0;

    while (millis() - startTime < CALIBRATION_TIME)
    {
        readIRSensors();

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (sensorValues[i] < sensorMin[i])
            {
                sensorMin[i] = sensorValues[i];
            }
        }

        if (sampleCount % 100 == 0)
        {
            Serial.print("."); // Progress indicator dots
        }

        sampleCount++;
        delay(10);
    }

    Serial.println("\nWHITE SURFACE CALIBRATION COMPLETE!\n");

    // --- PHASE 2: BLACK SURFACE ---
    Serial.println("PHASE 2: BLACK SURFACE CALIBRATION");
    Serial.println("Place/sweep sensors over the BLACK surface or line.");
    Serial.println("--> Type any key in Serial Monitor and press ENTER to start <--");
    waitForUserPrompt();

    Serial.println("\nCalibrating BLACK surface for 5 seconds...");
    startTime = millis();
    sampleCount = 0;

    while (millis() - startTime < CALIBRATION_TIME)
    {
        readIRSensors();

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (sensorValues[i] > sensorMax[i])
            {
                sensorMax[i] = sensorValues[i];
            }
        }

        if (sampleCount % 100 == 0)
        {
            Serial.print(".");
        }

        sampleCount++;
        delay(10);
    }

    Serial.println("\nBLACK SURFACE CALIBRATION COMPLETE!\n");

    // Compute thresholds
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }

    // Summary tables
    Serial.println("===== CALIBRATION SUMMARY =====");
    Serial.println("Sensor | White | Black | Threshold | Range");
    Serial.println("-------|-------|-------|-----------|------");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print("  ");
        Serial.print(i);
        Serial.print("    |  ");
        Serial.print(sensorMin[i]);
        Serial.print("  |  ");
        Serial.print(sensorMax[i]);
        Serial.print("  |    ");
        Serial.print(sensorThreshold[i]);
        Serial.print("    |  ");
        Serial.println(sensorMax[i] - sensorMin[i]);
    }
    Serial.println();

    Serial.println("===== COPY-PASTE VALUES =====");
    Serial.print("int whiteValues[] = {");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorMin[i]);
        if (i < NUM_SENSORS - 1)
            Serial.print(", ");
    }
    Serial.println("};");

    Serial.print("int blackValues[] = {");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorMax[i]);
        if (i < NUM_SENSORS - 1)
            Serial.print(", ");
    }
    Serial.println("};");

    Serial.print("int thresholds[] = {");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorThreshold[i]);
        if (i < NUM_SENSORS - 1)
            Serial.print(", ");
    }
    Serial.println("};");
    Serial.println();
    Serial.println("CALIBRATION COMPLETE!");
}

void continuousReading()
{
    Serial.println();
    Serial.println("===== CONTINUOUS SENSOR READING =====");
    Serial.println("Real-time sensor values (0 = White, 1 = Black):");
    Serial.println();

    while (true)
    {
        readIRSensors();

        Serial.print("Raw: ");
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            Serial.print(sensorValues[i]);
            Serial.print(" ");
        }

        Serial.print("| Binary: ");
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (sensorValues[i] > sensorThreshold[i])
            {
                Serial.print("1 ");
            }
            else
            {
                Serial.print("0 ");
            }
        }
        Serial.println();

        delay(200);
    }
}

void setup()
{
    Serial.begin(115200);

    while (!Serial)
    {
        ;
    }

    pinMode(IR_EN, OUTPUT);
    digitalWrite(IR_EN, HIGH);

    calibrateSensors();
    continuousReading();
}

void loop()
{
    // Empty
}