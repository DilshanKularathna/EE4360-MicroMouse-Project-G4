// ================================================================
// bench.cpp - interactive bench test for the maze runner hardware.
//
// Uses exactly the code the main firmware runs (Motion lib, hardware
// adapter, LineFollower), so what you see here is what the explorer gets.
//
//   pio run -e bench -t upload
//   pio device monitor -e bench      (115200 baud, line ending = Newline)
//
// Commands (one letter + Enter):
//   s  stream sensors every 200 ms (any key stops)
//   w  one readWalls()             t  one readFloorTile()
//   f  move one cell               r / l / u  turn right / left / 180
//   a  align on the front wall     g  gyro drift over 60 s
//   c  IR calibration sweep (5 s)  L  line-follow until bridge exit (15 s max)
//   h  this help
// ================================================================
#include <Arduino.h>
#include "config.h"
#include "IRArray.h"
#include "LineFollower.h"
#include "Gyro.h"
#include "Motion.h"
#include "hardware_adapter.h"

IRArray               ir(IR_PINS, PIN_IR_EN, NUM_IR_SENSORS);
LineFollower          lineFollower(ir);
G4RobotPlatform       platform(ir);
G4LineFollowerAdapter lineAdapter(lineFollower, ir);

static void printHelp() {
    Serial.println(F("\n=== BENCH TEST ==="));
    Serial.println(F(" s stream sensors (any key stops)"));
    Serial.println(F(" w readWalls      t readFloorTile"));
    Serial.println(F(" f one cell       r/l/u turn right/left/180"));
    Serial.println(F(" a front align    g gyro drift 60 s"));
    Serial.println(F(" c IR calibration sweep 5 s"));
    Serial.println(F(" L line follow until bridge exit (15 s max)"));
    Serial.println(F(" h help"));
}

static void clearInput() {
    delay(20);
    while (Serial.available()) Serial.read();
}

static bool keyPressed() {
    if (!Serial.available()) return false;
    clearInput();
    return true;
}

// Same thresholds as G4RobotPlatform::readWalls(), without its gyro refresh
static void streamSensors() {
    bool wasLogging = adapterLog;
    adapterLog = false;
    Serial.println(F("F_cm L_cm R_cm | walls F L R | IR0..IR7 | floor | yaw   (any key stops)"));
    while (!keyPressed()) {
        float f = motionReadDistance(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
        float l = motionReadDistance(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);
        float r = motionReadDistance(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
        int raw[8];
        ir.readRaw(raw);
        TileType tile = platform.readFloorTile();
        if (gyroReady()) gyroUpdate();

        Serial.print(f, 1); Serial.print(' ');
        Serial.print(l, 1); Serial.print(' ');
        Serial.print(r, 1);
        Serial.print(F(" | "));
        Serial.print(f < FRONT_WALL_CM); Serial.print(' ');
        Serial.print(l < WALL_THRESHOLD); Serial.print(' ');
        Serial.print(r < WALL_THRESHOLD);
        Serial.print(F(" |"));
        for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(raw[i]); }
        Serial.print(F(" | "));
        Serial.print(tile == TILE_WHITE ? F("white") : tile == TILE_BRIDGE_MARKER ? F("marker") : F("black"));
        Serial.print(F(" | "));
        Serial.println(gyroReady() ? gyroYaw() : 0.0f, 1);
        delay(200);
    }
    adapterLog = wasLogging;
}

// Higher IR value = darker. Slide the robot over black floor AND white tile.
static void irCalibration() {
    int lo[8], hi[8];
    for (int i = 0; i < 8; i++) { lo[i] = 1023; hi[i] = 0; }
    Serial.println(F("Sweeping for 5 s - slide the array over black floor and a white tile..."));
    unsigned long t0 = millis();
    while (millis() - t0 < 5000) {
        int raw[8];
        ir.readRaw(raw);
        for (int i = 0; i < 8; i++) {
            if (raw[i] < lo[i]) lo[i] = raw[i];
            if (raw[i] > hi[i]) hi[i] = raw[i];
        }
        delay(5);
    }
    Serial.print(F("const int WHITE_VALUES[8] = {"));
    for (int i = 0; i < 8; i++) { Serial.print(lo[i]); Serial.print(i < 7 ? F(", ") : F("};\n")); }
    Serial.print(F("const int BLACK_VALUES[8] = {"));
    for (int i = 0; i < 8; i++) { Serial.print(hi[i]); Serial.print(i < 7 ? F(", ") : F("};\n")); }
    Serial.print(F("const int DEFAULT_IR_THRESHOLDS[8] = {"));
    for (int i = 0; i < 8; i++) { Serial.print((lo[i] + hi[i]) / 2); Serial.print(i < 7 ? F(", ") : F("};\n")); }
    for (int i = 0; i < 8; i++) {
        if (hi[i] - lo[i] < 200) {
            Serial.print(F("WARNING: sensor "));
            Serial.print(i);
            Serial.println(F(" barely changed - did it see both black and white?"));
        }
    }
}

static void lineFollowTest() {
    Serial.println(F("Line following... (any key stops)"));
    unsigned long t0 = millis();
    bool exitSeen = false;
    lineAdapter.begin();
    while (!keyPressed()) {
        if (lineAdapter.step()) { exitSeen = true; break; }
    }
    lineAdapter.stop();
    unsigned long ms = millis() - t0;
    Serial.print(F("Stopped after "));
    Serial.print(ms);
    Serial.print(F(" ms: "));
    if (!exitSeen)          Serial.println(F("stopped by key"));
    else if (ms >= 14900)   Serial.println(F("TIMEOUT (15 s) - exit not detected"));
    else                    Serial.println(F("bridge exit reached (black floor)"));
}

static void gyroDrift() {
    if (!gyroReady()) { Serial.println(F("No gyro found")); return; }
    Serial.println(F("Keep the robot still for 60 s..."));
    gyroUpdate();
    float start = gyroYaw();
    unsigned long t0 = millis(), lastPrint = 0;
    while (millis() - t0 < 60000UL) {
        gyroUpdate();
        if (millis() - lastPrint >= 10000UL) {
            lastPrint = millis();
            Serial.print((millis() - t0) / 1000); Serial.print(F(" s: drift "));
            Serial.print(gyroYaw() - start, 2); Serial.println(F(" deg"));
        }
    }
    Serial.print(F("Drift over 60 s: "));
    Serial.print(gyroYaw() - start, 2);
    Serial.println(F(" deg"));
}

void setup() {
    Serial.begin(115200);
    delay(500);
    adapterLog = true;   // w / t / f / turns print through the adapter's log lines
    Serial.println(F("Initializing - keep the robot still (gyro calibration)..."));
    platform.begin();
    ir.setThresholds(DEFAULT_IR_THRESHOLDS);
    lineFollower.begin(WHITE_VALUES, BLACK_VALUES);
    lineFollower.setPIDGains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    lineFollower.setSpeeds(DEFAULT_BASE_PWM, DEFAULT_MAX_PWM);
    Serial.println(gyroReady() ? F("Gyro OK") : F("Gyro NOT found - encoder turns"));
    printHelp();
}

void loop() {
    if (!Serial.available()) return;
    char c = Serial.read();
    clearInput();
    switch (c) {
        case 's': streamSensors(); break;
        case 'w': platform.readWalls(); break;       // the adapter logs the reading
        case 't': platform.readFloorTile(); break;
        case 'f': platform.moveForwardOneCell(); break;
        case 'r': platform.turn(1); break;
        case 'l': platform.turn(-1); break;
        case 'u': platform.turn(2); break;
        case 'a':
            motionAlignFront();
            Serial.print(F("Front now "));
            Serial.print(motionReadDistance(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO), 1);
            Serial.print(F(" cm (target "));
            Serial.print(FRONT_TARGET_CM, 1);
            Serial.println(F(")"));
            break;
        case 'g': gyroDrift(); break;
        case 'c': irCalibration(); break;
        case 'L': lineFollowTest(); break;
        case 'h': printHelp(); break;
        default: break;
    }
    Serial.println(F("> ready"));
}
