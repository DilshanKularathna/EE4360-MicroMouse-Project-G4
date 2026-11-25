#ifndef HARDWARE_ADAPTER_H
#define HARDWARE_ADAPTER_H

#include <Arduino.h>
#include "robot_hardware_interface.h"
#include "MotorDriver.h"
#include "IRArray.h"
#include "Ultrasonic.h"
#include "LineFollower.h"
#include "config.h"

/**
 * ============================================================================
 * Concrete Platform Adapter for Group 4 Micromouse Hardware
 * ----------------------------------------------------------------------------
 * Connects the abstract IRobotPlatform interface to Group 4's hardware:
 * - MotorDriver (RPWM/LPWM with optical encoders)
 * - IRArray (8-channel reflectance array on bottom)
 * - Ultrasonic sensor(s) (Front/Left/Right distance measurement)
 * ============================================================================
 */
class G4RobotPlatform : public IRobotPlatform {
private:
    MotorDriver& motorDriver;
    IRArray&     irArray;
    Ultrasonics& frontUltrasonic;

    float wallThresholdCM;

    // Bridge marker detection state machine
    enum MarkerState {
        MARKER_IDLE = 0,
        MARKER_WHITE_1,
        MARKER_BLACK_CENTER,
        MARKER_WHITE_2
    };
    MarkerState   markerState;
    unsigned long markerStateTime;

public:
    G4RobotPlatform(MotorDriver& motors, IRArray& ir, Ultrasonics& frontUltra, float wallThresh = 18.0f);

    virtual void begin() override;
    virtual WallSensors readWalls() override;
    virtual TileType readFloorTile() override;
    virtual bool moveForwardOneCell() override;
    virtual void turn(int relativeTurns) override;
    virtual void stop() override;
};

/**
 * ============================================================================
 * Pluggable Line Follower Adapter
 * ----------------------------------------------------------------------------
 * Wraps Group 4's LineFollower or teammate's 220601P_linefollow implementation.
 * Can be swapped out at any time with an alternate line follower.
 * ============================================================================
 */
class G4LineFollowerAdapter : public ILineFollower {
private:
    LineFollower& lineFollower;
    IRArray&      irSensors;
    unsigned long bridgeStartTime;
    unsigned long maxBridgeCrossTimeMs;

public:
    G4LineFollowerAdapter(LineFollower& follower, IRArray& ir, unsigned long maxTimeoutMs = 15000);

    virtual void begin() override;
    virtual bool step() override;
    virtual void stop() override;
};

#endif // HARDWARE_ADAPTER_H
