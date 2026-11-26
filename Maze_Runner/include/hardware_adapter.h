#ifndef HARDWARE_ADAPTER_H
#define HARDWARE_ADAPTER_H

#include <Arduino.h>
#include "robot_hardware_interface.h"
#include "IRArray.h"
#include "LineFollower.h"

/**
 * ============================================================================
 * Concrete Platform Adapter for Group 4 Micromouse Hardware
 * ----------------------------------------------------------------------------
 * Connects the abstract IRobotPlatform interface to Group 4's hardware:
 * - Motion lib (encoder PID moves, gyro turns, wall centring)
 * - 3 ultrasonic sensors (front / left / right) for wall detection
 * - IRArray (8-channel reflectance array on the bottom) for floor tiles
 * ============================================================================
 */
// Run-time switch for the adapter's log lines (starts as LOG_ADAPTER in config.h).
extern bool adapterLog;

class G4RobotPlatform : public IRobotPlatform {
private:
    IRArray& irArray;
    bool     initialized;

    // Bridge marker detection state machine
    enum MarkerState {
        MARKER_IDLE = 0,
        MARKER_WHITE_1,
        MARKER_BLACK_CENTER,
        MARKER_WHITE_2
    };
    MarkerState   markerState;
    unsigned long markerStateTime;

    TileType classifyFloor();

public:
    explicit G4RobotPlatform(IRArray& ir);

    // Safe to call more than once: later calls only refresh the gyro bias.
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
 * Wraps Group 4's LineFollower for the bridge crossing.
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
