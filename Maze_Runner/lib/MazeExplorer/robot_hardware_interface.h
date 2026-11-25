#ifndef ROBOT_HARDWARE_INTERFACE_H
#define ROBOT_HARDWARE_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "maze_types.h"

/**
 * ============================================================================
 * Wall Sensor Readings
 * ----------------------------------------------------------------------------
 * Clean boolean flags indicating wall presence around the current cell.
 * Derived by the hardware adapter from ultrasonic or ToF distance sensors.
 * ============================================================================
 */
struct WallSensors {
    bool wallFront;
    bool wallLeft;
    bool wallRight;
};

/**
 * ============================================================================
 * Pluggable Line Follower Interface
 * ----------------------------------------------------------------------------
 * Anyone implementing line following (e.g. 220601P_linefollow) can implement
 * this interface or provide a step callback without modifying the maze logic.
 * ============================================================================
 */
class ILineFollower {
public:
    virtual ~ILineFollower() {}
    virtual void begin() = 0;
    
    /**
     * Executes one step/iteration of line following across the bridge.
     * @return true when the end of the bridge/line corridor is reached.
     */
    virtual bool step() = 0;
    
    virtual void stop() = 0;
};

/**
 * ============================================================================
 * Robot Hardware Platform Abstraction
 * ----------------------------------------------------------------------------
 * Decouples the exploration algorithm from lower-level motor PWM, encoder ticks,
 * and raw ADC sensor readings.
 * ============================================================================
 */
class IRobotPlatform {
public:
    virtual ~IRobotPlatform() {}
    
    virtual void begin() = 0;

    /**
     * Read wall status relative to robot's current heading.
     */
    virtual WallSensors readWalls() = 0;

    /**
     * Detect current tile type using the IR sensor array underneath the robot.
     * Detects:
     *   TILE_NORMAL_BLACK: standard black floor
     *   TILE_WHITE: full-white start or goal tile
     *   TILE_BRIDGE_MARKER: 9cm white square with 3cm black center
     */
    virtual TileType readFloorTile() = 0;

    /**
     * Drives forward by exactly one 25 cm cell using encoder feedback & wall centering.
     * @return true if movement succeeded, false if an obstacle or stall occurred.
     */
    virtual bool moveForwardOneCell() = 0;

    /**
     * Rotates robot by relative increments of 90 degrees:
     *   relativeTurns = -1 : Turn Left (90° CCW)
     *   relativeTurns = +1 : Turn Right (90° CW)
     *   relativeTurns =  2 : Turn Around (180°)
     */
    virtual void turn(int relativeTurns) = 0;

    /**
     * Immediately stops both drive motors.
     */
    virtual void stop() = 0;
};

#endif // ROBOT_HARDWARE_INTERFACE_H
