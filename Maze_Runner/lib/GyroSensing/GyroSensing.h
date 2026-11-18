#pragma once

#include <Arduino.h>

class GyroSensing {
    public:
        // Check sensor is working and wiring is correct in I2C Bus
        bool begin();

        // Start calibration of sensor for the robot
        void calibrate();

        // Update the current sensor data
        void update();

        // Get Accelerations of each axis
        float getAccelerationX() const {return _accelerationX;}
        float getAccelerationY() const {return _accelerationY;}
        float getAccelerationZ() const {return _accelerationZ;}

        // Get Angles of devations
        float getdeviationXY() const {return _deviationXY;}
        float getdeviationXZ() const {return _deviationXZ;}
        float getdeviationYZ() const {return _deviationYZ;}

    private:
    float _accelerationX = 0.0f, _accelerationY = 0.0f, _accelerationZ = 0.0f;
    float _deviationXY = 0.0f, _deviationXZ = 0.0f, _deviationYZ = 0.0f;
    float _offsetXY =0.0f, _offsetXZ = 0.0f, _offsetYZ = 0.0f;

};