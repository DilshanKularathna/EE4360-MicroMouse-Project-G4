#pragma once

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
    // PID Parameters
    float _kp;
    float _ki;
    float _kd;

    // PID State Variables
    int _lastError;
    float _integral;
    float _integralMin;
    float _integralMax;

    // Battery Parameters
    float _vMax;
    float _criticalLowV;
    float _r1;
    float _r2;
    float _vRef;
    int _batteryPin;

    // Operational Flags
    bool _useBatteryCorrection;

public:
    /**
     * @brief Constructor for the PID Controller.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    PIDController(float kp, float ki, float kd);

    /**
     * @brief Configures the hardware and parameters for battery monitoring.
     * @param pin The Arduino analog pin connected to the voltage divider (e.g., A7).
     * @param vMax Full charge voltage of the pack (e.g., 12.6 for 3S, 16.8 for 4S Li-ion).
     * @param criticalLowV Low voltage cutoff threshold (e.g., 9.9 for 3S, 13.2 for 4S).
     * @param r1 Value of the top resistor in Ohms (connected to Battery +).
     * @param r2 Value of the bottom resistor in Ohms (connected to GND).
     * @param vRef Reference voltage of the Arduino microcontroller (typically 5.0 or 3.3).
     */
    void setupBattery(int pin, float vMax, float criticalLowV, float r1 = 10000.0, float r2 = 4700.0, float vRef = 5.0);

    /**
     * @brief Enables or disables real-time battery voltage compensation.
     * @param enable True to enable scaling factor calculation, false to run standard PID.
     */
    void setBatteryCorrection(bool enable);

    /**
     * @brief Sets internal limit bounds to prevent integral windup.
     * @param minVal Lower bound for stored integral value.
     * @param maxVal Upper bound for stored integral value.
     */
    void setIntegralLimits(float minVal, float maxVal);

    /**
     * @brief Reads the voltage divider pin and processes the true battery pack voltage.
     * @return Calculated battery pack voltage in Volts.
     */
    float readBatteryVoltage();

    /**
     * @brief Evaluates whether the battery has dropped below the unsafe voltage floor.
     * @return True if voltage is below critical cutoff, false if safe.
     */
    bool isBatteryCritical();

    /**
     * @brief Calculates the steering adjustment value from line positional error.
     * @param error Symmetrical deviation from line center (position - 3500).
     * @return Steering adjustment multiplier factor.
     */
    float computeAdjustment(int error);

    /**
     * @brief Modifies a baseline input PWM speed step using the live battery scale ratio.
     *        If correction is turned off, it returns the baseline speed unadjusted.
     * @param baseSpeed The target baseline velocity token or calculation (e.g., BASE_PWM + adjustment).
     * @return The final voltage-compensated PWM command integer.
     */
    int scalePWM(int baseSpeed);

    /**
     * @brief Resets the accumulated integral and historical error state registers.
     */
    void reset();
};

#endif
