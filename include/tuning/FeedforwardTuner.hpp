#pragma once

#include "hardware/Motor/MotorGroup.hpp"
#include "units/units.hpp"
#include "utils/Utils.hpp"
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace tuning {

/**
 * Utility class for tuning feedforward control parameters (kS, kV, kA)
 */
class FeedforwardTuner {
public:
    /**
     * Constructor for FeedforwardTuner
     */
    FeedforwardTuner();

    /**
     * Tunes the kS parameter (static friction)
     * @return The measured kS value
     */
    Number tuneKs();

    /**
     * Tunes the kV parameter (velocity constant)
     * @param kS The static friction constant to use in the test
     * @return The measured kV value
     */
    Number tuneKv(Number kS);

    /**
     * Tunes the kA parameter (acceleration constant)
     * @param kS The static friction constant
     * @param kV The velocity constant
     * @return The measured kA value
     */
    Number tuneKa(Number kS, Number kV);

    /**
     * Sets the wheel diameter for velocity calculations
     * @param diameter Wheel diameter
     */
    void setWheelDiameter(Length diameter);

    /**
     * Add a callback for velocity data visualization/recording
     * @param callback Function to call with voltage and velocity data points
     */
    void addVelocityDataCallback(std::function<void(double voltage, double velocity)> callback);

    /**
     * Add a callback for acceleration data visualization/recording
     * @param callback Function to call with acceleration and voltage data points
     */
    void addAccelerationDataCallback(std::function<void(double acceleration, double voltage)> callback);
    
    /**
     * Clear all registered callbacks
     */
    void clearCallbacks();

private:
    Length m_wheelDiameter = 2.75_in;

    // Callbacks for data collection/visualization (multiple views can register)
    std::vector<std::function<void(double, double)>> m_velocityDataCallbacks;
    std::vector<std::function<void(double, double)>> m_accelerationDataCallbacks;
};

} // namespace tuning
