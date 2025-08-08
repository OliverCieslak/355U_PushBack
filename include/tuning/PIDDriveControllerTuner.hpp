#pragma once

#include "control/PIDDriveController.hpp"
#include "tuning/CharacterizationView.hpp"
#include "tuning/FeedforwardTuner.hpp"
#include "units/units.hpp"
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace tuning {

/**
 * @brief Tuning utility for PIDDriveController parameters
 * 
 * This class provides methods to interactively tune and test
 * parameters for the PIDDriveController.
 */
class PIDDriveControllerTuner {
public:
    /**
     * @brief Construct a new PIDDriveControllerTuner
     * 
     * @param controller Reference to the PIDDriveController to tune
     * @param configFilePath Path to save/load configurations
     */
    PIDDriveControllerTuner(
        control::PIDDriveController& controller,
        const std::string& configFilePath = "pid_drive_config.txt"
    );

    /**
     * @brief Run an interactive tuning session for linear motion
     * 
     * @param testDistance Distance to use for testing
     * @param maxVoltage Maximum voltage to use
     */
    void tuneLinearMotion(Length testDistance = 48_in, Number maxVoltage = 12.0);

    /**
     * @brief Run an interactive tuning session for angular motion
     * 
     * @param testAngle Angle to use for testing
     * @param maxVoltage Maximum voltage to use
     */
    void tuneAngularMotion(Angle testAngle = 360_stDeg, Number maxVoltage = 12.0);

    /**
     * @brief Run an interactive tuning session for feedforward parameters
     * 
     * @param testDistance Distance to use for testing
     * @param maxVoltage Maximum voltage to use
     */
    void tuneFeedforward(Length testDistance = 36_in, Number maxVoltage = 12.0);

    /**
     * @brief Test drive to pose with current parameters
     * 
     * @param x X coordinate
     * @param y Y coordinate
     * @param heading Target heading
     * @param maxVoltage Maximum voltage to use
     */
    void testDriveToPose(Length x, Length y, Angle heading, Number maxVoltage = 12.0);

    /**
     * @brief Save current configuration to file
     * 
     * @param filepath Path to save the configuration (uses default if empty)
     * @return true if successful
     */
    bool saveConfiguration(const std::string& filepath = "");

    /**
     * @brief Load configuration from file
     * 
     * @param filepath Path to load the configuration from (uses default if empty)
     * @return true if successful
     */
    bool loadConfiguration(const std::string& filepath = "");

    /**
     * @brief Get the current configuration
     * 
     * @return PIDDriveConfig Current configuration
     */
    control::PIDDriveConfig getCurrentConfig() const;

    /**
     * @brief Set a callback for logging tuning data
     * 
     * @param callback Function to call with telemetry data
     */
    void setTelemetryCallback(std::function<void(const std::string&, double)> callback);

    /**
     * @brief Structure to hold performance metrics data
     */
    struct PerformanceMetrics {
        Time settlingTime = 0_msec;
        Length maxLinearError = 0_in;
        Angle maxAngularError = 0_stDeg;
        double overshoot;
        double steadyStateError;
    };

    /**
     * @brief Structure to hold PID tuning data points
     */
    struct TuningDataPoint {
        double kp;
        double ki;
        double kd;
        double score;
        PerformanceMetrics metrics;
    };

private:
    // The controller being tuned
    control::PIDDriveController& m_controller;
    
    // Path for saving/loading configurations
    std::string m_configFilePath;
    
    // Current configuration parameters
    control::PIDDriveConfig m_currentConfig;
    
    // Telemetry callback
    std::function<void(const std::string&, double)> m_telemetryCallback;
    
    // Data collection for performance analysis
    std::vector<double> m_errorHistory;
    std::vector<double> m_timeHistory;
    Number maxVoltage = 12.0;
    
    /**
     * @brief Run a test with the current parameters
     * 
     * @param testType Type of test (linear, angular, pose)
     * @param parameters Test parameters
     * @return PerformanceMetrics Performance data from the test
     */
    PerformanceMetrics runTest(const std::string& testType, const std::vector<double>& parameters);
    
    /**
     * @brief Update the controller with new parameters
     * 
     * @param config New configuration
     */
    void updateControllerParameters(const control::PIDDriveConfig& config);
    
    /**
     * @brief Display tuning options and handle user input
     * 
     * @param parameterType Type of parameters being tuned
     */
    void displayTuningInterface(const std::string& parameterType);
    
    /**
     * @brief Log performance data after a test
     * 
     * @param metrics Performance metrics to log
     * @param parameters Parameters used for the test
     */
    void logPerformanceData(const PerformanceMetrics& metrics, const std::vector<double>& parameters);

    /**
     * @brief Generate a search space of parameter values around a base value
     * 
     * @param baseValue Center value
     * @param minValue Minimum allowed value
     * @param maxValue Maximum allowed value
     * @param points Number of points to generate
     * @return std::vector<double> Generated values
     */
    std::vector<double> generateSearchSpace(double baseValue, double minValue, double maxValue, int points);
};

void tuneLinearPID();
void tuneAngularPID();

} // namespace tuning
