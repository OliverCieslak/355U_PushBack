#include "tuning/PIDDriveControllerTuner.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

extern tuning::CharacterizationView characterizationView;
extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;
extern LinearVelocity maxVelocity;
extern Length trackWidth;
extern Length wheelDiameter;
extern Number kS;  // Static friction (volts)
extern Number kV;  // Velocity feedforward (volts per velocity)
extern Number kA;  // Acceleration feedforward (volts per acceleration)
extern double linearKp;
extern double linearKi;
extern double linearKd;
extern double angularKp;
extern double angularKi;
extern double angularKd;
extern control::PIDDriveController pidDriveController;

namespace tuning {

PIDDriveControllerTuner::PIDDriveControllerTuner(
    control::PIDDriveController& controller,
    const std::string& configFilePath
) : m_controller(controller),
    m_configFilePath(configFilePath),
    m_currentConfig(getCurrentConfig())
{
    // Initialize default telemetry callback
    m_telemetryCallback = [](const std::string& name, double value) {
        std::cout << name << ": " << value << std::endl;
    };
}

void PIDDriveControllerTuner::tuneLinearMotion(Length testDistance, Number maxVoltage) {
    std::cout << "=== Automated Linear Motion Tuning ===" << std::endl;
    std::cout << "Test Distance: " << to_in(testDistance) << " inches" << std::endl;
    std::cout << "Optimizing parameters to minimize error and settling time..." << std::endl;
    
    // Starting parameters
    double bestKp = m_currentConfig.linearKp;
    double bestKi = m_currentConfig.linearKi;
    double bestKd = m_currentConfig.linearKd;
    double bestScore = std::numeric_limits<double>::max();
    
    // Parameter ranges to test
    std::vector<double> kpValues = generateSearchSpace(bestKp, 0.01, 10.0, 10);
    std::vector<double> kiValues = generateSearchSpace(bestKi, 0.0, 1.0, 8);
    std::vector<double> kdValues = generateSearchSpace(bestKd, 0.0, 5.0, 8);
    
    // Track progress
    int totalTests = kpValues.size() * kiValues.size() * kdValues.size();
    int testsCompleted = 0;
    
    std::vector<TuningDataPoint> dataPoints;
    Time optimalTime = testDistance / maxVelocity;

    // Grid search for best parameters
    for (double kp : kpValues) {
        for (double ki : kiValues) {
            for (double kd : kdValues) {
                // Update controller with test parameters
                m_controller.setLinearGains(kp, ki, kd);
                // pidDriveTunerView.updateLinearGains(kp, ki, kd);
                
                // Run the test and measure performance
                auto metrics = runTest("linear", {to_in(testDistance)});
                                
                // Calculate score (weighted sum of settling time and error)
                // Lower score is better
                double timeWeight = 0.6;  // Weight for settling time
                double errorWeight = 0.4;  // Weight for error
                double score = timeWeight * metrics.settlingTime / optimalTime + 
                               errorWeight * metrics.maxLinearError / testDistance;
                
                // Store data point
                dataPoints.push_back({kp, ki, kd, score, metrics});
                
                // Update if this is the best score so far
                if (score < bestScore) {
                    bestScore = score;
                    bestKp = kp;
                    bestKi = ki;
                    bestKd = kd;
                    
                    std::cout << "New Best Parameters for Linear" << std::endl;
                    std::cout << "Kp: " << bestKp << ", Ki: " << bestKi << ", Kd: " << bestKd << std::endl;
                }
                
                // Update progress
                testsCompleted++;
                int percentComplete = (testsCompleted * 100) / totalTests;
                
                // Give the robot time to reset before trying to drive back to where it started
                pros::delay(500);

                // Drive the robot back to its starting position after each test
                m_controller.driveDistance(-testDistance, maxVoltage, 30_sec, true);

                // Give the robot time to reset between tests
                pros::delay(500);
            }
        }
    }
    
    // Set the best parameters found
    m_currentConfig.linearKp = bestKp;
    m_currentConfig.linearKi = bestKi;
    m_currentConfig.linearKd = bestKd;
    m_controller.setLinearGains(bestKp, bestKi, bestKd);
    // pidDriveTunerView.updateLinearGains(bestKp, bestKi, bestKd);
    
    std::cout << "Tuning complete. Best parameters found:" << std::endl;
    std::cout << "Kp: " << bestKp << ", Ki: " << bestKi << ", Kd: " << bestKd << std::endl;
    std::cout << "Score: " << bestScore << std::endl;
    
    // Optional: run one final test with the best parameters
    runTest("linear", {to_in(testDistance)});
    
    // Save the results
    saveConfiguration();
}

void PIDDriveControllerTuner::tuneAngularMotion(Angle testAngle, Number maxVoltage) {
    std::cout << "=== Automated Angular Motion Tuning ===" << std::endl;
    std::cout << "Test Angle: " << to_stDeg(testAngle) << " degrees" << std::endl;
    std::cout << "Optimizing parameters to minimize error and settling time..." << std::endl;
    
    // Starting parameters
    double bestKp = m_currentConfig.angularKp;
    double bestKi = m_currentConfig.angularKi;
    double bestKd = m_currentConfig.angularKd;
    double bestScore = std::numeric_limits<double>::max();
    
    // Parameter ranges to test
    std::vector<double> kpValues = generateSearchSpace(bestKp, 0.01, 5.0, 10);
    std::vector<double> kiValues = generateSearchSpace(bestKi, 0.0, 0.5, 8);
    std::vector<double> kdValues = generateSearchSpace(bestKd, 0.0, 2.0, 8);
    
    // Track progress
    int totalTests = kpValues.size() * kiValues.size() * kdValues.size();
    int testsCompleted = 0;
    
    std::vector<TuningDataPoint> dataPoints;
    
    // Calculate the optimal time for the test, assuming the robot immediately reaches max speed
    // and is a skid-steer robot, so the turning speed is based on the trackWidth and maxVelocity
    Time optimalTime = to_stRad(testAngle) / (2 * maxVelocity / trackWidth);

    // Grid search for best parameters
    for (double kp : kpValues) {
        for (double ki : kiValues) {
            for (double kd : kdValues) {
                // Update controller with test parameters
                m_controller.setAngularGains(kp, ki, kd);
                // pidDriveTunerView.updateAngularGains(kp, ki, kd);
                
                // Run the test and measure performance
                auto metrics = runTest("angular", {to_stDeg(testAngle)});
                
                // Calculate score (weighted sum of settling time and error)
                double timeWeight = 0.6;
                double errorWeight = 0.4;
                double score = timeWeight * metrics.settlingTime / optimalTime + 
                               errorWeight * (to_stDeg(metrics.maxAngularError) / to_stDeg(testAngle));
                
                // Store data point
                dataPoints.push_back({kp, ki, kd, score, metrics});
                
                // Update if this is the best score so far
                if (score < bestScore) {
                    bestScore = score;
                    bestKp = kp;
                    bestKi = ki;
                    bestKd = kd;
                    
                    std::cout << "New Best Parameters for Angular" << std::endl;
                    std::cout << "Kp: " << bestKp << ", Ki: " << bestKi << ", Kd: " << bestKd << std::endl;
                }
                
                // Update progress
                testsCompleted++;
                int percentComplete = (testsCompleted * 100) / totalTests;
                
                // Give the robot time to reset between tests
                pros::delay(500);
            }
        }
    }
    
    // Set the best parameters found
    m_currentConfig.angularKp = bestKp;
    m_currentConfig.angularKi = bestKi;
    m_currentConfig.angularKd = bestKd;
    m_controller.setAngularGains(bestKp, bestKi, bestKd);
    // pidDriveTunerView.updateAngularGains(bestKp, bestKi, bestKd);
    
    std::cout << "Tuning complete. Best parameters found:" << std::endl;
    std::cout << "Kp: " << bestKp << ", Ki: " << bestKi << ", Kd: " << bestKd << std::endl;
    std::cout << "Score: " << bestScore << std::endl;
    
    // Optional: run one final test with the best parameters
    runTest("angular", {to_stDeg(testAngle)});
    
    // Save the results
    saveConfiguration();
}

void PIDDriveControllerTuner::tuneFeedforward(Length testDistance, Number maxVoltage) {
    std::cout << "=== Automated Feedforward Tuning ===" << std::endl;
    std::cout << "Test Distance: " << to_in(testDistance) << " inches" << std::endl;
    
    // Measure kS (static friction coefficient) first
    std::cout << "Measuring static friction coefficient (kS)..." << std::endl;
    FeedforwardTuner feedforwardTuner;
    feedforwardTuner.setWheelDiameter(wheelDiameter);
    
    // Add data collection callbacks
    feedforwardTuner.addVelocityDataCallback(
        [this](double voltage, double velocity) {
            characterizationView.addVelocityDataPoint(voltage, velocity);
            m_telemetryCallback("voltage", voltage);
            m_telemetryCallback("velocity", velocity);
        }
    );
    
    // Measure kS
    Number kS = feedforwardTuner.tuneKs();
    m_currentConfig.kS = kS;
    
    // Now measure kV (velocity coefficient)
    std::cout << "Measuring velocity coefficient (kV)..." << std::endl;
    Number kV = feedforwardTuner.tuneKv(kS);
    m_currentConfig.kV = kV;
    
    // Update the controller
    m_controller.setFeedforwardConstants(kV, kS);
    // pidDriveTunerView.updateFeedforwardConstants(kV, kS);
    
    std::cout << "Feedforward tuning complete." << std::endl;
    std::cout << "kS: " << kS << std::endl;
    std::cout << "kV: " << kV << std::endl;
    
    // Test the feedforward parameters
    runTest("feedforward", {to_in(testDistance)});
    
    // Save the results
    saveConfiguration();
}

void PIDDriveControllerTuner::testDriveToPose(Length x, Length y, Angle heading, Number maxVoltage) {
    std::cout << "Testing drive to pose..." << std::endl;
    std::cout << "Target: X=" << to_in(x) << " in, Y=" << to_in(y)
              << " in, Heading=" << to_cDeg(heading) << " deg" << std::endl;

    auto startTime = std::chrono::steady_clock::now();
    
    units::Pose targetPose(x, y, heading);
    bool success = m_controller.driveToPose(targetPose, maxVoltage, 10_sec, true);
    
    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    
    std::cout << "Test completed in " << elapsed.count() << " seconds." << std::endl;
    std::cout << "Drive to pose " << (success ? "succeeded" : "failed") << std::endl;
    
    // Note: Cannot access current pose directly. A method like getCurrentPose()
    // needs to be added to the PIDDriveController class to calculate pose errors.
    std::cout << "Position and heading error calculation not available." << std::endl;
}

PIDDriveControllerTuner::PerformanceMetrics PIDDriveControllerTuner::runTest(
    const std::string& testType, 
    const std::vector<double>& parameters
) {
    PerformanceMetrics metrics;
    metrics.maxLinearError = 0_in;
    metrics.maxAngularError = 0_stDeg;
    metrics.overshoot = 0.0;
    metrics.settlingTime = 0_sec;
    metrics.steadyStateError = 0.0;
    
    // Reset data collection
    m_errorHistory.clear();
    m_timeHistory.clear();
    
    auto startTime = pros::micros();
    bool result = false;
    
    // Set up error tracking callback
    auto errorCallback = [this, &metrics, testType](double error, double time) {
        m_errorHistory.push_back(error);
        m_timeHistory.push_back(time);
        
        // Track maximum error based on test type
        if (testType == "linear") {
            if (std::abs(error) > to_in(metrics.maxLinearError)) {
                metrics.maxLinearError = from_in(std::abs(error));
            }
        } else if (testType == "angular") {
            if (std::abs(error) > to_stDeg(metrics.maxAngularError)) {
                metrics.maxAngularError = from_stDeg(std::abs(error));
            }
        }
    };
    
    // Register callback with controller
    m_controller.setErrorCallback(errorCallback);
    
    if (testType == "linear") {
        Length distance = from_in(parameters[0]);
        std::cout << "\nRunning linear test with distance: " << to_in(distance) << " inches" << std::endl;
        
        result = m_controller.driveDistance(distance, maxVoltage, 30_sec, true);
    } 
    else if (testType == "angular") {
        Angle angle = from_stDeg(parameters[0]);
        std::cout << "\nRunning angular test with angle: " << to_stDeg(angle) << " degrees" << std::endl;
        
        result = m_controller.turnAngle(angle, maxVoltage, 30_sec, true);
    }
    else if (testType == "feedforward") {
        Length distance = from_in(parameters[0]);
        std::cout << "\nRunning feedforward test with distance: " << to_in(distance) << " inches" << std::endl;
        
        result = m_controller.driveDistance(distance, maxVoltage, 30_sec, true);
    }
    
    auto endTime = pros::micros();
    metrics.settlingTime = from_usec(endTime - startTime);
    
    // Calculate overshoot and steady-state error from collected data
    if (!m_errorHistory.empty()) {
        // Find maximum error (for overshoot calculation)
        double maxErrorValue = 0.0;
        for (double error : m_errorHistory) {
            if (std::abs(error) > std::abs(maxErrorValue)) {
                maxErrorValue = error;
            }
        }
        
        // Calculate overshoot as percentage
        double targetValue = parameters[0]; // First parameter is the target
        metrics.overshoot = (maxErrorValue / targetValue) * 100.0;
        
        // Calculate steady-state error (average of last few samples)
        int steadyStateWindow = std::min(10, static_cast<int>(m_errorHistory.size()));
        double sumError = 0.0;
        for (int i = m_errorHistory.size() - steadyStateWindow; i < m_errorHistory.size(); i++) {
            sumError += std::abs(m_errorHistory[i]);
        }
        metrics.steadyStateError = sumError / steadyStateWindow;
    }
    
    std::cout << "Test " << (result ? "succeeded" : "failed") << " in " 
              << to_sec(metrics.settlingTime) << " seconds" << std::endl;
    std::cout << "Max Error: " << to_in(metrics.maxLinearError) << " inches" << std::endl;
    std::cout << "Overshoot: " << metrics.overshoot << "%" << std::endl;
    std::cout << "Steady-state Error: " << metrics.steadyStateError << std::endl;
    
    // Unregister callback
    m_controller.clearErrorCallback();
    
    return metrics;
}

bool PIDDriveControllerTuner::saveConfiguration(const std::string& filepath) {
    // Check if SD card is installed
    if (!pros::usd::is_installed()) {
        characterizationView.showStatusMessage("Warning", "SD card not found, values not saved");
        return false;
    }
    // Override path to SD card file if using default path
    std::string path = filepath.empty() ? "/usd/pid_drive_config.txt" : filepath;
    
    try {
        std::ofstream file(path);
        if (!file.is_open()) {
            std::cerr << "Could not open file for writing: " << path << std::endl;
            return false;
        }
        
        // Write linear PID
        file << "linearKp=" << m_currentConfig.linearKp << std::endl;
        file << "linearKi=" << m_currentConfig.linearKi << std::endl;
        file << "linearKd=" << m_currentConfig.linearKd << std::endl;
        
        // Write angular PID
        file << "angularKp=" << m_currentConfig.angularKp << std::endl;
        file << "angularKi=" << m_currentConfig.angularKi << std::endl;
        file << "angularKd=" << m_currentConfig.angularKd << std::endl;
        
        // Write feedforward
        file << "kV=" << m_currentConfig.kV << std::endl;
        file << "kS=" << m_currentConfig.kS << std::endl;
                
        file.close();
        
        characterizationView.showStatusMessage("Success", ("Configuration saved to " + path).c_str());
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error saving configuration: " << e.what() << std::endl;
        return false;
    }
}

bool PIDDriveControllerTuner::loadConfiguration(const std::string& filepath) {
    // Check if SD card is installed
    if (!pros::usd::is_installed()) {
        characterizationView.showStatusMessage("Warning", "SD card not found, loading configuration skipped");
        return false;
    }
    // Override path to SD card file if using default path
    std::string path = filepath.empty() ? "/usd/pid_drive_config.txt" : filepath;
    
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Could not open file: " << path << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty()) continue;
            
            // Find key-value separator
            size_t pos = line.find('=');
            if (pos == std::string::npos) continue;
            
            std::string key = line.substr(0, pos);
            std::string valueStr = line.substr(pos + 1);
            double value = std::stod(valueStr);
            
            // Parse key-value pairs
            if (key == "linearKp") {
                m_currentConfig.linearKp = value;
            } else if (key == "linearKi") {
                m_currentConfig.linearKi = value;
            } else if (key == "linearKd") {
                m_currentConfig.linearKd = value;
            } else if (key == "angularKp") {
                m_currentConfig.angularKp = value;
            } else if (key == "angularKi") {
                m_currentConfig.angularKi = value;
            } else if (key == "angularKd") {
                m_currentConfig.angularKd = value;
            } else if (key == "kV") {
                m_currentConfig.kV = value;
            } else if (key == "kS") {
                m_currentConfig.kS = value;
            }
        }
        
        file.close();
        
        // Update controller parameters
        updateControllerParameters(m_currentConfig);
        
        characterizationView.showStatusMessage("Success", ("Configuration loaded from " + path).c_str());
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading configuration: " << e.what() << std::endl;
        return false;
    }
}

control::PIDDriveConfig PIDDriveControllerTuner::getCurrentConfig() const {
    return m_currentConfig;
}

void PIDDriveControllerTuner::setTelemetryCallback(std::function<void(const std::string&, double)> callback) {
    if (callback) {
        m_telemetryCallback = callback;
    }
}

void PIDDriveControllerTuner::updateControllerParameters(const control::PIDDriveConfig& config) {
    m_controller.setLinearGains(
        config.linearKp,
        config.linearKi,
        config.linearKd
    );
    
    m_controller.setAngularGains(
        config.angularKp,
        config.angularKi,
        config.angularKd
    );
    
    m_controller.setFeedforwardConstants(
        config.kV,
        config.kS
    );
    
    m_controller.setTolerances(
        config.linearTolerance,
        config.angularTolerance
    );
}

void PIDDriveControllerTuner::logPerformanceData(
    const PerformanceMetrics& metrics, 
    const std::vector<double>& parameters
) {
    m_telemetryCallback("settling_time_ms", to_msec(metrics.settlingTime));
    m_telemetryCallback("max_error", to_in(metrics.maxLinearError));
    m_telemetryCallback("overshoot_percent", metrics.overshoot);
    m_telemetryCallback("steady_state_error", metrics.steadyStateError);
    
    for (size_t i = 0; i < parameters.size(); i++) {
        m_telemetryCallback("parameter_" + std::to_string(i), parameters[i]);
    }
}

void PIDDriveControllerTuner::displayTuningInterface(const std::string& parameterType) {
    std::cout << "=== Tuning " << parameterType << " Parameters ===" << std::endl;
    // This method would be implemented with a more sophisticated interface
    // potentially platform-specific UI elements or a terminal-based menu system
}

std::vector<double> PIDDriveControllerTuner::generateSearchSpace(
    double baseValue, double minValue, double maxValue, int points) {
    
    std::vector<double> values;
    
    // Include the base value
    values.push_back(baseValue);
    
    // Add values around the base value, with higher density near the base
    double range = maxValue - minValue;
    double stepSize = range / (points - 1);
    
    for (int i = 0; i < points; i++) {
        double value = minValue + i * stepSize;
        if (std::abs(value - baseValue) > 0.001) {  // Avoid duplicating base value
            values.push_back(value);
        }
    }
    
    return values;
}

void tuneLinearPID() {
    PIDDriveControllerTuner tuner(pidDriveController);
    tuner.tuneLinearMotion(24_in, 12.0);
}

void tuneAngularPID() {
    PIDDriveControllerTuner tuner(pidDriveController);
    tuner.tuneAngularMotion(360_stDeg, 12.0);
}
} // namespace tuning
