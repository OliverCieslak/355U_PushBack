#include "tuning/FeedforwardTuner.hpp"
#include "api.h"
#include <cmath>
#include <vector>
#include <utility>
#include <iostream>

extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;

namespace tuning {

FeedforwardTuner::FeedforwardTuner()
{
    // Initialize with empty callback vectors
}

void FeedforwardTuner::setWheelDiameter(Length diameter) {
    m_wheelDiameter = diameter;
}

void FeedforwardTuner::addVelocityDataCallback(std::function<void(double, double)> callback) {
    if (callback) {
        m_velocityDataCallbacks.push_back(callback);
    }
}

void FeedforwardTuner::addAccelerationDataCallback(std::function<void(double, double)> callback) {
    if (callback) {
        m_accelerationDataCallbacks.push_back(callback);
    }
}

void FeedforwardTuner::clearCallbacks() {
    m_velocityDataCallbacks.clear();
    m_accelerationDataCallbacks.clear();
}

Number FeedforwardTuner::tuneKs() {
    // Set motor brake modes
    leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
    rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);

    // Variables needed for kS tuning
    Number kS = 0.0;  // Will be updated by the tuning process
    int characterizationStep = 0;
    bool characterizingComplete = false;
    
    std::cout << "Starting kS calibration" << std::endl;
    std::cout << "SAFETY: Max voltage limited to 10.0V, timeout after 15 seconds" << std::endl;
    
    // Safety limits
    const double MAX_VOLTAGE = 10.0;  // Limit voltage to prevent robot from going too fast
    const int MAX_STEPS = 500;        // 500 * 0.02V = 10.0V max
    const uint32_t TIMEOUT_MS = 15000; // 15 second timeout
    uint32_t startTime = pros::millis();
    
    // Test loop
    while (!characterizingComplete && characterizationStep < MAX_STEPS) {
        // Safety timeout check
        if (pros::millis() - startTime > TIMEOUT_MS) {
            std::cout << "SAFETY: Timeout reached, stopping calibration" << std::endl;
            break;
        }
        
        // Slowly increase voltage until movement is detected
        double voltage = characterizationStep * 0.02;      // Increase by 20mV each step (was 100mV)
        
        // Safety limit
        if (voltage > MAX_VOLTAGE) {
            std::cout << "SAFETY: Max voltage reached without detecting movement" << std::endl;
            break;
        }
        
        int voltageInt = static_cast<int>(voltage * 1000); // Display in mV
        
        // Convert voltage to percentage (assuming 12V max)
        double percentOutput = voltage / 12.0;
        leftMotors.move(percentOutput);
        rightMotors.move(percentOutput);

        // Get wheel velocities
        double leftVel = prosLeftMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
        double rightVel = prosRightMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
        double avgVel = (std::abs(leftVel) + std::abs(rightVel)) / 2.0;

        // Add data point for visualization to all registered callbacks
        for (const auto& callback : m_velocityDataCallbacks) {
            callback(voltage, avgVel);
        }
        
        // Update display every 500ms to avoid flicker
        if (pros::millis() % 500 == 10) {
            std::cout << "V: " << voltageInt << " mV, Vel: " << avgVel << " in/s" << std::endl;
        }
        
        // If we detect movement, we've found kS
        if (avgVel > 0.5) { // Consider movement detected when velocity > 0.5 in/s
            kS = voltage * 0.9; // Slightly reduce the estimate to be conservative
            std::cout << "Movement detected! kS: " << kS << " V" << std::endl;
            
            leftMotors.move(0);
            rightMotors.move(0);
            
            characterizingComplete = true;
        }
        
        characterizationStep++;
        pros::delay(20);
    }
    
    // Ensure motors are stopped
    leftMotors.move(0);
    rightMotors.move(0);
    
    // Display final result
    if (kS > 0) {
        std::cout << "kS Calibration completed successfully" << std::endl;
        std::cout << "kS: " << kS << " V" << std::endl;
    } else {
        std::cout << "kS Calibration failed - no movement detected within safety limits" << std::endl;
        kS = 0.5; // Default fallback value
        std::cout << "Using fallback kS: " << kS << " V" << std::endl;
    }
    
    return kS;
}

Number FeedforwardTuner::tuneKv(Number kS) {
    // Set motor brake modes
    leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
    rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);

    // Variables needed for kV tuning
    Number kV = 0.0;
    bool characterizingComplete = false;
    std::vector<std::pair<double, double>> velocityData;  // Voltage, Velocity pairs
    
    std::cout << "Starting kV calibration" << std::endl;
    std::cout << "kS: " << kS << " V" << std::endl;
    std::cout << "SAFETY: Max voltage limited to 6.0V, 1.5s per test" << std::endl;
    
    // Safety-limited test voltages (reduced from original)
    const std::vector<double> testVoltages = {1.0, 1.25, 1.75};
    const double MAX_APPLIED_VOLTAGE = 8.0;  // Maximum total voltage including kS compensation
    const uint32_t TEST_DURATION_MS = 1000;
    const uint32_t TIMEOUT_MS = 30000;       // 30 second total timeout
    
    uint32_t startTime = pros::millis();
    
    for (double voltage : testVoltages) {
        if (characterizingComplete) break;
        
        // Safety timeout check
        if (pros::millis() - startTime > TIMEOUT_MS) {
            std::cout << "SAFETY: Total timeout reached, stopping calibration" << std::endl;
            break;
        }
        
        // Apply voltage (compensating for static friction)
        double appliedVoltage = voltage + kS;
        
        // Safety limit on applied voltage
        if (appliedVoltage > MAX_APPLIED_VOLTAGE) {
            std::cout << "SAFETY: Skipping " << voltage << "V test - would exceed max voltage limit" << std::endl;
            continue;
        }
        
        if (appliedVoltage > 12.0) appliedVoltage = 12.0;  // Cap at max voltage
        
        // Convert voltage to percentage (assuming 12V max)
        double percentOutput = appliedVoltage / 12.0;
        leftMotors.move(percentOutput);
        rightMotors.move(percentOutput);
        
        // Wait for steady state velocity (reduced duration for safety)
        std::cout << "Testing: " << voltage << " V (applied: " << appliedVoltage << " V)" << std::endl;
        pros::delay(TEST_DURATION_MS);  // Allow motors to reach steady state
        
        // Sample velocity multiple times and average
        double avgVelocity = 0.0;
        int samples = 5;
        
        for (int i = 0; i < samples; i++) {
            double leftVel = prosLeftMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
            double rightVel = prosRightMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
            avgVelocity += (std::abs(leftVel) + std::abs(rightVel)) / 2.0;
            pros::delay(20);
        }
        
        avgVelocity /= samples;
        
        // Safety check - if robot is going too fast, stop the test
        if (avgVelocity > 60.0) {  // 60 in/s = 5 ft/s is plenty fast enough
            std::cout << "SAFETY: Robot moving too fast (" << avgVelocity << " in/s), stopping test" << std::endl;
            leftMotors.move(0);
            rightMotors.move(0);
            break;
        }
        
        // Record data point
        velocityData.push_back({voltage, avgVelocity});
        
        // Add data point to all registered callbacks
        for (const auto& callback : m_velocityDataCallbacks) {
            callback(voltage, avgVelocity);
        }
        
        std::cout << "V: " << voltage << " V, Vel: " << avgVelocity << " in/s" << std::endl;
        
        // Stop motors before next test
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(500);  // Brief pause between tests
    }
    
    // Ensure motors are stopped
    leftMotors.move(0);
    rightMotors.move(0);
    
    // Extract x and y values from pairs
    std::vector<double> xValues;
    std::vector<double> yValues;
    for (const auto& pair : velocityData) {
        xValues.push_back(pair.first);
        yValues.push_back(pair.second);
    }
    
    if (xValues.size() >= 3) {  // Need at least 3 points for reasonable regression
        double slope = utils::calculateLinearRegressionSlope(xValues, yValues);
        std::cout << "[Debug] Raw regression slope (velocity/volt): " << slope << std::endl;
        if (slope != 0) {
            kV = 1.0 / slope;
            std::cout << "kV Calibration completed successfully" << std::endl;
            std::cout << "kV: " << kV << " V·s/in" << std::endl;
        } else {
            std::cout << "kV Calibration failed - invalid slope calculated" << std::endl;
            kV = 0.01; // Default fallback value
            std::cout << "Using fallback kV: " << kV << " V·s/in" << std::endl;
        }
    } else {
        std::cout << "kV Calibration failed - not enough data points" << std::endl;
        kV = 0.01; // Default fallback value
        std::cout << "Using fallback kV: " << kV << " V·s/in" << std::endl;
    }
    return kV;
}

Number FeedforwardTuner::tuneKa(Number kS, Number kV) {
    // Set motor brake modes
    leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
    rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);

    // Variables needed for kA tuning
    Number kA = 0.0;
    bool characterizingComplete = false;
    std::vector<std::pair<double, double>> accelerationData;  // Acceleration, Voltage pairs
    
    std::cout << "Starting kA calibration" << std::endl;
    std::cout << "kS: " << kS << " V, kV: " << kV << " V·s/in" << std::endl;
    
    // Test different voltage levels to find voltage-acceleration relationship (limit to 4V max for safety)
    const std::vector<double> testVoltages = {2.0, 3.0, 4.0, 5.0, 6.0};
    
    for (double voltage : testVoltages) {
        if (characterizingComplete) break;
        
        // Reset robot to rest before each test
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(1000);  // Wait for robot to stop completely
        
        // Prepare to measure initial velocity
        double initialVelocity = 0.0;
        for (int i = 0; i < 5; i++) {
            double leftVel = prosLeftMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
            double rightVel = prosRightMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
            initialVelocity += (std::abs(leftVel) + std::abs(rightVel)) / 2.0;
            pros::delay(20);
        }
        initialVelocity /= 5;
        
        // Apply test voltage
        std::cout << "Testing: " << voltage << " V" << std::endl;
        
        // Apply full test voltage - convert to percentage (assuming 12V max)
        double percentOutput = voltage / 12.0;
        leftMotors.move(percentOutput);
        rightMotors.move(percentOutput);
        
        // Measure time and velocity for acceleration calculation
        uint32_t startTime = pros::millis();
        pros::delay(500);  // Accelerate for 500ms
        uint32_t endTime = pros::millis();
        double elapsedTime = (endTime - startTime) / 1000.0;  // Convert to seconds
        
        // Measure final velocity
        double finalVelocity = 0.0;
        for (int i = 0; i < 5; i++) {
            double leftVel = prosLeftMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
            double rightVel = prosRightMotors.get_actual_velocity() * to_in(m_wheelDiameter) * M_PI / 60.0;
            finalVelocity += (std::abs(leftVel) + std::abs(rightVel)) / 2.0;
            pros::delay(20);
        }
        finalVelocity /= 5;
        
        // Calculate acceleration
        double acceleration = (finalVelocity - initialVelocity) / elapsedTime;
        
        // Calculate the voltage component due to acceleration
        // Total voltage = kS + kV * velocity + kA * acceleration
        // We know voltage, kS, kV, and average velocity, so we can solve for kA's portion
        double avgVelocity = (initialVelocity + finalVelocity) / 2.0;
        double accelVoltage = voltage - kS - (kV * avgVelocity);
        
        // Only accept positive acceleration values to avoid noise
        if (acceleration > 0.1) {
            // Record data point (acceleration voltage vs. acceleration)
            accelerationData.push_back({acceleration, accelVoltage});
            
            // Add data point to all registered callbacks
            for (const auto& callback : m_accelerationDataCallbacks) {
                callback(acceleration, accelVoltage);
            }
            
            std::cout << "Accel: " << acceleration << " in/s², Volt: " << accelVoltage << " V" << std::endl;
        }
        
        // Stop motors
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(1000);  // Wait before next test
    }
    
    // Extract x and y values from pairs
    std::vector<double> xValues;
    std::vector<double> yValues;
    for (const auto& pair : accelerationData) {
        xValues.push_back(pair.first);
        yValues.push_back(pair.second);
    }
    
    kA = utils::calculateLinearRegressionSlope(xValues, yValues);
    std::cout << "kA Calibration completed" << std::endl;
    std::cout << "[kA Measured] Value: " << kA << " V·s²/in (may be negative if data is limited)" << std::endl;
    return kA;
}

} // namespace tuning
