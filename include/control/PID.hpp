#pragma once

#include <algorithm>
#include <cmath>

namespace control {

// Fallback implementation of clamp for older C++ standards
template<typename T>
T clamp(const T& value, const T& low, const T& high) {
    return std::max(low, std::min(value, high));
}

/**
 * @brief A simplified PID controller class that uses regular doubles
 */
class PID {
private:
    // Gains
    double kP; // Proportional gain
    double kI; // Integral gain
    double kD; // Derivative gain
    double kV; // Velocity feedforward gain
    double kS; // Static feedforward gain

    // Controller state
    double previousError;
    double errorSum;
    double lastInput;
    double previousTime;
    double lastSetpoint;
    
    // Constraints
    double minOutput;
    double maxOutput;
    double maxErrorSum;
    double errorDeadband;
    bool hasBounds;
    bool atSetpointFlag;

public:
    /**
     * @brief Construct a new PID controller
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kV Velocity feedforward gain
     * @param kS Static feedforward gain
     */
    PID(double kP, double kI, double kD, double kV = 0.0, double kS = 0.0)
        : kP(kP), kI(kI), kD(kD), kV(kV), kS(kS),
          previousError(0), errorSum(0), lastInput(0), previousTime(0),
          lastSetpoint(0), hasBounds(false), maxErrorSum(0), errorDeadband(0),
          atSetpointFlag(false) {}
    
    /**
     * @brief Reset the controller state
     */
    void reset() {
        errorSum = 0.0;
        previousError = 0.0;
        lastInput = 0.0;
        previousTime = 0.0;
        lastSetpoint = 0.0;
        atSetpointFlag = false;
    }
    
    /**
     * @brief Set output boundaries
     * 
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setOutputLimits(double min, double max) {
        minOutput = min;
        maxOutput = max;
        hasBounds = true;
    }
    
    /**
     * @brief Set integral windup limit
     * 
     * @param limit Maximum absolute value for error sum
     */
    void setIntegralLimit(double limit) {
        maxErrorSum = std::abs(limit);
    }
    
    /**
     * @brief Set error deadband
     * 
     * @param deadband Error values within ±deadband will be considered zero
     */
    void setErrorDeadband(double deadband) {
        errorDeadband = std::abs(deadband);
    }
    
    /**
     * @brief Calculate PID output based on current measurement and setpoint
     * 
     * @param currentValue Current measured value
     * @param setpoint Target value
     * @param currentTime Current time in seconds
     * @param feedforward Additional feedforward value (adds to kF calculation)
     * @return Output value
     */
    double calculate(double currentValue, double setpoint, double currentTime, double feedforward = 0.0) {
        // Calculate time delta
        double dt = currentTime - previousTime;
        
        // Skip first calculation if time isn't initialized
        if (previousTime == 0) {
            previousTime = currentTime;
            lastInput = currentValue;
            lastSetpoint = setpoint;
            previousError = setpoint - currentValue;
            return 0.0;
        }
        
        // Calculate error
        double error = setpoint - currentValue;
        
        // Apply error deadband if configured
        if (std::abs(error) < errorDeadband) {
            error = 0.0;
            atSetpointFlag = true;
        } else {
            atSetpointFlag = false;
        }
        
        // Proportional term
        double pTerm = error * kP;
        
        // Integral term - only accumulate if we're not at deadband
        if (error != 0.0) {
            errorSum += error * dt;
        }
        
        // Apply integral windup limit if set
        if (maxErrorSum > 0) {
            errorSum = control::clamp(errorSum, -maxErrorSum, maxErrorSum);
        }
        
        double iTerm = errorSum * kI;
        
        // Derivative term (on measurement, not error, to avoid derivative kick)
        double dInput = (currentValue - lastInput) / dt;
        double dTerm = -dInput * kD; // Note: negative because dInput goes opposite to error
        
        // Feedforward term from setpoint rate of change and static component
        double setpointDerivative = (setpoint - lastSetpoint) / dt;
        double ffTerm = setpointDerivative * kV + kS + feedforward;
        
        // Calculate total output
        double output = pTerm + iTerm + dTerm + ffTerm;
        
        // Apply output constraints if set
        if (hasBounds) {
            output = control::clamp(output, minOutput, maxOutput);
        }
        
        // Update state for next iteration
        previousError = error;
        lastInput = currentValue;
        lastSetpoint = setpoint;
        previousTime = currentTime;
        
        return output;
    }
    
    /**
     * @brief Update controller gains
     * 
     * @param kP New proportional gain
     * @param kI New integral gain
     * @param kD New derivative gain
     * @param kV New velocity feedforward gain
     * @param kS New static feedforward gain
     */
    void setGains(double kP, double kI, double kD, double kV = -1.0, double kS = -1.0) {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
        if (kV >= 0.0) this->kV = kV;
        if (kS >= 0.0) this->kS = kS;
    }
    
    /**
     * @brief Get the proportional gain
     */
    double getKp() const {
        return kP;
    }
    
    /**
     * @brief Get the integral gain
     */
    double getKi() const {
        return kI;
    }
    
    /**
     * @brief Get the derivative gain
     */
    double getKd() const {
        return kD;
    }
    
    /**
     * @brief Get the velocity feedforward gain
     */
    double getKv() const {
        return kV;
    }
    
    /**
     * @brief Get the static feedforward gain
     */
    double getKs() const {
        return kS;
    }
    
    /**
     * @brief Set the velocity feedforward gain
     */
    void setKv(double kV) {
        this->kV = kV;
    }
    
    /**
     * @brief Set the static feedforward gain
     */
    void setKs(double kS) {
        this->kS = kS;
    }
    
    /**
     * @brief Check if controller is at the setpoint
     * 
     * @return True if error is within deadband
     */
    bool atSetpoint() const {
        return atSetpointFlag;
    }
};

} // namespace control
