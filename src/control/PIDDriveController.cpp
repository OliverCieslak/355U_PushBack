#include "control/PIDDriveController.hpp"
#include <algorithm>
#include <cmath>

namespace control {

PIDDriveController::PIDDriveController(
    lemlib::MotorGroup& leftMotors,
    lemlib::MotorGroup& rightMotors,
    const PIDDriveConfig& config,
    std::function<units::Pose()> poseProvider
) : m_leftMotors(leftMotors),
    m_rightMotors(rightMotors),
    m_config(config),
    m_poseProvider(poseProvider),
    m_linearController(config.linearKp, config.linearKi, config.linearKd),
    m_angularController(config.angularKp, config.angularKi, config.angularKd),
    m_previousPose(poseProvider()),  // Initialize previous pose
    m_linearSettleTimer(0_msec),
    m_angularSettleTimer(0_msec),
    m_requiredSettleTime(50_msec) // Default 50ms
{
    m_linearController.setOutputLimits(-12.0, 12.0);
    m_angularController.setOutputLimits(-12.0, 12.0);
}

bool PIDDriveController::driveDistance(Length distance, Number maxVoltage, Time timeout, bool waitUntilSettled) {
    // Initialize motion parameters
    m_linearTarget = distance;
    m_initialHeading = m_poseProvider().orientation;
    m_maxVoltage = maxVoltage;
    m_timeout = timeout;
    m_elapsedTime = 0_sec;
    
    // Reset distance tracking
    m_accumulatedDistance = 0_in;
    m_previousPose = m_poseProvider();
    
    // Set motion type and flags
    m_motionType = MotionType::LINEAR;
    m_isMoving = true;
    
    // Reset controllers
    m_linearController.reset();
    m_angularController.reset();
    
    // Reset action execution flags
    m_actionScheduler.resetActions();
    
    // If waiting, loop until settled or timeout
    if (waitUntilSettled) {
        const Time updateInterval = 10_msec;
        while (update(updateInterval) && m_elapsedTime < m_timeout) {
            pros::delay(10); // 10ms delay (matches updateInterval)
            m_elapsedTime += updateInterval;
        }
        
        return isSettled();
    }
    
    return true;
}

bool PIDDriveController::turnToHeading(Angle targetHeading, Number maxVoltage, Time timeout, bool waitUntilSettled) {
    // Initialize motion parameters
    m_angularTarget = targetHeading;
    m_initialHeading = m_poseProvider().orientation;
    m_maxVoltage = maxVoltage;
    m_timeout = timeout;
    m_elapsedTime = 0_sec;
    
    // Set motion type and flags
    m_motionType = MotionType::ANGULAR;
    m_isMoving = true;
    
    // Reset controllers
    m_linearController.reset();
    m_angularController.reset();
    
    // Reset action execution flags
    m_actionScheduler.resetActions();
    
    // If waiting, loop until settled or timeout
    if (waitUntilSettled) {
        const Time updateInterval = 10_msec;
        while (update(updateInterval) && m_elapsedTime < m_timeout) {
            pros::delay(10); // 10ms delay (matches updateInterval)
            m_elapsedTime += updateInterval;
        }
        
        return isSettled();
    }
    
    return true;
}

bool PIDDriveController::turnAngle(Angle angle, Number maxVoltage, Time timeout, bool waitUntilSettled) {
    // Convert relative angle to absolute heading
    Angle currentHeading = m_poseProvider().orientation;
    return turnToHeading(currentHeading + angle, maxVoltage, timeout, waitUntilSettled);
}

// Update the existing driveToPose method
bool PIDDriveController::driveToPose(units::Pose targetPose, Number maxVoltage, Time timeout, bool waitUntilSettled) {
    // Use the boomerang approach with default parameters
    return driveToPoseBoomerang(targetPose, BoomerangPathConfig(), maxVoltage, timeout, waitUntilSettled);
}

bool PIDDriveController::driveToPoseBoomerang(
    units::Pose targetPose, 
    const BoomerangPathConfig& pathConfig,
    Number maxVoltage, 
    Time timeout,
    bool waitUntilSettled
) {
    // Initialize motion parameters
    m_poseTarget = targetPose;
    m_maxVoltage = maxVoltage;
    m_timeout = timeout;
    m_elapsedTime = 0_sec;
    
    // Set motion type and flags
    m_motionType = MotionType::POSE;
    m_isMoving = true;
    
    // Configure boomerang parameters
    m_boomerangConfig = pathConfig;
    m_isCloseToTarget = false;
    m_prevSameSide = false;
    m_prevLateralOutput = 0.0;
    m_prevAngularOutput = 0.0;
    
    // Reset controllers
    m_linearController.reset();
    m_angularController.reset();
    
    // Reset action execution flags
    m_actionScheduler.resetActions();
    
    // If waiting, loop until settled or timeout
    if (waitUntilSettled) {
        const Time updateInterval = 10_msec;
        while (update(updateInterval) && m_elapsedTime < m_timeout) {
            pros::delay(10); // 10ms delay (matches updateInterval)
            m_elapsedTime += updateInterval;
        }
        
        return isSettled();
    }
    
    return true;
}

bool PIDDriveController::update(Time dt) {
    // If not moving, just return false
    if (!m_isMoving) {
        return false;
    }
    
    // Update elapsed time
    m_elapsedTime += dt;
    
    // Check for timeout
    if (m_elapsedTime >= m_timeout) {
        stop();
        return false;
    }
    
    // Get current pose
    units::Pose currentPose = m_poseProvider();
    
    // Update accumulated distance
    if (m_motionType == MotionType::LINEAR || m_motionType == MotionType::POSE) {
        m_accumulatedDistance += currentPose.distanceTo(m_previousPose);
        m_previousPose = currentPose;
    }
    
    // Process scheduled actions
    // For each motion type, we need to calculate total distance differently
    Length totalDistance = 0_in;
    switch (m_motionType) {
        case MotionType::LINEAR:
            totalDistance = m_linearTarget;
            break;
        case MotionType::ANGULAR:
            // For angular motion, we can't meaningfully use distance triggers
            totalDistance = 0_in;
            break;
        case MotionType::POSE:
            totalDistance = m_previousPose.distanceTo(m_poseTarget);
            break;
        default:
            totalDistance = 0_in;
    }
    
    // Process actions
    m_actionScheduler.processActions(m_elapsedTime, m_timeout, m_accumulatedDistance, totalDistance);
    
    // Variables for motor outputs
    Number leftVoltage = 0.0;
    Number rightVoltage = 0.0;
    
    // Handle different motion types
    switch (m_motionType) {
        case MotionType::LINEAR: {
            // Calculate heading error
            Angle headingError = units::constrainAngle180(currentPose.orientation - m_initialHeading);
            
            // Calculate outputs from PID controllers
            Number linearOutput = m_linearController.calculate(to_in(m_accumulatedDistance), to_in(m_linearTarget), to_msec(m_elapsedTime));
            Number headingCorrection = m_angularController.calculate(to_stDeg(headingError), 0, to_msec(m_elapsedTime));
            // Limit headingCorrection to ±10% of linearOutput
            Number maxCorrection = std::abs(linearOutput) * 0.10;
            if (headingCorrection > maxCorrection) headingCorrection = maxCorrection;
            if (headingCorrection < -maxCorrection) headingCorrection = -maxCorrection;
            
            // Apply feedforward to linear output
            LinearVelocity estimatedVelocity = LinearVelocity(linearOutput / 12.0 * 2.0);
            linearOutput = applyFeedforward(linearOutput, estimatedVelocity);
            
            // Calculate wheel voltages
            leftVoltage = linearOutput - headingCorrection;
            rightVoltage = linearOutput + headingCorrection;
            
            // Check if settled
            if (isLinearSettled(m_accumulatedDistance)) {
                stop();
                return false;
            }
            break;
        }
        
        case MotionType::ANGULAR: {
            // Calculate heading error (constrained to [-180, 180])
            Angle headingError = units::constrainAngle180(currentPose.orientation - m_angularTarget);
            
            // Calculate output from angular PID controller
            Number angularOutput = m_angularController.calculate(to_stDeg(headingError), 0, to_msec(m_elapsedTime));
            
            // Apply angular output to wheels (differential turning)
            leftVoltage = -angularOutput;
            rightVoltage = angularOutput;
            
            // Check if settled
            if (isAngularSettled(currentPose.orientation)) {
                stop();
                return false;
            }
            break;
        }
        
        case MotionType::POSE: {
            // Boomerang control implementation
            
            // Check if the robot is close enough to the target to start settling
            Length distanceToTarget = currentPose.distanceTo(m_poseTarget);
            if (distanceToTarget < m_boomerangConfig.closeDistance && !m_isCloseToTarget) {
                m_isCloseToTarget = true;
                m_boomerangConfig.maxLateralSpeed = std::max(m_prevLateralOutput, Number(0.47));
            }
            
            // Find the carrot point (virtual target)
            units::Pose carrot;
            if (m_isCloseToTarget) {
                // When close, use actual target position
                carrot = m_poseTarget;
            } else {
                // Calculate carrot point behind target
                // We need to find a point that's behind the target along the target's orientation vector
                double offsetX = -to_in(m_boomerangConfig.lead * distanceToTarget) * cos(to_stRad(m_poseTarget.orientation));
                double offsetY = -to_in(m_boomerangConfig.lead * distanceToTarget) * sin(to_stRad(m_poseTarget.orientation));
                
                carrot = units::Pose(
                    m_poseTarget.x + from_in(offsetX), 
                    m_poseTarget.y + from_in(offsetY),
                    m_poseTarget.orientation
                );
            }
            
            // Calculate lateral error
            Length lateralError = [&] {
                Length error = distanceToTarget;
                // Calculate angle between robot orientation and direction to carrot
                Angle headingToCarrot = currentPose.angleTo(carrot);
                Angle headingError = units::constrainAngle180(currentPose.orientation - headingToCarrot);
                
                // Use cosine scaling based on heading error
                Number scalar = cos(to_stRad(headingError));
                
                // Apply scaling differently based on whether we're settling or not
                if (m_isCloseToTarget) {
                    error *= scalar;
                } else {
                    error *= units::sgn(scalar);
                }
                return error;
            }();
            
            // Calculate angular error
            Angle angularError = [&] {
                // Adjust orientation if driving in reverse
                Angle adjustedOrientation = m_boomerangConfig.reversed ? 
                    currentPose.orientation + 180_stDeg : currentPose.orientation;
                
                if (m_isCloseToTarget) {
                    // When close, target the final orientation
                    return units::constrainAngle180(adjustedOrientation - m_poseTarget.orientation);
                } else {
                    // During approach, target the carrot point
                    return units::constrainAngle180(adjustedOrientation - currentPose.angleTo(carrot));
                }
            }();
            
            // Check early exit conditions based on same-side detection
            if (m_isCloseToTarget && m_boomerangConfig.minLateralSpeed != 0) {
                // Check if robot and carrot are on the same side of the target heading line
                // We need to determine which side of a line passing through target with target's orientation
                
                // Create a vector perpendicular to the target's orientation
                double perpX = sin(to_stRad(m_poseTarget.orientation));
                double perpY = -cos(to_stRad(m_poseTarget.orientation));
                
                // Calculate dot product with vector from target to robot
                double robotDotProd = perpX * to_in(currentPose.x - m_poseTarget.x) + 
                                     perpY * to_in(currentPose.y - m_poseTarget.y);
                
                // Calculate dot product with vector from target to carrot
                double carrotDotProd = perpX * to_in(carrot.x - m_poseTarget.x) +
                                       perpY * to_in(carrot.y - m_poseTarget.y);
                
                // Add the early exit range to the carrot dot product for some wiggle room
                carrotDotProd += m_boomerangConfig.earlyExitRange;
                
                // Same side if the dot products have the same sign
                bool robotSide = robotDotProd >= 0;
                bool carrotSide = carrotDotProd >= 0;
                bool sameSide = robotSide == carrotSide;
                
                // Exit if robot and carrot are on opposite sides of the target heading line
                if (!sameSide && m_prevSameSide) {
                    stop();
                    return false;
                }
                
                m_prevSameSide = sameSide;
            }
            
            // Get angular output from PID controller
            Number angularOutput = [&] {
                Number output = m_angularController.calculate(to_stDeg(angularError), 0, to_msec(m_elapsedTime));
                
                // Restrict maximum speed
                output = units::clamp(output / 12.0, -m_boomerangConfig.maxAngularSpeed, m_boomerangConfig.maxAngularSpeed);
                
                // Store for next iteration
                m_prevAngularOutput = output;
                return output;
            }();
            
            // Get lateral output from PID controller
            Number lateralOutput = [&] {
                // Get output from PID
                Number output = m_linearController.calculate(0, to_in(lateralError), to_msec(m_elapsedTime));
                
                // Normalize to -1..1 range and restrict maximum speed
                output = units::clamp(output / 12.0, -m_boomerangConfig.maxLateralSpeed, m_boomerangConfig.maxLateralSpeed);
                                
                // Prevent slipping by limiting speed based on path curvature
                Length radius = 1.0_in / std::max(0.001, std::abs(utils::getSignedTangentArcCurvature(currentPose, carrot)));
                Number maxSlipSpeed = std::sqrt(m_boomerangConfig.driftCompensation * to_in(radius));
                output = units::clamp(output, -maxSlipSpeed, maxSlipSpeed);
                
                // Prioritize angular movement over lateral movement
                Number overturn = std::abs(angularOutput) + std::abs(output) - m_boomerangConfig.maxLateralSpeed;
                if (overturn > 0) {
                    output -= (output > 0) ? overturn : -overturn;
                }
                
                // Handle reversed driving
                if (m_boomerangConfig.reversed && !m_isCloseToTarget) {
                    output = std::min(output, Number(0.0));
                } else if (!m_boomerangConfig.reversed && !m_isCloseToTarget) {
                    output = std::max(output, Number(0.0));
                }
                
                // Enforce minimum speed
                if (m_boomerangConfig.reversed && -output < std::abs(m_boomerangConfig.minLateralSpeed) && output < 0) {
                    output = -std::abs(m_boomerangConfig.minLateralSpeed);
                } else if (!m_boomerangConfig.reversed && output < std::abs(m_boomerangConfig.minLateralSpeed) && output > 0) {
                    output = std::abs(m_boomerangConfig.minLateralSpeed);
                }
                
                // Store for next iteration
                m_prevLateralOutput = output;
                return output;
            }();
            
            // Calculate differential drive outputs
            auto [left, right] = desaturate(lateralOutput, angularOutput);
            
            // Convert normalized -1..1 values to voltages
            leftVoltage = left * m_maxVoltage;
            rightVoltage = right * m_maxVoltage;
            
            // Check if settled at target
            if (isPoseSettled(currentPose)) {
                stop();
                return false;
            }
            break;
        }
        
        case MotionType::NONE:
        default:
            stop();
            return false;
    }
    
    // Apply voltage limits
    leftVoltage = units::clamp(leftVoltage, -m_maxVoltage, m_maxVoltage);
    rightVoltage = units::clamp(rightVoltage, -m_maxVoltage, m_maxVoltage);
    
    // Set motor voltages
    m_leftMotors.move(leftVoltage / 12.0);
    m_rightMotors.move(rightVoltage / 12.0);
    
    // Update previous pose for next distance calculation
    m_previousPose = currentPose;
    
    return true;
}

void PIDDriveController::stop() {
    // Stop motors
    m_leftMotors.move(0);
    m_rightMotors.move(0);
    
    // Reset state
    m_isMoving = false;
    m_motionType = MotionType::NONE;
}

bool PIDDriveController::isMoving() const {
    return m_isMoving;
}

bool PIDDriveController::isSettled() const {
    // If not moving, considered settled
    if (!m_isMoving) {
        return true;
    }
    
    // Get current pose
    units::Pose currentPose = m_poseProvider();
    
    // Check based on motion type
    switch (m_motionType) {
        case MotionType::LINEAR:
            // For linear, use accumulated distance and m_linearTarget
            return const_cast<PIDDriveController*>(this)->isLinearSettled(m_accumulatedDistance);
        case MotionType::ANGULAR:
            // For angular, use current heading and m_angularTarget
            return const_cast<PIDDriveController*>(this)->isAngularSettled(currentPose.orientation);
        case MotionType::POSE:
            return isPoseSettled(currentPose);
        case MotionType::NONE:
        default:
            return true;
    }
}

void PIDDriveController::setLinearGains(double kP, double kI, double kD) {
    m_config.linearKp = kP;
    m_config.linearKi = kI;
    m_config.linearKd = kD;
    m_linearController.setGains(kP, kI, kD);
}

void PIDDriveController::setAngularGains(double kP, double kI, double kD) {
    m_config.angularKp = kP;
    m_config.angularKi = kI;
    m_config.angularKd = kD;
    m_angularController.setGains(kP, kI, kD);
}

void PIDDriveController::setFeedforwardConstants(Number kV, Number kS) {
    m_config.kV = kV;
    m_config.kS = kS;
}

void PIDDriveController::setTolerances(Length linearTolerance, Angle angularTolerance) {
    m_config.linearTolerance = linearTolerance;
    m_config.angularTolerance = angularTolerance;
}

void PIDDriveController::setErrorCallback(std::function<void(double, double)> callback) {
    m_errorCallback = callback;
}

void PIDDriveController::clearErrorCallback() {
    m_errorCallback = nullptr;
}

// New action methods for PIDDriveController

bool PIDDriveController::addAction(std::function<void()> action, ActionTrigger trigger, double value) {
    // Validate value based on trigger type
    if (value < 0) {
        return false; // Negative values don't make sense for any trigger
    }
    
    if (trigger == ActionTrigger::TIME_FROM_END || trigger == ActionTrigger::DISTANCE_FROM_END) {
        // For "from end" triggers, make sure value doesn't exceed motion length
        double maxValue = (trigger == ActionTrigger::TIME_FROM_END) ? 
                          to_sec(m_timeout) : 
                          (m_motionType == MotionType::LINEAR ? to_in(m_linearTarget) : 0.0);
        
        if (value > maxValue) {
            return false; // Value exceeds motion total time/distance
        }
    }
    
    // Add the action to our scheduler
    return m_actionScheduler.addAction(action, trigger, value);
}

bool PIDDriveController::addActionAtTimeFromStart(std::function<void()> action, Time t) {
    return m_actionScheduler.addActionAtTimeFromStart(action, t);
}

bool PIDDriveController::addActionAtTimeFromEnd(std::function<void()> action, Time t) {
    return m_actionScheduler.addActionAtTimeFromEnd(action, t);
}

bool PIDDriveController::addActionAtDistanceFromStart(std::function<void()> action, Length d) {
    return m_actionScheduler.addActionAtDistanceFromStart(action, d);
}

bool PIDDriveController::addActionAtDistanceFromEnd(std::function<void()> action, Length d) {
    return m_actionScheduler.addActionAtDistanceFromEnd(action, d);
}

void PIDDriveController::clearActions() {
    m_actionScheduler.clearActions();
}

// Private helper methods

// Removed misplaced const function signature for isAngularSettled
bool PIDDriveController::isLinearSettled(Length currentDistance) {
    bool withinTolerance = units::abs(m_linearTarget - currentDistance) <= m_config.linearTolerance;
    if (withinTolerance) {
        m_linearSettleTimer += 10_msec; // Assume called every 10ms (update interval)
    } else {
        m_linearSettleTimer = 0_msec;
    }
    return m_linearSettleTimer >= m_requiredSettleTime;
}

bool PIDDriveController::isAngularSettled(Angle currentHeading) {
    bool withinTolerance = units::abs(units::constrainAngle180(currentHeading - m_angularTarget)) <= m_config.angularTolerance;
    if (withinTolerance) {
        m_angularSettleTimer += 10_msec;
    } else {
        m_angularSettleTimer = 0_msec;
    }
    return m_angularSettleTimer >= m_requiredSettleTime;
}

// --- Settling logic ---
// Member variables moved to class definition
bool PIDDriveController::isPoseSettled(const units::Pose& currentPose) const {
    // Check both position and heading
    Length distanceError = currentPose.distanceTo(m_poseTarget);
    Angle headingError = units::abs(units::constrainAngle180(currentPose.orientation - m_poseTarget.orientation));
    
    return (distanceError <= m_config.linearTolerance) && (headingError <= m_config.angularTolerance);
}

Number PIDDriveController::applyFeedforward(Number voltage, LinearVelocity velocity) {
    // Apply feedforward: kS * sgn(v) + kV * v
    Number feedforward = 0.0;
    
    if (velocity != 0.0_inps) {
        feedforward = units::sgn(velocity) * m_config.kS + m_config.kV * to_inps(velocity);
    }
    
    return voltage + feedforward;
}

std::pair<Number, Number> PIDDriveController::desaturate(Number lateral, Number angular) const {
    // Calculate raw motor powers
    Number leftPower = lateral - angular;
    Number rightPower = lateral + angular;
    
    // Find the maximum absolute power
    Number maxPower = std::max(std::abs(leftPower), std::abs(rightPower));
    
    // Desaturate if above 1.0
    if (maxPower > 1.0) {
        leftPower /= maxPower;
        rightPower /= maxPower;
    }
    
    return {leftPower, rightPower};
}

} // namespace control
