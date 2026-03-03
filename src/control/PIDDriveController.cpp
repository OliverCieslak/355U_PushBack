#include "control/PIDDriveController.hpp"
#include <algorithm>
#include <cmath>

namespace control {

PIDDriveController::PIDDriveController(
    lemlib::MotorGroup& leftMotors,
    lemlib::MotorGroup& rightMotors,
    const PIDDriveConfig& config,
    std::function<units::Pose()> poseProvider,
    std::function<std::pair<LinearVelocity, AngularVelocity>()> velocityProvider
) : m_leftMotors(leftMotors),
    m_rightMotors(rightMotors),
    m_config(config),
    m_poseProvider(poseProvider),
    m_velocityProvider(velocityProvider),
    m_linearController(config.linearKp, config.linearKi, config.linearKd),
    m_angularController(config.angularKp, config.angularKi, config.angularKd),
    m_headingController(config.headingKp, config.headingKi, config.headingKd),
    m_previousPose(poseProvider()),  // Initialize previous pose
    m_linearSettleTimer(0_msec),
    m_angularSettleTimer(0_msec),
    m_requiredSettleTime(50_msec) // Default 50ms
{
    m_linearController.setOutputLimits(-12.0, 12.0);
    m_angularController.setOutputLimits(-12.0, 12.0);
    m_headingController.setOutputLimits(-12.0, 12.0);
    
    // Set integral windup limits to prevent overshooting
    m_linearController.setIntegralLimit(50.0);
    m_angularController.setIntegralLimit(30.0);
    m_headingController.setIntegralLimit(30.0);
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
    m_headingController.reset();
    
    // Reset settle timers
    m_linearSettleTimer = 0_msec;
    m_angularSettleTimer = 0_msec;
    
    // Reset action execution flags
    m_actionScheduler.resetActions();
    
    // If waiting, loop until settled or timeout
    if (waitUntilSettled) {
        uint32_t now = pros::millis();
        while (update(10_msec) && m_elapsedTime < m_timeout) {
            pros::Task::delay_until(&now, 10);
        }
        
        return isSettled();
    }
    
    return true;
}

bool PIDDriveController::driveToPoint(const Point& targetPoint, Number maxVoltage, Time timeout, bool reversed, bool waitUntilSettled) {
    // Initialize motion parameters
    m_pointTarget = targetPoint;
    m_maxVoltage = maxVoltage;
    m_timeout = timeout;
    m_elapsedTime = 0_sec;

    // Reset distance tracking
    m_accumulatedDistance = 0_in;
    m_previousPose = m_poseProvider();

    // Set motion type and flags
    m_motionType = MotionType::POINT;
    m_pointReversed = reversed;
    m_isMoving = true;

    // Reset controllers
    m_linearController.reset();
    m_angularController.reset();
    m_headingController.reset();

    // Reset settle timers
    m_linearSettleTimer = 0_msec;
    m_angularSettleTimer = 0_msec;

    // Reset action execution flags
    m_actionScheduler.resetActions();

    if (waitUntilSettled) {
        uint32_t now = pros::millis();
        while (update(10_msec) && m_elapsedTime < m_timeout) {
            pros::Task::delay_until(&now, 10);
        }
        return isSettled();
    }
    return true;
}

bool PIDDriveController::followPath(
    const std::vector<units::Pose>& waypoints,
    Number maxVoltage,
    Time timeout,
    bool reversed,
    Length switchDistance,
    bool waitUntilSettled
) {
    if (waypoints.empty()) return false;

    // Store path state
    m_pathWaypoints = waypoints;
    m_pathIndex = 0;
    m_pathSwitchDistance = switchDistance;
    m_pathReversed = reversed;
    m_pathDoingFinalTurn = false;
    m_maxVoltage = maxVoltage;
    m_timeout = timeout;
    m_elapsedTime = 0_sec;

    // Reset distance tracking
    m_accumulatedDistance = 0_in;
    m_previousPose = m_poseProvider();

    // Set motion type
    m_motionType = MotionType::PATH;
    m_isMoving = true;

    // Reset controllers
    m_linearController.reset();
    m_angularController.reset();
    m_headingController.reset();

    // Reset settle timers
    m_linearSettleTimer = 0_msec;
    m_angularSettleTimer = 0_msec;

    // Reset action execution flags
    m_actionScheduler.resetActions();

    if (waitUntilSettled) {
        uint32_t now = pros::millis();
        while (update(10_msec) && m_elapsedTime < m_timeout) {
            pros::Task::delay_until(&now, 10);
        }
        return !m_isMoving; // true if we finished (not timed out while still moving)
    }
    return true;
}

bool PIDDriveController::turnToHeading(
    Angle targetHeading,
    Number maxVoltage,
    Time timeout,
    bool waitUntilSettled,
    TurnMode turnMode
) {
    // Initialize motion parameters
    m_angularTarget = targetHeading;
    m_initialHeading = m_poseProvider().orientation;
    m_maxVoltage = maxVoltage;
    m_timeout = timeout;
    m_elapsedTime = 0_sec;

    m_turnMode = turnMode;
    
    // Set motion type and flags
    m_motionType = MotionType::ANGULAR;
    m_isMoving = true;
    
    // Reset controllers
    m_linearController.reset();
    m_angularController.reset();
    m_headingController.reset();
    
    // Reset settle timers
    m_linearSettleTimer = 0_msec;
    m_angularSettleTimer = 0_msec;
    
    // Reset action execution flags
    m_actionScheduler.resetActions();

    // If waiting, loop until settled or timeout
    if (waitUntilSettled) {
        uint32_t now = pros::millis();
        while (update(10_msec) && m_elapsedTime < m_timeout) {
            pros::Task::delay_until(&now, 10);
        }
        
        return isSettled();
    }
    
    return true;
}

bool PIDDriveController::turnAngle(Angle angle, Number maxVoltage, Time timeout, bool waitUntilSettled, TurnMode turnMode) {
    // Convert relative angle to absolute heading
    Angle currentHeading = m_poseProvider().orientation;
    return turnToHeading(currentHeading + angle, maxVoltage, timeout, waitUntilSettled, turnMode);
}

bool PIDDriveController::turnToPoint(const Point& targetPoint, Number maxVoltage, Time timeout, bool waitUntilSettled, TurnMode turnMode) {
    m_turnToPointActive = true;
    m_turnToPointReversed = false;
    m_turnToPointTarget = targetPoint;
    units::Pose currentPose = m_poseProvider();
    units::Pose targetPose{targetPoint.x, targetPoint.y, 0_stRad};
    Angle headingToTarget = currentPose.angleTo(targetPose);
    bool result = turnToHeading(headingToTarget, maxVoltage, timeout, waitUntilSettled, turnMode);
    m_turnToPointActive = false;
    return result;
}

bool PIDDriveController::turnAwayFromPoint(const Point& targetPoint, Number maxVoltage, Time timeout, bool waitUntilSettled, TurnMode turnMode) {
    m_turnToPointActive = true;
    m_turnToPointReversed = true;
    m_turnToPointTarget = targetPoint;
    units::Pose currentPose = m_poseProvider();
    units::Pose targetPose{targetPoint.x, targetPoint.y, 0_stRad};
    Angle headingToTarget = currentPose.angleTo(targetPose) + 180_stDeg;
    bool result = turnToHeading(headingToTarget, maxVoltage, timeout, waitUntilSettled, turnMode);
    m_turnToPointActive = false;
    return result;
}

// Update the existing driveToPose method
bool PIDDriveController::driveToPose(units::Pose targetPose, Number maxVoltage, Time timeout, bool reversed, bool waitUntilSettled) {
    // Use the boomerang approach with default parameters
    return driveToPoseBoomerang(targetPose, BoomerangPathConfig(0.5, reversed), maxVoltage, timeout, waitUntilSettled);
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
    m_headingController.reset();
    
    // Reset settle timers
    m_linearSettleTimer = 0_msec;
    m_angularSettleTimer = 0_msec;
    
    // Reset action execution flags
    m_actionScheduler.resetActions();
    
    // If waiting, loop until settled or timeout
    if (waitUntilSettled) {
        uint32_t now = pros::millis();
        while (update(10_msec) && m_elapsedTime < m_timeout) {
            pros::Task::delay_until(&now, 10);
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
    if (m_motionType == MotionType::LINEAR || m_motionType == MotionType::POSE || m_motionType == MotionType::POINT || m_motionType == MotionType::PATH) {
        m_accumulatedDistance += currentPose.distanceTo(m_previousPose);
        m_previousPose = currentPose;
    }
    
    // Process scheduled actions
    // For each motion type, we need to calculate total distance differently
    Length totalDistance = 0_in;
    switch (m_motionType) {
        case MotionType::LINEAR:
            // Use magnitude for distance-based triggers
            totalDistance = units::abs(m_linearTarget);
            break;
        case MotionType::ANGULAR:
            // For angular motion, we can't meaningfully use distance triggers
            totalDistance = 0_in;
            break;
        case MotionType::POSE:
            totalDistance = m_previousPose.distanceTo(m_poseTarget);
            break;
        case MotionType::POINT: {
            units::Pose ptPose{m_pointTarget.x, m_pointTarget.y, 0_stRad};
            totalDistance = m_previousPose.distanceTo(ptPose);
            break;
        }
        case MotionType::PATH:
            if (!m_pathWaypoints.empty()) {
                totalDistance = m_previousPose.distanceTo(m_pathWaypoints.back());
            }
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

            Length signedDistance_err = units::sgn(m_linearTarget) * m_accumulatedDistance;
            Length linearError = m_linearTarget - signedDistance_err;
            Number linearOutput = m_linearController.calculate(-to_in(linearError), 0.0, to_msec(m_elapsedTime));

            Number headingCorrection = m_headingController.calculate(to_stDeg(headingError), 0, to_msec(m_elapsedTime));
            // Limit headingCorrection to ±50% of linearOutput
            Number maxCorrection = std::abs(linearOutput) * 0.50;
            if (headingCorrection > maxCorrection) headingCorrection = maxCorrection;
            if (headingCorrection < -maxCorrection) headingCorrection = -maxCorrection;
            
            // Calculate wheel voltages
            leftVoltage = linearOutput - headingCorrection;
            rightVoltage = linearOutput + headingCorrection;
            
            // Check if settled
            Length signedDistance = units::sgn(m_linearTarget) * m_accumulatedDistance;
            if (isLinearSettled(signedDistance)) {
                stop();
                return false;
            }
            break;
        }
        
        case MotionType::ANGULAR: {
            // If tracking a point, recalculate target heading each iteration
            if (m_turnToPointActive) {
                units::Pose pointPose{m_turnToPointTarget.x, m_turnToPointTarget.y, 0_stRad};
                m_angularTarget = currentPose.angleTo(pointPose);
                if (m_turnToPointReversed) m_angularTarget += 180_stDeg;
            }
            // Calculate heading error (constrained to [-180, 180])
            // Positive error means we should turn in the positive direction.
            Angle headingError = units::constrainAngle180(m_angularTarget - currentPose.orientation);

            Number angularOutput = m_angularController.calculate(-to_stDeg(headingError), 0.0, to_msec(m_elapsedTime));

            // Tapered static friction feedforward
            {
                double absErr = std::abs(to_stDeg(headingError));
                constexpr double kS_fadeStart = 3.0;
                double kS_scale = std::min(absErr / kS_fadeStart, 1.0);
                if (angularOutput != 0.0) {
                    angularOutput += units::sgn(angularOutput) * m_config.kS * kS_scale;
                }
            }
            
            // Apply angular output to wheels (differential turning)
            switch (m_turnMode) {
                case TurnMode::TANK:
                    leftVoltage = -angularOutput;
                    rightVoltage = angularOutput;
                    break;
                case TurnMode::SWING_LEFT:
                    // Pivot about the right side (right stationary)
                    leftVoltage = -2.0 * angularOutput;
                    rightVoltage = 0.0;
                    break;
                case TurnMode::SWING_RIGHT:
                    // Pivot about the left side (left stationary)
                    leftVoltage = 0.0;
                    rightVoltage = 2.0 * angularOutput;
                    break;
                default:
                    leftVoltage = -angularOutput;
                    rightVoltage = angularOutput;
                    break;
            }
            
            // Check if settled
            if (isAngularSettled(currentPose.orientation)) {
                stop();
                return false;
            }
            break;
        }
        
        case MotionType::POINT: {
            // Blend of driveDistance and turnToHeading using current odometry
            units::Pose currentPose = m_poseProvider();

            // Direction to target point and distance
            units::Pose targetPose{m_pointTarget.x, m_pointTarget.y, 0_stRad};
            Angle headingToTarget = currentPose.angleTo(targetPose);
            Length distanceToTarget = currentPose.distanceTo(targetPose);

            // Errors
            Angle adjustedOrientation = m_pointReversed ? (currentPose.orientation + 180_stDeg) : currentPose.orientation;
            Angle angularError = units::constrainAngle180(adjustedOrientation - headingToTarget);
            Length linearError = distanceToTarget * units::cos(angularError);

            Number linearOutput = m_linearController.calculate(-to_in(linearError), 0.0, to_msec(m_elapsedTime));

            // ×2 gain boost for driveToPoint to overcome static friction
            linearOutput *= 2.0;

            if (m_pointReversed) linearOutput = -linearOutput;

            // Limit approach speed when close to prevent momentum overshoot.
            {
                constexpr double rampStartIn = 18.0;
                constexpr double minFraction = 0.15;
                double distIn = std::abs(to_in(distanceToTarget));
                if (distIn < rampStartIn) {
                    double fraction = minFraction + (1.0 - minFraction) * (distIn / rampStartIn);
                    Number maxOutput = m_maxVoltage * fraction;
                    linearOutput = units::clamp(linearOutput, -maxOutput, maxOutput);
                }
            }

            // Velocity damping near target
            if (distanceToTarget < 8_in && m_velocityProvider) {
                LinearVelocity vel = m_velocityProvider().first;
                Number damping = m_config.kV * to_inps(vel) * 3.0;
                linearOutput -= damping;
            }

            // Tapered static friction (kS) feedforward
            {
                double kS_scale = std::min(to_in(distanceToTarget) / 2.0, 1.0);
                if (linearOutput != 0.0) {
                    linearOutput += units::sgn(linearOutput) * m_config.kS * kS_scale;
                }
            }

            Number headingCorrection = m_headingController.calculate(to_stDeg(angularError), 0, to_msec(m_elapsedTime));

            // Add sign-aware static feedforward to heading correction
            if (headingCorrection != 0.0) {
                headingCorrection += units::sgn(headingCorrection) * m_config.kS;
            }

            // Limit heading correction relative to linear output to prevent oscillation.
            double absAngErr = std::abs(to_stDeg(angularError));
            Number capRatio = (distanceToTarget < 6_in) ? 0.15 : 0.40;
            if (absAngErr < 15.0) {
                capRatio *= (absAngErr / 15.0);
                capRatio = std::max(capRatio, Number(0.05));
            }
            Number maxCorrection = std::abs(linearOutput) * capRatio;
            headingCorrection = units::clamp(headingCorrection, -maxCorrection, maxCorrection);

            // Normalize and desaturate
            Number lateralCmd = units::clamp(linearOutput / m_maxVoltage, -1.0, 1.0);
            Number angularCmd = units::clamp(headingCorrection / m_maxVoltage, -1.0, 1.0);
            auto [leftCmd, rightCmd] = desaturate(lateralCmd, angularCmd);

            leftVoltage = leftCmd * m_maxVoltage;
            rightVoltage = rightCmd * m_maxVoltage;

            // Consider settled when within linear tolerance of point for required settle time
            if (isPointSettled(currentPose)) {
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
                if (m_isCloseToTarget) {
                    // When close, settle to final heading WITHOUT reverse offset
                    return units::constrainAngle180(currentPose.orientation - m_poseTarget.orientation);
                } else {
                    // During approach, adjust for reverse so robot's back faces the carrot
                    Angle adjustedOrientation = m_boomerangConfig.reversed ? 
                        currentPose.orientation + 180_stDeg : currentPose.orientation;
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
                // Deadband: ignore heading errors under 3° to prevent the PID's
                // D-term from amplifying IMU noise and carrot-point jitter into
                // rapid left/right oscillations during reverse approach.
                double absAngErrDeg = std::abs(to_stDeg(angularError));
                if (absAngErrDeg < 3.0) {
                    m_prevAngularOutput = 0.0;
                    return Number(0.0);
                }

                Number output = m_headingController.calculate(to_stDeg(angularError), 0, to_msec(m_elapsedTime));
                
                // Boost heading output for curve tracking (headingKp alone is too gentle)
                // Use less boost when close to target to prevent whip-around on final heading match.
                // Reverse motions need a gentler boost — aggressive correction makes the
                // angular output compete with minLateralSpeed, causing motors to flip sign
                // and the robot to jitter instead of driving smoothly.
                double boost;
                if (m_isCloseToTarget) {
                    boost = 1.2;
                } else if (m_boomerangConfig.reversed) {
                    boost = 1.2;
                } else {
                    boost = 2.5;
                }
                output *= boost;
                
                // Restrict maximum speed
                output = units::clamp(output / 12.0, -m_boomerangConfig.maxAngularSpeed, m_boomerangConfig.maxAngularSpeed);
                
                // Store for next iteration
                m_prevAngularOutput = output;
                return output;
            }();
            
            // Get lateral output from PID controller
            Number lateralOutput = [&] {
                // Get output from PID
                Number output = m_linearController.calculate(-to_in(lateralError), 0.0, to_msec(m_elapsedTime));
                
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
                
                // Enforce minimum speed only during approach — once close to
                // target, let the PID decelerate freely so it can actually settle.
                // Without this gate the robot hits the target at 60% power and
                // overshoots because it can never slow below minLateralSpeed.
                if (!m_isCloseToTarget) {
                    if (m_boomerangConfig.reversed && -output < std::abs(m_boomerangConfig.minLateralSpeed) && output < 0) {
                        output = -std::abs(m_boomerangConfig.minLateralSpeed);
                    } else if (!m_boomerangConfig.reversed && output < std::abs(m_boomerangConfig.minLateralSpeed) && output > 0) {
                        output = std::abs(m_boomerangConfig.minLateralSpeed);
                    }
                }
                
                // Store for next iteration
                m_prevLateralOutput = output;
                return output;
            }();
            
            // For reverse: cap angular correction so desaturated motors don't
            // flip signs (one forward, one backward = jittery pivoting).
            // Keep angular ≤ 35% of lateral so both wheels drive the same direction
            // with only a gentle steering bias.
            if (m_boomerangConfig.reversed && !m_isCloseToTarget) {
                Number maxAng = std::abs(lateralOutput) * 0.35;
                angularOutput = units::clamp(angularOutput, -maxAng, maxAng);
            }

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

        case MotionType::PATH: {
            // --- Multi-waypoint arc follower with pose waypoints ---
            // If doing a final turn-in-place after reaching the last point:
            if (m_pathDoingFinalTurn) {
                Angle headingError = units::constrainAngle180(m_angularTarget - currentPose.orientation);
                Number angularOutput = m_angularController.calculate(-to_stDeg(headingError), 0.0, to_msec(m_elapsedTime));
                leftVoltage = -angularOutput;
                rightVoltage = angularOutput;
                if (isAngularSettled(currentPose.orientation)) {
                    stop();
                    return false;
                }
                break;
            }

            // Current target waypoint (a full Pose)
            const units::Pose& target = m_pathWaypoints[m_pathIndex];
            bool isLastWaypoint = (m_pathIndex == m_pathWaypoints.size() - 1);

            // Distance and heading to current target
            Length distToTarget = currentPose.distanceTo(target);
            Angle headingToTarget = currentPose.angleTo(target);

            // Advance to next waypoint if close enough (unless it's the last one)
            if (!isLastWaypoint && distToTarget <= m_pathSwitchDistance) {
                m_pathIndex++;
                // Reset angular PID to prevent derivative kick on target switch
                m_angularController.reset();
                break; // re-evaluate on next cycle with new target
            }

            // Heading error (flip 180° if driving in reverse)
            Angle adjustedOrientation = m_pathReversed
                ? (currentPose.orientation + 180_stDeg)
                : currentPose.orientation;

            // Blend toward waypoint's desired orientation as we get closer.
            // Far away: steer purely toward the point.  Close: bias toward
            // the waypoint's specified heading (if one was given).
            bool waypointHasHeading = (std::abs(to_stDeg(target.orientation)) < 900.0);
            Angle desiredHeading = headingToTarget; // default: aim at point

            if (waypointHasHeading) {
                // Blend factor: 1.0 when at switchDistance, 0.0 when far away
                Number blend = 1.0 - units::clamp(
                    to_in(distToTarget) / to_in(m_pathSwitchDistance), 0.0, 1.0);
                // Weighted average of "aim at point" and "desired orientation"
                Angle orientationDelta = units::constrainAngle180(target.orientation - headingToTarget);
                desiredHeading = headingToTarget + orientationDelta * blend;
            }

            Angle angularError = units::constrainAngle180(adjustedOrientation - desiredHeading);

            // Linear output — PID drives distance to zero
            Number linearOutput = m_linearController.calculate(-to_in(distToTarget), 0.0, to_msec(m_elapsedTime));
            linearOutput = m_pathReversed ? -std::abs(linearOutput) : std::abs(linearOutput);

            // For non-last waypoints, enforce a minimum forward speed so the
            // robot doesn't stop and do a point turn at each waypoint
            if (!isLastWaypoint) {
                Number minSpeed = m_maxVoltage * 0.25; // 25% forward minimum
                if (!m_pathReversed && linearOutput < minSpeed) linearOutput = minSpeed;
                if (m_pathReversed && linearOutput > -minSpeed) linearOutput = -minSpeed;
            }

            // Angular output — PID steers toward target
            Number headingCorrection = m_angularController.calculate(to_stDeg(angularError), 0, to_msec(m_elapsedTime));

            // When close to the last waypoint, limit heading correction
            if (isLastWaypoint && distToTarget < 6_in) {
                Number maxCorr = std::abs(linearOutput) * 0.15;
                headingCorrection = units::clamp(headingCorrection, -maxCorr, maxCorr);
            }

            // Measured velocity for feedforward - way better than the old voltage-based guess
            LinearVelocity estVel = 0_inps;
            if (m_velocityProvider) {
                estVel = m_velocityProvider().first;
            }
            linearOutput = applyFeedforward(linearOutput, estVel);

            // Desaturate and scale to voltage
            Number latCmd = units::clamp(linearOutput / m_maxVoltage, -1.0, 1.0);
            Number angCmd = units::clamp(headingCorrection / m_maxVoltage, -1.0, 1.0);
            auto [lCmd, rCmd] = desaturate(latCmd, angCmd);
            leftVoltage = lCmd * m_maxVoltage;
            rightVoltage = rCmd * m_maxVoltage;

            // Check if we've reached the final waypoint
            if (isLastWaypoint && distToTarget <= m_config.linearTolerance) {
                m_linearSettleTimer += 10_msec;
                if (m_linearSettleTimer >= m_requiredSettleTime) {
                    // Check if the last waypoint wants a specific final heading
                    if (std::abs(to_stDeg(target.orientation)) < 900.0) {
                        // Switch to final turn mode
                        m_pathDoingFinalTurn = true;
                        m_angularTarget = target.orientation;
                        m_angularController.reset();
                        m_angularSettleTimer = 0_msec;
                        break;
                    }
                    stop();
                    return false;
                }
            } else {
                m_linearSettleTimer = 0_msec;
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
        case MotionType::POINT:
            return const_cast<PIDDriveController*>(this)->isPointSettled(currentPose);
        case MotionType::PATH: {
            // PATH settles when update() sets m_isMoving = false via stop()
            // If doing final turn, check angular settle
            if (m_pathDoingFinalTurn) {
                return const_cast<PIDDriveController*>(this)->isAngularSettled(currentPose.orientation);
            }
            // Otherwise check if we're at the last waypoint
            if (!m_pathWaypoints.empty()) {
                return currentPose.distanceTo(m_pathWaypoints.back()) <= m_config.linearTolerance;
            }
            return true;
        }
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

void PIDDriveController::setHeadingCorrectionGains(double kP, double kI, double kD) {
    m_config.headingKp = kP;
    m_config.headingKi = kI;
    m_config.headingKd = kD;
    m_headingController.setGains(kP, kI, kD);
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
    
    // Use slightly looser tolerances than driveDistance — boomerang has to
    // satisfy both position AND heading simultaneously, which is harder.
    // The lateral PID with Kp=0.5 stalls ~1.3in from target when friction
    // varies, so 1.5in tolerance avoids intermittent timeouts.
    Length poseTolerance = m_config.linearTolerance * 3.0;   // 1.5in
    Angle headingTolerance = m_config.angularTolerance * 3.0; // 3°
    
    bool withinTolerance = (distanceError <= poseTolerance) && (headingError <= headingTolerance);
    
    // Use a settle timer so the robot must stay within tolerance, not just
    // pass through on a single sample.
    if (withinTolerance) {
        const_cast<PIDDriveController*>(this)->m_angularSettleTimer += 10_msec;
    } else {
        const_cast<PIDDriveController*>(this)->m_angularSettleTimer = 0_msec;
    }
    return const_cast<PIDDriveController*>(this)->m_angularSettleTimer >= 50_msec;
}

bool PIDDriveController::isPointSettled(const units::Pose& currentPose) {
    Length distanceError = currentPose.distanceTo(units::Pose{m_pointTarget.x, m_pointTarget.y, 0_stRad});
    // Use a looser tolerance than driveDistance — driveToPoint has heading error
    // involved and small offsets are corrected by subsequent movements.  The
    // tighter linearTolerance (0.5in) causes intermittent timeouts when friction
    // variation stalls the robot 0.5-0.6in from target.
    Length pointTolerance = m_config.linearTolerance * 2.0; // 1.0in with default config
    bool withinTolerance = distanceError <= pointTolerance;
    if (withinTolerance) {
        m_linearSettleTimer += 10_msec; // mirror linear settle semantics
    } else {
        m_linearSettleTimer = 0_msec;
    }
    // Require 150ms within tolerance (vs 50ms for driveDistance).
    // The wider zone means the robot can enter it while still carrying
    // momentum; 150ms ensures it has actually decelerated and stopped
    // rather than flying through.
    return m_linearSettleTimer >= 150_msec;
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
