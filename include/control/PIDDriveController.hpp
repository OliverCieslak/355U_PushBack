#pragma once

#include "control/ActionScheduler.hpp"
#include "control/PID.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "utils/DistanceUtils.hpp"
#include <functional>
#include <memory>

namespace control {

/**
 * @brief Configuration for a PID-based differential drive controller
 */
struct PIDDriveConfig {
    // Robot physical parameters
    Length trackWidth;           // Distance between left and right wheels
    Length wheelDiameter;        // Diameter of wheels
    
    // Linear motion PID gains
    double linearKp;             // Proportional gain for linear motion
    double linearKi;             // Integral gain for linear motion
    double linearKd;             // Derivative gain for linear motion
    
    // Angular motion PID gains
    double angularKp;            // Proportional gain for angular motion
    double angularKi;            // Integral gain for angular motion
    double angularKd;            // Derivative gain for angular motion
    
    // Feedforward constants
    Number kV;                   // Velocity feedforward constant (volts per velocity)
    Number kS;                   // Static friction feedforward constant (volts)
    
    // Tolerances for considering motion complete
    Length linearTolerance;      // Linear position tolerance
    Angle angularTolerance;      // Angular position tolerance

    /**
     * @brief Construct a PID drive controller configuration with default values
     */
    PIDDriveConfig(
        Length trackWidth,
        Length wheelDiameter,
        double linearKp = 1.0,
        double linearKi = 0.0,
        double linearKd = 0.0,
        double angularKp = 1.0,
        double angularKi = 0.0,
        double angularKd = 0.0,
        Number kV = 0.0,
        Number kS = 0.0,
        Length linearTolerance = .5_in,
        Angle angularTolerance = 2_stDeg
    ) : trackWidth(trackWidth), wheelDiameter(wheelDiameter),
        linearKp(linearKp), linearKi(linearKi), linearKd(linearKd),
        angularKp(angularKp), angularKi(angularKi), angularKd(angularKd),
        kV(kV), kS(kS),
        linearTolerance(linearTolerance), angularTolerance(angularTolerance) {}
};

/**
 * @brief Configuration parameters for boomerang path control
 */
struct BoomerangPathConfig {
    // Path generation parameters
    Number lead = 0.5;           // Lead parameter determining path curvature (0-1)
    bool reversed = false;       // Whether to drive backward to target
    
    // Speed control parameters
    Number maxLateralSpeed = 1.0;  // Maximum lateral speed (0-1)
    Number minLateralSpeed = 0.1;  // Minimum lateral speed (0-1)
    Number maxAngularSpeed = 1.0;  // Maximum angular speed (0-1)
    
    // Control parameters
    Number earlyExitRange = 0.0;   // Early exit range offset
    Number driftCompensation = 0.5; // Compensation factor for drifting in turns
    Length closeDistance = 7.5_in;  // Distance to start settling phase

    /**
     * @brief Construct a boomerang path configuration with default values
     * 
     * @param lead Path curvature parameter (0-1, higher = more curved)
     * @param reversed Whether to drive backward to target
     */
    BoomerangPathConfig(Number lead = 0.5, bool reversed = false) 
        : lead(lead), reversed(reversed) {}
};

/**
 * @brief PID-based controller for differential drive robots
 * 
 * This class provides methods to control a differential drive robot using PID controllers
 * for both linear and angular motion.
 */
class PIDDriveController {
public:
    // Use the ActionTrigger enum from ActionScheduler
    using ActionTrigger = control::ActionTrigger;
    // Alias for a 2D point (x, y) in field coordinates
    using Point = units::Vector2D<Length>;

    /**
     * @brief Construct a new PID Drive Controller
     * 
     * @param leftMotors Left side motor group
     * @param rightMotors Right side motor group
     * @param config PID drive configuration
     * @param poseProvider Function that returns the current robot pose
     */
    PIDDriveController(
        lemlib::MotorGroup& leftMotors,
        lemlib::MotorGroup& rightMotors,
        const PIDDriveConfig& config,
        std::function<units::Pose()> poseProvider
    );

    /**
     * @brief Drive forward/backward a specified distance
     * 
     * @param distance Distance to drive (positive = forward, negative = backward)
     * @param maxVoltage Maximum voltage to apply to motors
     * @param timeout Maximum time to spend attempting to reach target
     * @param waitUntilSettled Whether to block until motion is complete
     * @return true if target was reached within timeout
     */
    bool driveDistance(
        Length distance,
        Number maxVoltage = 12.0,
        Time timeout = 5_sec,
        bool waitUntilSettled = true
    );
    
    /**
     * @brief Turn to a specified absolute heading
     * 
     * @param targetHeading Target absolute heading
     * @param maxVoltage Maximum voltage to apply to motors
     * @param timeout Maximum time to spend attempting to reach target
     * @param waitUntilSettled Whether to block until motion is complete
     * @return true if target was reached within timeout
     */
    bool turnToHeading(
        Angle targetHeading,
        Number maxVoltage = 12.0,
        Time timeout = 3_sec,
        bool waitUntilSettled = true
    );
    
    /**
     * @brief Turn by a relative angle
     * 
     * @param angle Angle to turn (positive = counterclockwise, negative = clockwise)
     * @param maxVoltage Maximum voltage to apply to motors
     * @param timeout Maximum time to spend attempting to reach target
     * @param waitUntilSettled Whether to block until motion is complete
     * @return true if target was reached within timeout
     */
    bool turnAngle(
        Angle angle,
        Number maxVoltage = 12.0,
        Time timeout = 3_sec,
        bool waitUntilSettled = true
    );
    
    /**
     * @brief Drive to a specified pose (position and heading)
     * 
     * Uses boomerang control to create a smooth path.
     * 
     * @param targetPose Target pose
     * @param maxVoltage Maximum voltage to apply to motors
     * @param timeout Maximum time to spend attempting to reach target
     * @param waitUntilSettled Whether to block until motion is complete
     * @return true if target was reached within timeout
     */
    bool driveToPose(
        units::Pose targetPose,
        Number maxVoltage = 12.0,
        Time timeout = 10_sec,
        bool reversed = false,
        bool waitUntilSettled = true
    );

    /**
     * @brief Drive to a target point (x, y) using linear and angular PID control
     *
     * This blends the behaviors of driveDistance (for forward motion) and
     * turnToHeading (for steering toward the point). It ignores final heading
     * and considers the motion complete when within linearTolerance of the point.
     *
     * @param targetPoint Target point (field x,y)
     * @param maxVoltage Maximum voltage to apply to motors
     * @param timeout Maximum time to spend attempting to reach target
     * @param waitUntilSettled Whether to block until motion is complete
     * @return true if target was reached within timeout
     */
    bool driveToPoint(
        const Point& targetPoint,
        Number maxVoltage = 12.0,
        Time timeout = 10_sec,
        bool reversed = false,
        bool waitUntilSettled = true
    );

    /**
     * @brief Drive to a specified pose with custom boomerang path parameters
     * 
     * @param targetPose Target pose
     * @param pathConfig Boomerang path configuration
     * @param maxVoltage Maximum voltage to apply to motors
     * @param timeout Maximum time to spend attempting to reach target
     * @param waitUntilSettled Whether to block until motion is complete
     * @return true if target was reached within timeout
     */
    bool driveToPoseBoomerang(
        units::Pose targetPose,
        const BoomerangPathConfig& pathConfig = BoomerangPathConfig(),
        Number maxVoltage = 12.0,
        Time timeout = 10_sec,
        bool waitUntilSettled = true
    );
    
    /**
     * @brief Update the controller, calculating and applying motor outputs
     * 
     * @param dt Time since last update
     * @return true if still moving to target, false if complete or stopped
     */
    bool update(Time dt);
    
    /**
     * @brief Stop all motion immediately
     */
    void stop();
    
    /**
     * @brief Check if the controller is currently moving to a target
     * 
     * @return true if moving to target
     */
    bool isMoving() const;
    
    /**
     * @brief Check if the controller has reached its target
     * 
     * @return true if target is reached
     */
    bool isSettled() const;
    
    /**
     * @brief Set new PID gains for linear motion
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    void setLinearGains(double kP, double kI, double kD);
    
    /**
     * @brief Set new PID gains for angular motion
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    void setAngularGains(double kP, double kI, double kD);
    
    /**
     * @brief Set new feedforward constants
     * 
     * @param kV Velocity feedforward constant
     * @param kS Static friction feedforward constant
     */
    void setFeedforwardConstants(Number kV, Number kS);
    
    /**
     * @brief Set new tolerance values for considering motion complete
     * 
     * @param linearTolerance Linear position tolerance
     * @param angularTolerance Angular position tolerance
     */
    void setTolerances(Length linearTolerance, Angle angularTolerance);
    
    /**
     * @brief Set a callback function to receive error values during controller operation
     * 
     * @param callback The function to call with each error value
     */
    void setErrorCallback(std::function<void(double, double)> callback);

    /**
     * @brief Clear the error callback function
     */
    void clearErrorCallback();

    /**
     * @brief Add an action to be executed during motion
     * 
     * @param action Function to execute (takes no parameters, returns void)
     * @param trigger Type of trigger
     * @param value Trigger value in seconds (for time) or inches (for distance)
     * @return true if action was added successfully
     */
    bool addAction(std::function<void()> action, ActionTrigger trigger, double value);

    /**
     * @brief Add an action to be executed at a specific time from start
     * 
     * @param action Function to execute
     * @param t Time from motion start
     * @return true if action was added successfully
     */
    bool addActionAtTimeFromStart(std::function<void()> action, Time t);

    /**
     * @brief Add an action to be executed at a specific time from end
     * 
     * @param action Function to execute
     * @param t Time before motion end
     * @return true if action was added successfully
     */
    bool addActionAtTimeFromEnd(std::function<void()> action, Time t);

    /**
     * @brief Add an action to be executed at a specific distance from start
     * 
     * @param action Function to execute
     * @param d Distance from motion start
     * @return true if action was added successfully
     */
    bool addActionAtDistanceFromStart(std::function<void()> action, Length d);

    /**
     * @brief Add an action to be executed at a specific distance from end
     * 
     * @param action Function to execute
     * @param d Distance before motion end
     * @return true if action was added successfully
     */
    bool addActionAtDistanceFromEnd(std::function<void()> action, Length d);

    /**
     * @brief Clear all scheduled actions
     */
    void clearActions();

public:
    /**
     * @brief Set the required settle time (ms) for isSettled functions
     */
    void setRequiredSettleTime(Time t) { m_requiredSettleTime = t; }

private:
    // Hardware components
    lemlib::MotorGroup& m_leftMotors;
    lemlib::MotorGroup& m_rightMotors;
    PIDDriveConfig m_config;
    std::function<units::Pose()> m_poseProvider;
    
    // PID controllers
    PID m_linearController;
    PID m_angularController;
    
    // Motion state
    enum class MotionType {
        NONE,
        LINEAR,
    ANGULAR,
    POSE,
    POINT
    };
    MotionType m_motionType = MotionType::NONE;
    bool m_isMoving = false;
    Length m_linearTarget = 0_m;
    Angle m_angularTarget = 0_stRad;
    units::Pose m_poseTarget;
    Length m_initialDistance = 0_m;
    Angle m_initialHeading = 0_stRad;
    // --- Settling logic ---
    bool isLinearSettled(Length currentDistance);
    bool isAngularSettled(Angle currentHeading);
    // --- Settling logic ---
    Time m_linearSettleTimer = 0_msec;
    Time m_angularSettleTimer = 0_msec;
    Time m_requiredSettleTime = 50_msec; // Default to 50ms, configurable
    
    // Control parameters
    Number m_maxVoltage = 12.0;
    Time m_timeout = 0_sec;
    Time m_elapsedTime = 0_sec;
    
    // Distance tracking
    units::Pose m_previousPose;             // Previous robot pose for distance calculation
    Length m_accumulatedDistance = 0_m;     // Total accumulated distance traveled
    Point m_pointTarget;                    // Target point for POINT motion
    
    // Callback for error reporting
    std::function<void(double, double)> m_errorCallback;
    
    // Action scheduler for managing actions during motion
    ActionScheduler m_actionScheduler;
    
    // Static constant for pose blend distance
    static constexpr Length POSE_BLEND_DISTANCE = 3_in;

    // Boomerang control state
    BoomerangPathConfig m_boomerangConfig;
    bool m_isCloseToTarget = false;
    bool m_prevSameSide = false;
    Number m_prevLateralOutput = 0.0;
    Number m_prevAngularOutput = 0.0;
    // POINT mode reverse flag
    bool m_pointReversed = false;
    
    /**
     * @brief Check if the linear motion is settled
     * 
     * @param currentDistance Current linear distance
     * @return true if settled
     */
    bool isLinearSettled(Length currentDistance) const;
    
    /**
     * @brief Check if the angular motion is settled
     * 
     * @param currentHeading Current heading
     * @return true if settled
     */
    bool isAngularSettled(Angle currentHeading) const;
    
    /**
     * @brief Check if the pose motion is settled
     * 
     * @param currentPose Current robot pose
     * @return true if settled
     */
    bool isPoseSettled(const units::Pose& currentPose) const;

    /**
     * @brief Check if the point motion is settled (distance only)
     *
     * Uses linearTolerance and required settle time.
     */
    bool isPointSettled(const units::Pose& currentPose);
    
    /**
     * @brief Apply feedforward to output voltages
     * 
     * @param linearVoltage Base linear voltage
     * @param linearVelocity Current linear velocity
     * @return Number Adjusted voltage with feedforward
     */
    Number applyFeedforward(Number voltage, LinearVelocity velocity);

    /**
     * @brief Desaturate drive outputs to maintain proper turning ratio
     * 
     * @param lateral Lateral power (-1 to 1)
     * @param angular Angular power (-1 to 1)
     * @return std::pair<Number, Number> Left and right motor powers
     */
    std::pair<Number, Number> desaturate(Number lateral, Number angular) const;
};

} // namespace control
