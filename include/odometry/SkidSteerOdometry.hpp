#pragma once

#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "pros/rtos.hpp" // Add PROS task header
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "utils/Utils.hpp"
#include <memory>

namespace odometry {

/**
 * @brief Class that tracks the position and orientation of a skid-steer drive robot
 * 
 * This class uses left and right motor encoders to track distance traveled and a
 * V5 Inertial Sensor for heading tracking. The pose is updated incrementally
 * based on encoder and IMU measurements.
 */
class SkidSteerOdometry {
public:
    /**
     * @brief Construct a new Skid Steer Odometry object
     * 
     * @param leftEncoder Left side encoder for tracking distance
     * @param rightEncoder Right side encoder for tracking distance
     * @param imu V5 Inertial Sensor for tracking heading
     * @param trackWidth Distance between left and right wheels
     * @param wheelDiameter Diameter of the drive wheels
     * @param initialPose Initial pose of the robot
     */
    SkidSteerOdometry(
        lemlib::Encoder& leftEncoder,
        lemlib::Encoder& rightEncoder,
        lemlib::IMU& imu,
        Length trackWidth,
        Length wheelDiameter,
        const units::Pose& initialPose = units::Pose()
    );

    /**
     * @brief Construct a new Skid Steer Odometry object using motor groups
     * 
     * @param leftMotors Left side motors for tracking distance
     * @param rightMotors Right side motors for tracking distance
     * @param imu V5 Inertial Sensor for tracking heading
     * @param trackWidth Distance between left and right wheels
     * @param wheelDiameter Diameter of the drive wheels
     * @param initialPose Initial pose of the robot
     */
    SkidSteerOdometry(
        lemlib::MotorGroup& leftMotors,
        lemlib::MotorGroup& rightMotors,
        lemlib::IMU& imu,
        Length trackWidth,
        Length wheelDiameter,
        const units::Pose& initialPose = units::Pose()
    );

    /**
     * @brief Destructor to ensure task is properly stopped
     */
    ~SkidSteerOdometry();

    /**
     * @brief Start the odometry update task
     * 
     * Begins updating the robot's position automatically every 10ms
     */
    void start();

    /**
     * @brief Stop the odometry update task
     */
    void stop();

    /**
     * @brief Update the robot's position using the latest encoder and IMU readings
     * 
     * This method should be called periodically to update the estimated pose of the robot.
     * 
     * @return units::Pose The current estimated pose of the robot
     */
    units::Pose update();

    /**
     * @brief Get the current estimated pose of the robot
     * 
     * @return units::Pose The current estimated pose
     */
    units::Pose getPose() const;

    /**
     * @brief Reset the odometry to a known pose
     * 
     * @param pose The pose to reset to
     */
    void resetPose(const units::Pose& pose);

    /**
     * @brief Get the current robot velocities
     * 
     * @return std::pair<LinearVelocity, AngularVelocity> Linear and angular velocity
     */
    std::pair<LinearVelocity, AngularVelocity> getVelocity() const;

    /**
     * @brief Print diagnostic information about odometry state
     */
    void printDiagnostics() const;

private:
    lemlib::Encoder* m_leftEncoder;
    lemlib::Encoder* m_rightEncoder;
    lemlib::IMU* m_imu;
    Length m_trackWidth;
    Length m_wheelDiameter;
    units::Pose m_pose;
    
    // Mutex for thread-safe access to pose
    mutable pros::Mutex m_mutex;

    Angle m_prevLeftPosition = 0_stRad;
    Angle m_prevRightPosition = 0_stRad;
    Angle m_prevHeading = 0_stRad;
    double m_imuOffset = 0.0;
    
    // Task management members
    pros::Task* m_updateTask = nullptr;
    bool m_isRunning = false;
    
    /**
     * @brief Static trampoline function for the PROS task
     * 
     * @param param Pointer to the SkidSteerOdometry instance
     */
    static void updateTaskTrampoline(void* param);
    
    /**
     * @brief The task update loop that runs continuously
     */
    void taskUpdate();

    /**
     * @brief Convert wheel angles to robot displacement
     * 
     * @param leftDelta Change in left wheel angle
     * @param rightDelta Change in right wheel angle
     * @param headingDelta Change in heading
     * @return units::Vector2D<Length> The displacement vector
     */
    units::Vector2D<Length> calculateDisplacement(Angle leftDelta, Angle rightDelta, Angle headingDelta);

    // Velocity tracking
    uint32_t m_lastUpdateTime = 0;
    LinearVelocity m_linearVelocity = 0_mps;
    AngularVelocity m_angularVelocity = 0_radps;
    units::Pose m_prevPose;
};

} // namespace odometry
