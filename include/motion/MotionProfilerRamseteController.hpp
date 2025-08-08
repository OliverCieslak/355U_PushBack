#pragma once

#include "control/ActionScheduler.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "motion/RamseteImpl.hpp"
#include "motion/Trajectory.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "pros/rtos.hpp"
#include "utils/DistanceUtils.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace motion {

/**
 * @brief Main class that manages motion profiling for a skid-steer robot
 */
class MotionProfilerRamseteController {
public:
    // Use the ActionTrigger enum from ActionScheduler
    using ActionTrigger = control::ActionTrigger;

    /**
     * @brief Construct a new Motion Profiler
     * 
     * @param leftMotors Left side motor group
     * @param rightMotors Right side motor group
     * @param driveConfig Differential drive configuration
     * @param poseProvider Function that returns the current robot pose
     * @param b RAMSETE controller gain (default: 2.0)
     * @param zeta RAMSETE controller damping factor (default: 0.7)
     */
    MotionProfilerRamseteController(
        lemlib::MotorGroup& leftMotors,
        lemlib::MotorGroup& rightMotors,
        const control::DifferentialDriveConfig& driveConfig,
        std::function<units::Pose()> poseProvider,
        double b = 2.0,
        double zeta = 0.7
    );

    /**
     * @brief Destructor to clean up any active tasks
     */
    ~MotionProfilerRamseteController();
    
    /**
     * @brief Set a new trajectory to follow
     * 
     * @param trajectory The trajectory to follow
     */
    void setTrajectory(const Trajectory& trajectory);

    /**
     * @brief Generate a trajectory from waypoints and set it for future following
     * 
     * @param waypoints List of waypoints to follow
     * @param config Trajectory generation config
     * @return true if the trajectory was generated and set successfully
     */
    bool planTrajectoryFromWaypoints(
        const std::vector<Waypoint>& waypoints, 
        const TrajectoryConfig& config
    );

    /**
     * @brief Update the motion profiler, calculating and applying motor outputs
     * 
     * @param dt Time since last update
     * @return true if still following trajectory, false if complete
     */
    bool update(Time dt);

    /**
     * @brief Stop following the current trajectory
     */
    void stop();

    /**
     * @brief Check if the robot is currently following a trajectory
     * 
     * @return true if following a trajectory
     */
    bool isFollowing() const;

    /**
     * @brief Get the current trajectory being followed
     * 
     * @return const Trajectory& The current trajectory
     */
    const Trajectory& getTrajectory() const;

    /**
     * @brief Get the current trajectory time
     * 
     * @return Time Current time along the trajectory
     */
    Time getTrajectoryTime() const;

    /**
     * @brief Follow the currently set trajectory
     * 
     * @param async Whether to follow the trajectory asynchronously (default: false)
     * @return true if the trajectory was started successfully
     */
    bool followTrajectory(bool async = false);

    /**
     * @brief Add an action to be executed during trajectory following
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
     * @param t Time from trajectory start
     * @return true if action was added successfully
     */
    bool addActionAtTimeFromStart(std::function<void()> action, Time t);

    /**
     * @brief Add an action to be executed at a specific time from end
     * 
     * @param action Function to execute
     * @param t Time before trajectory end
     * @return true if action was added successfully
     */
    bool addActionAtTimeFromEnd(std::function<void()> action, Time t);

    /**
     * @brief Add an action to be executed at a specific distance from start
     * 
     * @param action Function to execute
     * @param d Distance from trajectory start
     * @return true if action was added successfully
     */
    bool addActionAtDistanceFromStart(std::function<void()> action, Length d);

    /**
     * @brief Add an action to be executed at a specific distance from end
     * 
     * @param action Function to execute
     * @param d Distance before trajectory end
     * @return true if action was added successfully
     */
    bool addActionAtDistanceFromEnd(std::function<void()> action, Length d);

    /**
     * @brief Clear all scheduled actions
     */
    void clearActions();

    /**
     * @brief Get the actual distance traveled by the robot
     * 
     * @return Length The accumulated actual distance traveled
     */
    Length getActualDistanceTraveled() const;

private:
    lemlib::MotorGroup& m_leftMotors;
    lemlib::MotorGroup& m_rightMotors;
    control::DifferentialDriveConfig m_driveConfig;
    std::function<units::Pose()> m_poseProvider;
    
    Trajectory m_trajectory;
    RamseteImpl m_ramsete;
    
    Time m_trajectoryTime = 0_sec;
    bool m_isFollowing = false;
    
    pros::Task* m_followTask = nullptr;

    Length m_actualDistanceTraveled = 0_in;  // Accumulated actual distance traveled
    units::Pose m_previousPose;              // Previous robot pose for distance calculation

    // Action scheduler for managing actions during trajectory following
    control::ActionScheduler m_actionScheduler;
    
    /**
     * @brief Convert chassis speeds to differential drive wheel speeds
     * 
     * @param velocityPose Chassis speed with linear and angular velocity
     * @return std::pair<AngularVelocity, AngularVelocity> Left and right wheel speeds
     */
    std::pair<AngularVelocity, AngularVelocity> chassisSpeedsToWheelSpeeds(
        const units::VelocityPose& velocityPose
    );
    
    /**
     * @brief Apply feedforward to calculate motor voltages
     * 
     * @param leftVelocity Left wheel velocity
     * @param rightVelocity Right wheel velocity
     * @param leftAccel Left wheel acceleration
     * @param rightAccel Right wheel acceleration
     * @return std::pair<Number, Number> Left and right motor voltages
     */
    std::pair<Number, Number> calculateFeedforward(
        AngularVelocity leftVelocity,
        AngularVelocity rightVelocity,
        AngularAcceleration leftAccel,
        AngularAcceleration rightAccel
    );

    /**
     * @brief Calculate the total distance traversed at a given time in the trajectory
     * 
     * @param time Time along the trajectory
     * @return Length Distance traversed
     */
    Length calculateDistanceAtTime(Time time) const;
};

} // namespace motion
