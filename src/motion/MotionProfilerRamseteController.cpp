#include "motion/MotionProfilerRamseteController.hpp"
#include <algorithm>
#include <cmath>

namespace motion {

MotionProfilerRamseteController::MotionProfilerRamseteController(
    lemlib::MotorGroup& leftMotors,
    lemlib::MotorGroup& rightMotors,
    const control::DifferentialDriveConfig& driveConfig,
    std::function<units::Pose()> poseProvider,
    double b,
    double zeta
) : m_leftMotors(leftMotors),
    m_rightMotors(rightMotors),
    m_driveConfig(driveConfig),
    m_poseProvider(poseProvider),
    m_ramsete(RamseteImpl(b, zeta))
{
}

void MotionProfilerRamseteController::setTrajectory(const Trajectory& trajectory) {
    m_trajectory = trajectory;
    m_trajectoryTime = 0_sec;
    m_isFollowing = true;
    m_actualDistanceTraveled = 0_in;  // Reset accumulated distance
    
    // Clear scheduled actions when setting a new trajectory
    clearActions();
}

bool MotionProfilerRamseteController::planTrajectoryFromWaypoints(
    const std::vector<Waypoint>& waypoints, 
    const TrajectoryConfig& config
) {
    // Generate a trajectory from the waypoints
    auto generatedTrajectory = TrajectoryGenerator::generateTrajectory(waypoints, config);
    
    // Check if trajectory generation was successful
    // This assumes TrajectoryGenerator returns a valid trajectory or some indication of failure
    if (generatedTrajectory.getTotalTime() <= 0_sec) {  // Using a basic validation check
        return false;
    }
    
    // Set the generated trajectory
    setTrajectory(generatedTrajectory);
    return true;
}

bool MotionProfilerRamseteController::update(Time dt) {
    if (!m_isFollowing) {
        return false;
    }
    
    // Get current state of the trajectory
    // auto trajectoryState = m_trajectory.sample(m_trajectoryTime);
    auto trajectoryState = m_trajectory.sampleByDistance(m_actualDistanceTraveled);
    
    // If we've reached the end of the trajectory
    if (m_trajectoryTime >= m_trajectory.getTotalTime()) {
        stop();
        return false;
    }

    if(m_actualDistanceTraveled >= m_trajectory.getTotalDistance()) {
        stop();
        return false;
    }

    // Check scheduled actions
    Length currentDistance = calculateDistanceAtTime(m_trajectoryTime);
    Length totalDistance = calculateDistanceAtTime(m_trajectory.getTotalTime());
    
    // Process actions using the action scheduler
    m_actionScheduler.processActions(m_trajectoryTime, m_trajectory.getTotalTime(), 
                                   currentDistance, totalDistance);
    
    // Get current robot pose
    auto currentPose = m_poseProvider();
    
    // Calculate and accumulate actual distance traveled
    if (m_isFollowing) {
        if (m_trajectoryTime > 0_sec) {  // Skip the first update
            Length distanceMoved = currentPose.distanceTo(m_previousPose);
            m_actualDistanceTraveled += distanceMoved;
        }
        m_previousPose = currentPose;  // Store current pose for next update
    }
    
    // Calculate the desired chassis speeds using the RAMSETE controller
    auto targetChassisSpeeds = m_ramsete.calculate(currentPose, trajectoryState);
    
    // Convert chassis speeds to wheel speeds
    auto [leftWheelSpeed, rightWheelSpeed] = chassisSpeedsToWheelSpeeds(targetChassisSpeeds);
    
    // Calculate wheel accelerations (using velocity change over time)
    static AngularVelocity lastLeftVelocity = 0_radps;
    static AngularVelocity lastRightVelocity = 0_radps;
    
    AngularAcceleration leftAccel = (dt.internal() > 0) ? 
        (leftWheelSpeed - lastLeftVelocity) / dt : 0_radps2;
    AngularAcceleration rightAccel = (dt.internal() > 0) ? 
        (rightWheelSpeed - lastRightVelocity) / dt : 0_radps2;
    
    lastLeftVelocity = leftWheelSpeed;
    lastRightVelocity = rightWheelSpeed;
    
    // Apply feedforward to calculate motor voltages
    auto [leftVoltage, rightVoltage] = calculateFeedforward(
        leftWheelSpeed, rightWheelSpeed, leftAccel, rightAccel
    );
    
    // Apply voltages to motors
    m_leftMotors.move(leftVoltage);
    m_rightMotors.move(rightVoltage);
    
    // Update trajectory time
    m_trajectoryTime += dt;
    
    return true;
}

// Static wrapper function for the asynchronous task
static void followTrajectoryTask(void* param) {
    MotionProfilerRamseteController* profiler = static_cast<MotionProfilerRamseteController*>(param);
    
    while (profiler->isFollowing()) {
        profiler->update(10_msec); // Update at 10ms intervals
        pros::delay(10);           // Sleep for 10ms
    }
}

MotionProfilerRamseteController::~MotionProfilerRamseteController() {
    // Clean up the task if it exists
    if (m_followTask != nullptr) {
        m_followTask->remove();
        delete m_followTask;
        m_followTask = nullptr;
    }
}

bool MotionProfilerRamseteController::followTrajectory(bool async) {
    // Make sure we have a valid trajectory to follow
    if (m_trajectory.getTotalTime() <= 0_sec) {
        return false;
    }
    
    // Reset trajectory time and mark as following
    m_trajectoryTime = 0_sec;
    m_isFollowing = true;
    m_actualDistanceTraveled = 0_in;  // Reset accumulated distance
    m_previousPose = m_poseProvider();  // Initialize previous pose
    
    // Reset all action execution flags
    m_actionScheduler.resetActions();
    
    if (async) {
        // Clean up any existing task
        if (m_followTask != nullptr) {
            m_followTask->remove();
            delete m_followTask;
            m_followTask = nullptr;
        }
        
        // Create a new task to follow the trajectory
        m_followTask = new pros::Task(followTrajectoryTask, this, TASK_PRIORITY_DEFAULT, 
                                     TASK_STACK_DEPTH_DEFAULT, "Motion Profile");
        return true;
    } else {
        // Synchronous following - block until complete
        while (m_isFollowing) {
            update(10_msec); 
            pros::delay(10);
        }
        return true;
    }
}

void MotionProfilerRamseteController::stop() {
    m_isFollowing = false;
    m_leftMotors.brake();
    m_rightMotors.brake();
    
    // The task will terminate naturally since it checks isFollowing()
}

bool MotionProfilerRamseteController::isFollowing() const {
    return m_isFollowing;
}

const Trajectory& MotionProfilerRamseteController::getTrajectory() const {
    return m_trajectory;
}

Time MotionProfilerRamseteController::getTrajectoryTime() const {
    return m_trajectoryTime;
}

std::pair<AngularVelocity, AngularVelocity> MotionProfilerRamseteController::chassisSpeedsToWheelSpeeds(
    const units::VelocityPose& velocityPose
) {
    // Calculate wheel speeds from chassis speeds
    LinearVelocity linearVelocity = velocityPose.x; // Forward velocity
    AngularVelocity angularVelocity = velocityPose.orientation; // Angular velocity
    
    // Use standard differential drive kinematics
    LinearVelocity angularComponent = from_inps(to_radps(angularVelocity) * to_in(m_driveConfig.trackWidth) / 2);
    LinearVelocity leftLinearVelocity = linearVelocity - angularComponent;
    LinearVelocity rightLinearVelocity = linearVelocity + angularComponent;
    
    // Convert linear velocity to angular velocity (wheel rotation)
    AngularVelocity leftAngularVelocity = utils::toAngular(leftLinearVelocity, m_driveConfig.wheelDiameter);
    AngularVelocity rightAngularVelocity = utils::toAngular(rightLinearVelocity, m_driveConfig.wheelDiameter);
    
    return {leftAngularVelocity, rightAngularVelocity};
}

std::pair<Number, Number> MotionProfilerRamseteController::calculateFeedforward(
    AngularVelocity leftVelocity,
    AngularVelocity rightVelocity,
    AngularAcceleration leftAccel,
    AngularAcceleration rightAccel
) {
    // Convert angular velocity to linear velocity
    LinearVelocity leftLinearVelocity = utils::toLinear(leftVelocity, m_driveConfig.wheelDiameter);
    LinearVelocity rightLinearVelocity = utils::toLinear(rightVelocity, m_driveConfig.wheelDiameter);
    
    // Convert angular acceleration to linear acceleration
    LinearAcceleration leftLinearAccel = utils::toLinear(leftAccel, m_driveConfig.wheelDiameter);
    LinearAcceleration rightLinearAccel = utils::toLinear(rightAccel, m_driveConfig.wheelDiameter);
    
    // Apply feedforward model: kS + kV * velocity + kA * acceleration
    Number leftVoltage = m_driveConfig.kS;
    if (leftLinearVelocity != 0_mps) leftVoltage *= units::sgn(leftLinearVelocity);
    leftVoltage += m_driveConfig.kV * to_mps(leftLinearVelocity) + m_driveConfig.kA * to_mps2(leftLinearAccel);
    
    Number rightVoltage = m_driveConfig.kS;
    if (rightLinearVelocity != 0_mps) rightVoltage *= units::sgn(rightLinearVelocity);
    rightVoltage += m_driveConfig.kV * to_mps(rightLinearVelocity) + m_driveConfig.kA * to_mps2(rightLinearAccel);
    
    // Clamp voltages to valid range (-1.0 to 1.0 for normalized voltage)
    leftVoltage = units::clamp(leftVoltage, Number(-1.0), Number(1.0));
    rightVoltage = units::clamp(rightVoltage, Number(-1.0), Number(1.0));
    
    return {leftVoltage, rightVoltage};
}

bool MotionProfilerRamseteController::addAction(std::function<void()> action, ActionTrigger trigger, double value) {
    // Validate value based on trigger type
    if (value < 0) {
        return false; // Negative values don't make sense for any trigger
    }
    
    if (trigger == ActionTrigger::TIME_FROM_END || trigger == ActionTrigger::DISTANCE_FROM_END) {
        // For "from end" triggers, make sure value doesn't exceed trajectory length
        double maxValue = (trigger == ActionTrigger::TIME_FROM_END) ? 
                          to_sec(m_trajectory.getTotalTime()) : 
                          to_in(calculateDistanceAtTime(m_trajectory.getTotalTime()));
        
        if (value > maxValue) {
            return false; // Value exceeds trajectory total time/distance
        }
    }
    
    // Add the action to our scheduler
    return m_actionScheduler.addAction(action, trigger, value);
}

bool MotionProfilerRamseteController::addActionAtTimeFromStart(std::function<void()> action, Time t) {
    return m_actionScheduler.addActionAtTimeFromStart(action, t);
}

bool MotionProfilerRamseteController::addActionAtTimeFromEnd(std::function<void()> action, Time t) {
    return m_actionScheduler.addActionAtTimeFromEnd(action, t);
}

bool MotionProfilerRamseteController::addActionAtDistanceFromStart(std::function<void()> action, Length d) {
    return m_actionScheduler.addActionAtDistanceFromStart(action, d);
}

bool MotionProfilerRamseteController::addActionAtDistanceFromEnd(std::function<void()> action, Length d) {
    return m_actionScheduler.addActionAtDistanceFromEnd(action, d);
}

void MotionProfilerRamseteController::clearActions() {
    m_actionScheduler.clearActions();
}

Length MotionProfilerRamseteController::calculateDistanceAtTime(Time time) const {
    // Approximate the distance traveled by summing up the distances between waypoints
    // up to the given time
    Length totalDistance = 0_in;
    Time accumulatedTime = 0_sec;
    
    if (m_trajectory.getStates().empty()) {
        return 0_in;
    }
    
    // Start with the first state
    auto previousState = m_trajectory.getStates()[0];
    
    for (size_t i = 1; i < m_trajectory.getStates().size(); i++) {
        auto currentState = m_trajectory.getStates()[i];
        
        // If we've gone beyond the requested time, we've found our segment
        if (currentState.timestamp > time) {
            // Calculate what fraction of this segment we've traversed
            double segmentProgress = 0.0;
            Time segmentDuration = currentState.timestamp - previousState.timestamp;
            
            if (segmentDuration > 0_sec) {
                segmentProgress = to_sec(time - previousState.timestamp) / to_sec(segmentDuration);
            }
            
            // Calculate the distance for this partial segment
            Length segmentDistance = previousState.pose.distanceTo(currentState.pose);
            totalDistance += segmentDistance * segmentProgress;
            
            return totalDistance;
        }
        
        // Add the full length of this segment
        totalDistance += previousState.pose.distanceTo(currentState.pose);
        previousState = currentState;
    }
    
    return totalDistance; // We've reached the end of the trajectory
}

Length MotionProfilerRamseteController::getActualDistanceTraveled() const {
    return m_actualDistanceTraveled;
}

} // namespace motion
