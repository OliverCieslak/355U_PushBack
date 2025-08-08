#include "motion/RamseteImpl.hpp"

namespace motion {

units::VelocityPose RamseteImpl::calculate(
    const units::Pose& currentPose, 
    const TrajectoryState& desiredState
) {
    // Extract desired pose and velocities
    const auto& desiredPose = desiredState.pose;
    LinearVelocity linearVelocity = desiredState.velocity;
    
    // Calculate angular velocity from linear velocity and curvature
    AngularVelocity angularVelocity = from_radps(to_mps(linearVelocity) * to_radpm(desiredState.curvature));

    // Calculate position error in robot's coordinate frame
    Length dx = desiredPose.x - currentPose.x;
    Length dy = desiredPose.y - currentPose.y;
    Angle dtheta = desiredPose.orientation - currentPose.orientation;

    // Normalize angle to [-π, π] using built-in function
    dtheta = units::constrainAngle180(dtheta);

    // Transform errors to robot frame
    double cosTheta = units::cos(currentPose.orientation);
    double sinTheta = units::sin(currentPose.orientation);
    
    Length x_error = cosTheta * dx + sinTheta * dy;
    Length y_error = -sinTheta * dx + cosTheta * dy;

    // Gain scheduling based on current velocity
    double k = 2 * m_zeta * std::sqrt(
        to_radps(angularVelocity) * to_radps(angularVelocity) + 
        m_b * to_mps(linearVelocity) * to_mps(linearVelocity)
    );

    // Calculate modified velocities based on Ramsete control law
    LinearVelocity v = from_mps(to_mps(linearVelocity) * units::cos(dtheta) + 
                 k * to_m(x_error));
    
    AngularVelocity omega = angularVelocity + 
                           from_radps(k * to_stRad(dtheta) +
                           (double)m_b * to_mps(linearVelocity) * 
                           utils::sinc(to_stRad(dtheta)) * 
                           to_m(y_error));

    // Create velocity pose with linear and angular components
    // VelocityPose needs x, y components of velocity and angular velocity
    return units::VelocityPose(v * units::cos(currentPose.orientation), 
                               v * units::sin(currentPose.orientation), 
                               omega);
}

bool RamseteImpl::atReference(
    const units::Pose& currentPose, 
    const Trajectory& trajectory,
    Time time, 
    Length poseTolerance
) {
    // Check if trajectory is empty or time is beyond trajectory's end time
    if (trajectory.getStates().empty() || time >= trajectory.getTotalTime()) {
        // Get the final state
        TrajectoryState finalState = trajectory.getStates().empty() ? 
                                    TrajectoryState() : 
                                    trajectory.sample(trajectory.getTotalTime());
        
        // Calculate distance to final position
        Length dx = finalState.pose.x - currentPose.x;
        Length dy = finalState.pose.y - currentPose.y;
        Length distance = units::sqrt(units::square(dx) + units::square(dy));
        
        // Check if within tolerance
        return distance < poseTolerance;
    }
    
    return false;
}

} // namespace motion
