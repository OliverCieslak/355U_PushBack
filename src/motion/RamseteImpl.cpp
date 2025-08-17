#include "motion/RamseteImpl.hpp"

namespace motion {

units::VelocityPose RamseteImpl::calculate(
    const units::Pose& currentPose, 
    const TrajectoryState& desiredState
) {
    // Extract desired pose and velocities
    const auto& desiredPose = desiredState.pose; // assumed already constructed with from_cDeg -> standard frame
    LinearVelocity linearVelocity = desiredState.velocity;

    // Debug / experiment toggles
    constexpr bool INVERT_Curvature = true;       // Invert curvature: observed desired heading decreasing while positive curvature gave CCW wRef
    constexpr bool INVERT_Y_ERROR = false;        // Flip lateral error sign if steering opposite
    constexpr bool INVERT_HEADING_ERROR = false;  // Flip dtheta sign if rotation correction reversed
    constexpr bool INVERT_OMEGA_OUTPUT = false;   // Invert final angular velocity output sign
    constexpr bool DISABLE_ANGULAR_CORRECTION = false; // Re-enable angular correction for heading convergence
    constexpr bool ZERO_CURVATURE = false;        // Re-enable trajectory curvature to evaluate curvature sign
    constexpr double OMEGA_GAIN = 0.3;            // Softer initial correction gain

    // Calculate angular velocity from linear velocity and curvature.
    // Empirically the chassis spins opposite the intended heading change, indicating a frame/sign mismatch.
    Curvature curv = ZERO_CURVATURE ? 0_radpm : desiredState.curvature * (INVERT_Curvature ? -1.0 : 1.0);
    AngularVelocity angularVelocity = from_radps(to_mps(linearVelocity) * to_radpm(curv));
    // If robot forward axis is reversed relative to trajectory frame, invert linearVelocity so wheel mapping stays consistent
    // Remove temporary forward-axis reversal hack; treat trajectory forward as robot forward.
    constexpr bool FORWARD_AXIS_REVERSED = false;

    // Optionally convert current and desired orientations from compass (0=N, CW+) to standard (0=+X/E, CCW+)
    constexpr bool CONVERT_CURRENT_COMPASS_TO_STANDARD = true;  // localized fix for sensor frame
    constexpr bool CONVERT_DESIRED_COMPASS_TO_STANDARD = false; // trajectory headings already stored in standard frame
    auto compassToStandard = [](Angle compassAngle){
        // Compass: 0=N (standard +Y), 90=E (standard +X), increases CW; Standard: 0=+X, positive CCW
        // Mapping: theta_std = 90deg - theta_compass
        return from_stDeg(90 - to_cDeg(compassAngle));
    };
    Angle currentHeadingStd = CONVERT_CURRENT_COMPASS_TO_STANDARD ? compassToStandard(currentPose.orientation)
                                                                 : currentPose.orientation;
    Angle desiredHeadingStd = CONVERT_DESIRED_COMPASS_TO_STANDARD ? compassToStandard(desiredPose.orientation)
                                                                 : desiredPose.orientation; // already standard

    // Calculate position error in robot's coordinate frame
    Length dx = desiredPose.x - currentPose.x;
    Length dy = desiredPose.y - currentPose.y;
    Angle dtheta = desiredHeadingStd - currentHeadingStd;
    if (INVERT_HEADING_ERROR) dtheta = -dtheta;

    // Normalize angle to [-π, π] using built-in function
    dtheta = units::constrainAngle180(dtheta);

    // Transform errors to robot frame
    double cosTheta = units::cos(currentHeadingStd);
    double sinTheta = units::sin(currentHeadingStd);
    
    Length x_error = cosTheta * dx + sinTheta * dy;
    Length y_error = -sinTheta * dx + cosTheta * dy;
    if (INVERT_Y_ERROR) y_error = -y_error;

    // Gain scheduling based on current velocity
    double k = 2 * m_zeta * std::sqrt(
        to_radps(angularVelocity) * to_radps(angularVelocity) + 
        m_b * to_mps(linearVelocity) * to_mps(linearVelocity)
    );

    // Calculate modified velocities based on Ramsete control law
    LinearVelocity v = ZERO_CURVATURE ? linearVelocity : from_mps(to_mps(linearVelocity) * units::cos(dtheta) + 
                 k * to_m(x_error));
    
    // Raw correction term
    double correction = k * to_stRad(dtheta) + (double)m_b * to_mps(linearVelocity) * utils::sinc(to_stRad(dtheta)) * to_m(y_error);
    correction *= OMEGA_GAIN; // scale
    AngularVelocity omega = angularVelocity;
    if (!DISABLE_ANGULAR_CORRECTION) {
        omega += from_radps(correction);
    }
    if (INVERT_OMEGA_OUTPUT) {
        omega = -omega;
    }

    // Create velocity pose with linear and angular components
    // VelocityPose needs x, y components of velocity and angular velocity
    static int dbg = 0;
    if (++dbg % 20 == 0) { // every ~200ms at 10ms loop
    printf("[RAMSETE_DBG] desH(raw)=%.1f desH(std)=%.1f curH(raw)=%.1f curH(std)=%.1f dH=%.1f xErr=%.2f yErr=%.2f vRef=%.2f wRef=%.2f vCmd=%.2f wCmd=%.2f corr=%.3f curv=%.4f (curvInv=%d) MODE=%s\n",
           to_cDeg(desiredPose.orientation), to_cDeg(desiredHeadingStd),
           to_cDeg(currentPose.orientation), to_cDeg(currentHeadingStd), to_stDeg(dtheta),
           to_in(x_error), to_in(y_error), to_inps(linearVelocity), to_radps(angularVelocity),
           to_inps(v), to_radps(omega), correction, to_radpm(curv), (int)INVERT_Curvature,
           ZERO_CURVATURE ? "STRAIGHT" : "NORMAL");
    }
    // Provide chassis forward velocity directly in x component; y unused for diff drive.
    return units::VelocityPose(v, 0_inps, omega);
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
