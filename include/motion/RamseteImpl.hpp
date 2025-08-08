#pragma once

#include "motion/Trajectory.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "utils/Utils.hpp"
#include <cmath>

namespace motion {

/**
 * @brief Implements a Ramsete calculator for trajectory following
 * 
 * The Ramsete calculator is a nonlinear time-varying computation that can be used to follow
 * trajectories with a differential drive robot. For more details, see:
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf
 */
class RamseteImpl {
public:
    /**
     * @brief Construct a new Ramsete calculator
     * 
     * @param b Path convergence tuning parameter (larger = more aggressive)
     * @param zeta Damping factor tuning parameter (1 = critically damped)
     */
    RamseteImpl(Number b = Number(2.0), Number zeta = Number(0.7))
        : m_b(b), m_zeta(zeta) {}
    
    /**
     * @brief Calculate the chassis speeds to follow the trajectory
     * 
     * @param currentPose Current robot pose
     * @param desiredState Desired trajectory state
     * @return units::VelocityPose Velocity pose with linear and angular velocities
     */
    units::VelocityPose calculate(
        const units::Pose& currentPose, 
        const TrajectoryState& desiredState
    );

    /**
     * @brief Reset the state
     */
    void reset() {};

    /**
     * @brief Check if the chassis has finished following the trajectory
     * 
     * @param currentPose Current robot pose
     * @param trajectory Reference trajectory
     * @param time Current time
     * @param poseTolerance Pose tolerance for considering the trajectory complete
     * @return true if the trajectory is complete
     */
    bool atReference(
        const units::Pose& currentPose, 
        const Trajectory& trajectory,
        Time time, 
        Length poseTolerance = 5_cm
    );

private:
    Number m_b;
    Number m_zeta;
    
    /**
     * @brief Calculate the sinc function: sin(x)/x
     * 
     * @param x Input value
     * @return double Result of sinc(x)
     */
    double sinc(double x);
};

} // namespace motion
