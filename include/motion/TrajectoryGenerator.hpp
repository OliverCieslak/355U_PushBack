#pragma once

#include "motion/Trajectory.hpp"
#include "utils/DistanceUtils.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace motion {

/**
 * @brief Configuration for trajectory generation
 */
struct TrajectoryConfig {
    LinearVelocity maxVelocity;
    LinearAcceleration maxAcceleration;
    LinearAcceleration maxCentripetalAcceleration;
    bool reversed = false;
    LinearVelocity startVelocity = 0_mps;
    LinearVelocity endVelocity = 0_mps;

    /**
     * @brief Construct a trajectory configuration
     * 
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     */
    TrajectoryConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, LinearAcceleration maxCentripetalAcceleration)
        : maxVelocity(maxVelocity), maxAcceleration(maxAcceleration), maxCentripetalAcceleration(maxCentripetalAcceleration) {}

    /**
     * @brief Set the trajectory to be reversed (robot drives backward)
     * 
     * @param reversed Whether the trajectory should be reversed
     * @return TrajectoryConfig& This config, for method chaining
     */
    TrajectoryConfig& setReversed(bool reversed) {
        this->reversed = reversed;
        return *this;
    }

    /**
     * @brief Set the start velocity
     * 
     * @param velocity Start velocity
     * @return TrajectoryConfig& This config, for method chaining
     */
    TrajectoryConfig& setStartVelocity(LinearVelocity velocity) {
        this->startVelocity = velocity;
        return *this;
    }

    /**
     * @brief Set the end velocity
     * 
     * @param velocity End velocity
     * @return TrajectoryConfig& This config, for method chaining
     */
    TrajectoryConfig& setEndVelocity(LinearVelocity velocity) {
        this->endVelocity = velocity;
        return *this;
    }

    /**
     * @brief Set the maximum centripetal acceleration
     * 
     * @param acceleration Maximum centripetal acceleration (lateral acceleration in turns)
     * @return TrajectoryConfig& This config, for method chaining
     */
    TrajectoryConfig& setMaxCentripetalAcceleration(LinearAcceleration acceleration) {
        this->maxCentripetalAcceleration = acceleration;
        return *this;
    }
};

/**
 * @brief Class to generate trajectories from waypoints
 */
class TrajectoryGenerator {
public:
    /**
     * @brief Generate a trajectory from a list of waypoints
     * 
     * @param waypoints List of waypoints for the robot to follow
     * @param config Configuration for trajectory generation
     * @return Trajectory Generated trajectory
     */
    static Trajectory generateTrajectory(
        const std::vector<Waypoint>& waypoints,
        const TrajectoryConfig& config
    );

    /**
     * @brief Generate a trajectory from a starting pose to an ending pose
     * 
     * @param start Starting pose
     * @param end Ending pose
     * @param config Configuration for trajectory generation
     * @return Trajectory Generated trajectory
     */
    static Trajectory generateTrajectory(
        const units::Pose& start,
        const units::Pose& end,
        const TrajectoryConfig& config
    );

    /**
     * @brief Generate a trajectory from a list of poses
     * 
     * @param poses List of poses for the robot to follow
     * @param config Configuration for trajectory generation
     * @return Trajectory Generated trajectory
     */
    static Trajectory generateTrajectory(
        const std::vector<units::Pose>& poses,
        const TrajectoryConfig& config
    );

private:
    /**
     * @brief Parameterize a trajectory with time and velocity
     * 
     * @param states States with position and curvature
     * @param config Configuration with velocity and acceleration constraints
     * @return std::vector<TrajectoryState> Time-parameterized trajectory states
     */
    static std::vector<TrajectoryState> timeParameterizeTrajectory(
        const std::vector<TrajectoryState>& states,
        const TrajectoryConfig& config
    );
    
    // Remove calculateCurvature method - moved to utils/DistanceUtils
};

} // namespace motion
