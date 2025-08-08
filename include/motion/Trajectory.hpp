#pragma once

#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include <algorithm>
#include <vector>

namespace motion {

/**
 * @brief A state in a trajectory, containing pose, curvature, velocity, and acceleration
 */
struct TrajectoryState {
    units::Pose pose = units::Pose(0_in, 0_in, 0_cDeg);
    Curvature curvature = 0_radpm;
    LinearVelocity velocity = 0_inps;
    LinearAcceleration acceleration = 0_inps2;
    Time timestamp = 0_msec;
    Length distance = 0_in; // Cumulative distance traveled from trajectory start

    /**
     * @brief Construct a new Trajectory State object
     */
    TrajectoryState() = default;

    /**
     * @brief Construct a new Trajectory State object with specified parameters
     * 
     * @param pose Robot pose (position and orientation)
     * @param curvature Path curvature at this point
     * @param velocity Linear velocity
     * @param acceleration Linear acceleration
     * @param timestamp Time from start of trajectory
     * @param distance Distance traveled from start of trajectory
     */
    TrajectoryState(units::Pose pose, Curvature curvature, LinearVelocity velocity, 
                   LinearAcceleration acceleration, Time timestamp, Length distance = 0_in)
        : pose(pose), curvature(curvature), velocity(velocity), 
          acceleration(acceleration), timestamp(timestamp), distance(distance) {}
};

/**
 * @brief A trajectory consisting of states that define the robot's path
 */
class Trajectory {
public:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() = default;

    /**
     * @brief Construct a trajectory from a list of states
     * 
     * @param states List of trajectory states
     */
    explicit Trajectory(std::vector<TrajectoryState> states);

    /**
     * @brief Get the total duration of the trajectory
     * 
     * @return Time Total trajectory duration
     */
    Time getTotalTime() const;

    /**
     * @brief Get the list of trajectory states
     * 
     * @return const std::vector<TrajectoryState>& List of states
     */
    const std::vector<TrajectoryState>& getStates() const;

    /**
     * @brief Get the state at the specified time
     * 
     * @param time Time from trajectory start
     * @return TrajectoryState Interpolated state at the specified time
     */
    TrajectoryState sample(Time time) const;

    /**
     * @brief Get the total distance of the trajectory
     * 
     * @return Length Total trajectory distance
     */
    Length getTotalDistance() const;

    /**
     * @brief Get the state at the specified distance along the trajectory
     * 
     * @param distance Distance from trajectory start
     * @return TrajectoryState Interpolated state at the specified distance
     */
    TrajectoryState sampleByDistance(Length distance) const;

private:
    std::vector<TrajectoryState> m_states;
};

/**
 * @brief A waypoint in a path, containing position, heading, and optional constraints
 */
struct Waypoint {
    Length x = 0_in;
    Length y = 0_in;
    Angle heading = 0_cRad;
    LinearVelocity velocityConstraint = 0_mps; // Optional velocity constraint at this point

    /**
     * @brief Construct a new Waypoint
     * 
     * @param x X position
     * @param y Y position
     * @param heading Robot heading in compass orientation
     */
    Waypoint(Length x, Length y, Angle heading) 
        : x(x), y(y), heading(heading) {}

    /**
     * @brief Construct a new Waypoint with velocity constraint
     * 
     * @param x X position
     * @param y Y position
     * @param heading Robot heading in compass orientation
     * @param velocityConstraint Maximum velocity at this point
     */
    Waypoint(Length x, Length y, Angle heading, LinearVelocity velocityConstraint)
        : x(x), y(y), heading(heading), velocityConstraint(velocityConstraint) {}
        
    /**
     * @brief Convert this waypoint to a pose
     * 
     * @return units::Pose The pose representation of this waypoint
     */
    units::Pose toPose() const {
        return units::Pose(x, y, heading);
    }
};

} // namespace motion
