#include "motion/TrajectoryGenerator.hpp"

namespace motion {

Trajectory TrajectoryGenerator::generateTrajectory(
    const std::vector<Waypoint>& waypoints,
    const TrajectoryConfig& config
) {
    if (waypoints.size() < 2) {
        // Return empty trajectory if not enough waypoints
        return Trajectory();
    }

    // Generate path points with higher resolution
    const int pointsPerSegment = 20;
    std::vector<TrajectoryState> pathPoints;
    pathPoints.reserve((waypoints.size() - 1) * pointsPerSegment);

    // Add initial point
    TrajectoryState initialState;
    initialState.pose = waypoints[0].toPose();
    initialState.timestamp = 0_sec;
    initialState.velocity = config.startVelocity;
    initialState.acceleration = 0_mps2;
    initialState.curvature = 0_radpm;
    pathPoints.push_back(initialState);

    // Generate spline points between each waypoint pair
    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        const auto& start = waypoints[i];
        const auto& end = waypoints[i + 1];
        
        // Create intermediate points along the spline
        for (int j = 1; j < pointsPerSegment; j++) {
            double t = static_cast<double>(j) / pointsPerSegment;
            
            // Use cubic spline interpolation for position
            double h00 = 2*t*t*t - 3*t*t + 1;
            double h10 = t*t*t - 2*t*t + t;
            double h01 = -2*t*t*t + 3*t*t;
            double h11 = t*t*t - t*t;
            
            // Tangent vectors at endpoints (simplified approach)
            Length tangentMagnitude = units::hypot(end.x - start.x, end.y - start.y) * 0.5;
            
            // Use heading to determine tangent directions
            Angle startAngle = Angle(start.heading);
            Angle endAngle = Angle(end.heading);
            
            Length startTangentX = tangentMagnitude * units::cos(startAngle);
            Length startTangentY = tangentMagnitude * units::sin(startAngle);
            Length endTangentX = tangentMagnitude * units::cos(endAngle);
            Length endTangentY = tangentMagnitude * units::sin(endAngle);
            
            // Interpolate position
            Length x = h00 * start.x + h10 * startTangentX + h01 * end.x + h11 * endTangentX;
            Length y = h00 * start.y + h10 * startTangentY + h01 * end.y + h11 * endTangentY;
            
            // Linearly interpolate heading (for initial approximation)
            Angle heading = startAngle * (1 - t) + endAngle * t;
            
            TrajectoryState state;
            state.pose = units::Pose(x, y, heading);
            state.timestamp = 0_sec; // Will be set during time parameterization
            
            // Calculate curvature using 3 points if possible
            if (pathPoints.size() >= 1) {
                units::Pose prevPose = pathPoints.back().pose;
                units::Pose currentPose = state.pose;
                units::Pose nextPose = (i < waypoints.size() - 2 && j == pointsPerSegment - 1)
                    ? waypoints[i + 2].toPose()
                    : (j < pointsPerSegment - 1)
                        ? units::Pose(
                            x + startTangentX * 0.1, 
                            y + startTangentY * 0.1, 
                            heading + 0.1_stRad)
                        : end.toPose();
                
                state.curvature = utils::calculateCurvature(prevPose, currentPose, nextPose);
            } else {
                state.curvature = 0_radpm;
            }
            
            pathPoints.push_back(state);
        }
    }
    
    // Add final waypoint
    TrajectoryState finalState;
    finalState.pose = waypoints.back().toPose();
    finalState.timestamp = 0_sec;
    finalState.velocity = config.endVelocity;
    finalState.acceleration = 0_mps2;
    
    if (pathPoints.size() >= 2) {
        const auto& secondLast = pathPoints[pathPoints.size() - 2];
        finalState.curvature = pathPoints.back().curvature;
    } else {
        finalState.curvature = 0_radpm;
    }
    
    pathPoints.push_back(finalState);
    
    // Time parameterize the trajectory
    auto timeParameterizedStates = timeParameterizeTrajectory(pathPoints, config);
    
    return Trajectory(timeParameterizedStates);
}

Trajectory TrajectoryGenerator::generateTrajectory(
    const units::Pose& start,
    const units::Pose& end,
    const TrajectoryConfig& config
) {   
    std::vector<Waypoint> waypoints = {
        Waypoint(start.x, start.y, start.orientation),
        Waypoint(end.x, end.y, end.orientation)
    };
    
    return generateTrajectory(waypoints, config);
}

Trajectory TrajectoryGenerator::generateTrajectory(
    const std::vector<units::Pose>& poses,
    const TrajectoryConfig& config
) {
    std::vector<Waypoint> waypoints;
    waypoints.reserve(poses.size());
    
    for (const auto& pose : poses) {
        waypoints.emplace_back(pose.x, pose.y, pose.orientation);
    }
    
    return generateTrajectory(waypoints, config);
}

std::vector<TrajectoryState> TrajectoryGenerator::timeParameterizeTrajectory(
    const std::vector<TrajectoryState>& states,
    const TrajectoryConfig& config
) {
    if (states.empty()) {
        return {};
    }
    
    std::vector<TrajectoryState> timeStates = states;
    
    // Calculate distances between points
    std::vector<Length> distances;
    distances.reserve(states.size());
    distances.push_back(0_m); // First point has zero distance
    
    for (size_t i = 1; i < states.size(); i++) {
        Length dx = states[i].pose.x - states[i-1].pose.x;
        Length dy = states[i].pose.y - states[i-1].pose.y;
        distances.push_back(distances[i-1] + units::hypot(dx, dy));
    }
    
    // Forward pass: calculate maximum achievable velocity at each point
    // Fix: Initialize with a value instead of default constructor
    std::vector<LinearVelocity> maxVelocities(states.size(), 0_inps);
    maxVelocities[0] = config.startVelocity;
    
    for (size_t i = 1; i < states.size(); i++) {
        // Calculate max velocity based on curvature
        Curvature curvature = units::abs(states[i].curvature);
        LinearVelocity curvatureVelocity = config.maxVelocity;
        
        // Reduce velocity for tighter curves using centripetal acceleration physics
        if (curvature > 0.001_radpm) {
            // Physics formula: centripetal_acceleration = v²/r where r = 1/curvature
            // So v = sqrt(centripetal_acceleration / curvature)
            curvatureVelocity = units::sqrt(config.maxCentripetalAcceleration / curvature);
        }
        
        // Check for custom velocity constraints at waypoints
        LinearVelocity constraintVelocity = config.maxVelocity;
        
        // Calculate max velocity based on acceleration from previous point
        Length ds = distances[i] - distances[i-1];
        if (ds < 0.1_in) {
            // Points are too close together
            maxVelocities[i] = maxVelocities[i-1];
            continue;
        }
        
        LinearVelocity accelVelocity = units::sqrt(
            units::square(maxVelocities[i-1]) + 2 * config.maxAcceleration * ds
        );
        
        // Take the minimum of all constraints
        maxVelocities[i] = units::min(config.maxVelocity, 
                              units::min(curvatureVelocity, 
                                  units::min(constraintVelocity, accelVelocity)));
    }
    
    // Backward pass: ensure we can decelerate to the end velocity
    maxVelocities.back() = config.endVelocity;
    
    for (int i = states.size() - 2; i >= 0; i--) {
        Length ds = distances[i+1] - distances[i];
        if (ds < .1_in) {
            // Points are too close together
            maxVelocities[i] = maxVelocities[i+1];
            continue;
        }
        
        LinearVelocity accelVelocity = units::sqrt(
            units::square(maxVelocities[i+1]) + 2 * config.maxAcceleration * ds
        );
        
        maxVelocities[i] = units::min(maxVelocities[i], accelVelocity);
    }
    
    // Assign velocities to trajectory states
    for (size_t i = 0; i < states.size(); i++) {
        timeStates[i].velocity = config.reversed ? -maxVelocities[i] : maxVelocities[i];
    }
    
    // Calculate timestamps and accelerations
    Time currentTime = 0_sec;
    timeStates[0].timestamp = currentTime;
    
    for (size_t i = 1; i < states.size(); i++) {
        Length ds = distances[i] - distances[i-1];
        LinearVelocity avgVelocity = (timeStates[i-1].velocity + timeStates[i].velocity) / 2.0;
        
        if (units::abs(avgVelocity) < .01_inps) {
            // Avoid division by zero
            currentTime += 0.001_sec;
        } else {
            currentTime += ds / avgVelocity;
        }
        
        timeStates[i].timestamp = currentTime;
        
        // Calculate acceleration
        Time dt = timeStates[i].timestamp - timeStates[i-1].timestamp;
        if (dt < 8_msec) {
            // Points are too close in time
            timeStates[i].acceleration = timeStates[i-1].acceleration;
        } else {
            timeStates[i].acceleration = (timeStates[i].velocity - timeStates[i-1].velocity) / dt;
        }
    }
    
    return timeStates;
}

} // namespace motion
