#include "motion/TrajectoryGenerator.hpp"

// Forward declaration of global trackWidth (declared in CompetitionAutons.cpp or similar)
extern Length trackWidth; // global (not in namespace motion)

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
            // Reduce tangent magnitude to soften curves (was 0.5). Smaller factor reduces overshoot & high curvature spikes.
            Length tangentMagnitude = units::hypot(end.x - start.x, end.y - start.y) * 0.25;
            
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
    
    // Optional smoothing pass (simple moving average) to reduce jitter in high-resolution points
    if (pathPoints.size() > 5) {
        std::vector<TrajectoryState> smoothed = pathPoints;
        int window = 3; // radius 1 on each side
        for (size_t i = 1; i + 1 < pathPoints.size(); ++i) {
            Length sx = 0_in; Length sy = 0_in; int count = 0;
            for (int o = -1; o <= 1; ++o) {
                int idx = (int)i + o;
                if (idx < 0 || idx >= (int)pathPoints.size()) continue;
                sx += pathPoints[idx].pose.x;
                sy += pathPoints[idx].pose.y;
                count++;
            }
            smoothed[i].pose.x = sx / (double)count;
            smoothed[i].pose.y = sy / (double)count;
        }
        // Recompute curvature after smoothing
        for (size_t i = 1; i + 1 < smoothed.size(); ++i) {
            smoothed[i].curvature = utils::calculateCurvature(smoothed[i-1].pose, smoothed[i].pose, smoothed[i+1].pose);
        }
        pathPoints.swap(smoothed);
    }

    // Recompute heading by integrating signed curvature (stored as 1/in) using trapezoidal rule while preserving initial heading.
    if (pathPoints.size() >= 2) {
        // Preserve supplied starting heading (already in pathPoints[0])
        Angle theta = pathPoints[0].pose.orientation;
        // Derive heading at second point via chord if curvature is extreme (noise) to stabilize start
        Length dx1 = pathPoints[1].pose.x - pathPoints[0].pose.x;
        Length dy1 = pathPoints[1].pose.y - pathPoints[0].pose.y;
        if (units::abs(pathPoints[1].curvature) > 0.75_radpm) {
            theta = units::atan2(dy1, dx1);
            pathPoints[1].pose.orientation = theta;
        }
        double prevK = pathPoints[1].curvature.internal();
        for (size_t i = 2; i < pathPoints.size(); ++i) {
            Length dx = pathPoints[i].pose.x - pathPoints[i-1].pose.x;
            Length dy = pathPoints[i].pose.y - pathPoints[i-1].pose.y;
            Length ds = units::hypot(dx, dy);
            double dsIn = to_in(ds);
            double k = pathPoints[i].curvature.internal();
            double kAvg = 0.5 * (k + prevK);
            double dTheta = kAvg * dsIn;
            theta += from_stRad(dTheta);
            pathPoints[i].pose.orientation = units::constrainAngle180(theta);
            prevK = k;
        }
        // After integration, adjust orientation so the final heading matches the user-specified final waypoint heading.
        // This prevents large unexpected heading offsets (e.g. ending at -60 compass when final waypoint requested 0 compass).
        Angle desiredFinal = waypoints.back().toPose().orientation; // waypoint final heading (already user-specified)
        Angle currentFinal = pathPoints.back().pose.orientation;
        Angle delta = units::constrainAngle180(desiredFinal - currentFinal);
        if (units::abs(delta) > from_stDeg(1.0)) {
            // Distribute correction proportionally along the path length so it is gradual, not a last-segment pivot.
            std::vector<Length> cum; cum.reserve(pathPoints.size());
            Length accumLen = 0_in; cum.push_back(accumLen);
            for (size_t i=1;i<pathPoints.size();++i) {
                Length ddx = pathPoints[i].pose.x - pathPoints[i-1].pose.x;
                Length ddy = pathPoints[i].pose.y - pathPoints[i-1].pose.y;
                accumLen += units::hypot(ddx, ddy);
                cum.push_back(accumLen);
            }
            Length totalLen = accumLen <= 1e-6_in ? 1_in : accumLen;
            for (size_t i=1;i<pathPoints.size();++i) { // keep starting heading unchanged
                double frac = to_in(cum[i]) / to_in(totalLen);
                Angle adj = delta * frac;
                pathPoints[i].pose.orientation = units::constrainAngle180(pathPoints[i].pose.orientation + adj);
            }
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
    
    // Final state's orientation exactly equals the user-specified final waypoint heading (already applied to pathPoints via adjustment above).
    if (pathPoints.size() >= 2) {
        finalState.pose.orientation = waypoints.back().toPose().orientation;
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
    
    // Calculate timestamps, accelerations, and cumulative distance field
    Time currentTime = 0_sec;
    timeStates[0].timestamp = currentTime;
    // Assign cumulative distance
    for (size_t i=0;i<timeStates.size();++i) {
        // distances vector already holds cumulative distances
        timeStates[i].distance = distances[i];
    }
    
    for (size_t i = 1; i < states.size(); i++) {
        Length ds = distances[i] - distances[i-1];
        LinearVelocity avgVelocity = (timeStates[i-1].velocity + timeStates[i].velocity) / 2.0;
        LinearVelocity absAvg = units::abs(avgVelocity);
        if (absAvg < .01_inps) {
            // Avoid division by zero; assume small time step
            currentTime += 1_msec;
        } else {
            currentTime += ds / absAvg; // use magnitude so time always increases
        }
        timeStates[i].timestamp = currentTime;
        // Calculate acceleration sign properly using actual velocity change over positive dt
        Time dt = timeStates[i].timestamp - timeStates[i-1].timestamp;
        if (dt < 1_msec) {
            timeStates[i].acceleration = 0_inps2;
        } else {
            timeStates[i].acceleration = (timeStates[i].velocity - timeStates[i-1].velocity) / dt;
        }
    }

    // Precompute wheel velocities respecting curvatureScale: reduce center velocity if differential would exceed max.
    Length tw = ::trackWidth; // use global definition
    double halfTrack = to_in(tw) / 2.0;
    double maxVAllowed = to_inps(config.maxVelocity);
    double scaleFactor = config.curvatureScale;
    for (auto &s : timeStates) {
        double vCenter = to_inps(s.velocity);      // signed center speed (negative when reversed)
        double k = s.curvature.internal() * scaleFactor; // geometric curvature (unchanged by reversal)
        double vMag = std::fabs(vCenter);
        double omega = vMag * k; // use magnitude to derive differential component
        // Forward-frame wheel speeds for positive vMag
        double fwdLeft = vMag - omega * halfTrack;
        double fwdRight = vMag + omega * halfTrack;
        double maxAbs = std::max(std::fabs(fwdLeft), std::fabs(fwdRight));
        if (maxVAllowed > 1e-6 && maxAbs > maxVAllowed) {
            double ratio = maxVAllowed / maxAbs;
            vMag *= ratio;
            omega = vMag * k;
            fwdLeft = vMag - omega * halfTrack;
            fwdRight = vMag + omega * halfTrack;
        }
        double vLeft, vRight;
        if (vCenter >= 0) {
            vLeft = fwdLeft;
            vRight = fwdRight;
        } else {
            // When driving backward, we want a positive geometric curvature (CCW) to still
            // produce a positive angular velocity (yaw left). Because omega = (vR - vL)/track,
            // simply negating both wheels would invert the sign (vR more negative than vL).
            // Swap the differential when negating so that (vR - vL) retains its forward sign.
            // Example: forward fwdLeft=10, fwdRight=14 -> CCW (vR>vL). Reverse mapping:
            // vLeft=-14, vRight=-10 -> vR - vL = 4 (still CCW) while both are negative (backing).
            vLeft = -fwdRight;
            vRight = -fwdLeft;
        }
        s.leftWheelVelocity = from_inps(vLeft);
        s.rightWheelVelocity = from_inps(vRight);
    }
    
    return timeStates;
}

} // namespace motion
