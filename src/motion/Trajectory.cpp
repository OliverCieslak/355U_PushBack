#include "motion/Trajectory.hpp"

namespace motion {

Trajectory::Trajectory(std::vector<TrajectoryState> states)
    : m_states(std::move(states)) {}

Time Trajectory::getTotalTime() const {
    if (m_states.empty()) {
        return 0_sec;
    }
    return m_states.back().timestamp;
}

const std::vector<TrajectoryState>& Trajectory::getStates() const {
    return m_states;
}

TrajectoryState Trajectory::sample(Time time) const {
    if (m_states.empty()) {
        return TrajectoryState();
    }
    
    // Clamp time to trajectory bounds
    time = units::clamp(time, 0_sec, getTotalTime());
    
    // Handle boundary cases
    if (time <= m_states.front().timestamp) {
        return m_states.front();
    }
    
    if (time >= m_states.back().timestamp) {
        return m_states.back();
    }
    
    // Binary search to find the right segment
    auto it = std::lower_bound(
        m_states.begin(), 
        m_states.end(), 
        time,
        [](const TrajectoryState& state, Time t) {
            return state.timestamp < t;
        }
    );
    
    if (it == m_states.begin()) {
        return *it;
    }
    
    // Get the two states to interpolate between
    const TrajectoryState& s0 = *(it - 1);
    const TrajectoryState& s1 = *it;
    
    // Calculate interpolation parameter
    Time deltaT = s1.timestamp - s0.timestamp;
    double t = 0.0;
    if (deltaT.internal() > 1e-6) {
        t = (time - s0.timestamp).internal() / deltaT.internal();
    }
    
    // Interpolate state values
    TrajectoryState interpolated;
    
    // Linear interpolation of position
    interpolated.pose.x = s0.pose.x + (s1.pose.x - s0.pose.x) * t;
    interpolated.pose.y = s0.pose.y + (s1.pose.y - s0.pose.y) * t;
    
    // Angular interpolation of heading - handle wraparound correctly
    Angle heading0 = s0.pose.orientation;
    Angle heading1 = s1.pose.orientation;
    
    // Ensure we take the shortest path in angle interpolation
    Angle delta = units::constrainAngle180(heading1 - heading0);
    interpolated.pose.orientation = units::constrainAngle360(heading0 + delta * t);
    
    // Linear interpolation of other values
    interpolated.curvature = s0.curvature + (s1.curvature - s0.curvature) * t;
    interpolated.velocity = s0.velocity + (s1.velocity - s0.velocity) * t;
    interpolated.acceleration = s0.acceleration + (s1.acceleration - s0.acceleration) * t;
    interpolated.timestamp = time;
    
    return interpolated;
}

Length Trajectory::getTotalDistance() const {
    if (m_states.empty()) {
        return 0_in;
    }
    return m_states.back().distance;
}

TrajectoryState Trajectory::sampleByDistance(Length distance) const {
    if (m_states.empty()) {
        return TrajectoryState();
    }
    
    // Clamp distance to trajectory bounds
    distance = units::clamp(distance, 0_in, getTotalDistance());
    
    // Handle boundary cases
    if (distance <= m_states.front().distance) {
        return m_states.front();
    }
    
    if (distance >= m_states.back().distance) {
        return m_states.back();
    }
    
    // Binary search to find the right segment
    auto it = std::lower_bound(
        m_states.begin(), 
        m_states.end(), 
        distance,
        [](const TrajectoryState& state, Length d) {
            return state.distance < d;
        }
    );
    
    if (it == m_states.begin()) {
        return *it;
    }
    
    // Get the two states to interpolate between
    const TrajectoryState& s0 = *(it - 1);
    const TrajectoryState& s1 = *it;
    
    // Calculate interpolation parameter
    Length deltaD = s1.distance - s0.distance;
    double t = 0.0;
    if (deltaD.internal() > 1e-6) {
        t = (distance - s0.distance).internal() / deltaD.internal();
    }
    
    // Interpolate state values
    TrajectoryState interpolated;
    
    // Linear interpolation of position
    interpolated.pose.x = s0.pose.x + (s1.pose.x - s0.pose.x) * t;
    interpolated.pose.y = s0.pose.y + (s1.pose.y - s0.pose.y) * t;
    
    // Angular interpolation of heading - handle wraparound correctly
    Angle heading0 = s0.pose.orientation;
    Angle heading1 = s1.pose.orientation;
    
    // Ensure we take the shortest path in angle interpolation
    Angle delta = units::constrainAngle180(heading1 - heading0);
    interpolated.pose.orientation = units::constrainAngle360(heading0 + delta * t);
    
    // Linear interpolation of other values
    interpolated.curvature = s0.curvature + (s1.curvature - s0.curvature) * t;
    interpolated.velocity = s0.velocity + (s1.velocity - s0.velocity) * t;
    interpolated.acceleration = s0.acceleration + (s1.acceleration - s0.acceleration) * t;
    interpolated.timestamp = s0.timestamp + (s1.timestamp - s0.timestamp) * t;
    interpolated.distance = distance;
    
    return interpolated;
}

} // namespace motion
