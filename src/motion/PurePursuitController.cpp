#include "motion/PurePursuitController.hpp"
#include "api.h" // provides pros::Task and delay
#include "utils/DistanceUtils.hpp"
#include <cmath>
#include <algorithm>

namespace motion {

// Implementation
PurePursuitController::PurePursuitController(
    lemlib::MotorGroup& leftMotors,
    lemlib::MotorGroup& rightMotors,
    const control::DifferentialDriveConfig& driveConfig,
    std::function<units::Pose()> poseProvider,
    const PurePursuitConfig& config
) : m_leftMotors(leftMotors), m_rightMotors(rightMotors), m_driveConfig(driveConfig),
    m_poseProvider(poseProvider), m_cfg(config), m_lookahead(config.lookahead) {}

PurePursuitController::~PurePursuitController() {
    stop();
}

void PurePursuitController::setConfig(const PurePursuitConfig& config) {
    m_cfg = config;
    m_lookahead = config.lookahead;
}

void PurePursuitController::setStanleyGains(Number gain, LinearVelocity softeningVel, Number omegaGain) {
    m_cfg.stanleyGain = gain.internal();
    m_cfg.stanleyOmegaGain = omegaGain.internal();
    m_stanleyUseVelMode = true;
    m_stanleySofteningVel = softeningVel;
}

void PurePursuitController::setPath(const std::vector<units::Pose>& path) {
    m_path = path;
    m_lastClosestIndex = 0;
    m_cumulativeLengths.clear();
    m_cumulativeLengths.reserve(m_path.size());
    Length accum = 0_in;
    m_cumulativeLengths.push_back(accum);
    for (size_t i=1;i<m_path.size();++i){
        Length dx = m_path[i].x - m_path[i-1].x;
        Length dy = m_path[i].y - m_path[i-1].y;
        accum += units::sqrt(dx*dx+dy*dy);
        m_cumulativeLengths.push_back(accum);
    }
    m_totalPathLength = accum;
}

void PurePursuitController::setTrajectory(const motion::Trajectory& traj) {
    m_trajStates = traj.getStates();
    m_trajTimeIndex = 0;
    m_lastClosestIndex = 0;
    m_runTime = 0_sec;
    m_distIncreasingCount = 0;
    m_lastDistEnd = 0_in;
    m_path.clear();
    m_path.reserve(m_trajStates.size());
    for (auto& s : m_trajStates) m_path.push_back(s.pose);
    m_lastClosestIndex = 0;
    m_cumulativeLengths.clear();
    m_cumulativeLengths.reserve(m_trajStates.size());
    if (!m_trajStates.empty() && m_trajStates.back().distance > 0_in) {
        for (auto& s: m_trajStates) m_cumulativeLengths.push_back(s.distance);
        m_totalPathLength = m_trajStates.back().distance;
    } else {
        Length accum=0_in; m_cumulativeLengths.push_back(accum);
        for(size_t i=1;i<m_path.size();++i){
            Length dx = m_path[i].x - m_path[i-1].x; Length dy = m_path[i].y - m_path[i-1].y;
            accum += units::sqrt(dx*dx+dy*dy); m_cumulativeLengths.push_back(accum);
        }
        m_totalPathLength = accum;
    }
    if (!m_trajStates.empty()) {
        m_reversed = false;
        m_maxTrajAccel = 0_inps2;
        for (auto &s : m_trajStates) {
            double vv = to_inps(s.velocity);
            if (std::fabs(vv) > 1e-3) { m_reversed = vv < 0; break; }
        }
        for (auto &s : m_trajStates) {
            if (units::abs(s.acceleration) > units::abs(m_maxTrajAccel)) m_maxTrajAccel = units::abs(s.acceleration);
        }
    }
}

int PurePursuitController::findClosestSegmentIndex(const units::Pose& current) {
    if (m_path.size() < 2) return 0;
    int bestIndex = m_lastClosestIndex;
    Length bestDist = 1e9_in;
    int start = std::max(0, m_lastClosestIndex - 2);
    int end   = std::min<int>(m_path.size() - 2, m_lastClosestIndex + 25);
    for (int i = start; i <= end; ++i) {
        auto& p0 = m_path[i];
        auto& p1 = m_path[i+1];
        double x1 = to_in(p0.x), y1 = to_in(p0.y);
        double x2 = to_in(p1.x), y2 = to_in(p1.y);
        double x = to_in(current.x), y = to_in(current.y);
        double dx = x2 - x1; double dy = y2 - y1;
        double segLen2 = dx*dx + dy*dy + 1e-9;
        double t = ((x - x1)*dx + (y - y1)*dy)/segLen2;
        t = std::clamp(t, 0.0, 1.0);
        double projx = x1 + t*dx; double projy = y1 + t*dy;
        double dist = std::hypot(x - projx, y - projy);
        if (dist < to_in(bestDist)) { bestDist = from_in(dist); bestIndex = i; }
    }
    m_lastClosestIndex = bestIndex;
    return bestIndex;
}

Length PurePursuitController::computePathProgress(const units::Pose& current, int segIndex, Length& alongDistToEnd) {
    if (m_path.size() < 2 || m_cumulativeLengths.size()!=m_path.size()) { alongDistToEnd = 0_in; return 0_in; }
    segIndex = std::clamp(segIndex, 0, (int)m_path.size()-2);
    auto& p0 = m_path[segIndex]; auto& p1 = m_path[segIndex+1];
    double x0=to_in(p0.x), y0=to_in(p0.y); double x1=to_in(p1.x), y1=to_in(p1.y);
    double dx=x1-x0, dy=y1-y0; double segLen2 = dx*dx+dy*dy + 1e-9; double t = ((to_in(current.x)-x0)*dx + (to_in(current.y)-y0)*dy)/segLen2; t = std::clamp(t,0.0,1.0);
    Length segLen = from_in(std::sqrt(segLen2));
    Length along = m_cumulativeLengths[segIndex] + segLen * t;
    alongDistToEnd = (m_totalPathLength - along);
    return along;
}

bool PurePursuitController::computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget) {
    if (m_path.size() < 2) { outTarget = current; return false; }
    double L = to_in(m_lookahead);
    int segIndex = findClosestSegmentIndex(current);
    for (int i = segIndex; i < (int)m_path.size()-1; ++i) {
        auto &p0 = m_path[i];
        auto &p1 = m_path[i+1];
        double x0 = to_in(p0.x), y0 = to_in(p0.y);
        double x1 = to_in(p1.x), y1 = to_in(p1.y);
        double dx = x1 - x0; double dy = y1 - y0;
        double fx = to_in(current.x) - x0; double fy = to_in(current.y) - y0;
        double a = dx*dx + dy*dy;
        double b = 2*(dx*fx + dy*fy);
        double c = fx*fx + fy*fy - L*L;
        double disc = b*b - 4*a*c;
        if (disc < 0) continue;
        double sqrtD = std::sqrt(disc);
        double t1 = (-b + sqrtD)/(2*a);
        double t2 = (-b - sqrtD)/(2*a);
        auto valid = [&](double t){ return t >= 0.0 && t <= 1.0; };
        double t = -1;
        if (valid(t1) && valid(t2)) t = std::max(t1, t2);
        else if (valid(t1)) t = t1;
        else if (valid(t2)) t = t2;
        if (t >= 0) {
            double lx = x0 + t*dx;
            double ly = y0 + t*dy;
            outTarget = units::Pose(from_in(lx), from_in(ly), 0_stDeg);
            return true;
        }
    }
    outTarget = m_path.back();
    return true;
}

bool PurePursuitController::update(Time dt) {
    if (!m_isFollowing || m_path.size() < 2) { m_isFollowing = false; m_leftMotors.brake(); m_rightMotors.brake(); return false; }

    auto pose = m_poseProvider();
    m_runTime += dt;

    // ── 1. Path progress ───────────────────────────────────────────────
    int segIdx = findClosestSegmentIndex(pose);
    Length distRemaining = 0_in;
    Length progress = computePathProgress(pose, segIdx, distRemaining);

    const units::Pose& goal = m_path.back();
    Length distToGoal = units::hypot(goal.x - pose.x, goal.y - pose.y);

    // ── 2. Exit conditions ─────────────────────────────────────────────
    bool atPos = distToGoal < m_cfg.waypointTolerance;
    bool timeout = m_runTime > 6_sec;

    // Stall detection: if distance to goal stops decreasing, we've overshot
    if (distToGoal < m_lastDistEnd - 0.3_in) {
        // Getting closer — reset counter
        m_lastDistEnd = distToGoal;
        m_distIncreasingCount = 0;
    } else {
        // Not getting closer
        m_distIncreasingCount++;
    }
    bool stalled = (m_distIncreasingCount > 30 && m_runTime > 0.5_sec); // ~300ms stuck

    // Final heading pivot (simple P controller)
    if (m_requireFinalHeading && (atPos || stalled)) {
        Angle hErr = units::constrainAngle180(goal.orientation - pose.orientation);
        if (units::abs(hErr) < m_cfg.finalPivotTolerance || timeout) { m_isFollowing = false; m_leftMotors.brake(); m_rightMotors.brake(); return false; }
        double track = to_in(m_driveConfig.trackWidth);
        double omega = std::clamp(2.0 * hErr.internal(), -1.5, 1.5);
        double vL = -omega * track / 2.0;
        double vR =  omega * track / 2.0;
        auto sgn = [](double v){ return (std::fabs(v) > 0.01) ? (v > 0 ? 1.0 : -1.0) : 0.0; };
        double cmdL = (m_driveConfig.kV.internal() * vL + m_driveConfig.kS.internal() * sgn(vL)) / 12.0;
        double cmdR = (m_driveConfig.kV.internal() * vR + m_driveConfig.kS.internal() * sgn(vR)) / 12.0;
        m_leftMotors.move(Number(std::clamp(cmdL, -1.0, 1.0)));
        m_rightMotors.move(Number(std::clamp(cmdR, -1.0, 1.0)));
        return true;
    }

    if (atPos || stalled || timeout) { m_isFollowing = false; m_leftMotors.brake(); m_rightMotors.brake(); return false; }

    // ── 3. Desired velocity (sample trajectory by DISTANCE, not time) ──
    double desiredVel = to_inps(m_cfg.manualBaseSpeed); // fallback if no trajectory
    if (!m_trajStates.empty()) {
        // Small lookahead offset to bootstrap from vel=0 at the start.
        // Shrinks to 0 once we're past 4 inches so we track the decel zone faithfully.
        Length offset = (progress < 4_in) ? (2_in - progress * 0.5) : 0_in;
        Length sampleDist = progress + offset;
        sampleDist = units::clamp(sampleDist, 0_in, m_totalPathLength);
        auto sample = Trajectory(m_trajStates).sampleByDistance(sampleDist);
        desiredVel = std::fabs(to_inps(sample.velocity));
    }

    // Distance-based deceleration override: √(2·a·d) velocity limit
    // Uses distToGoal (Euclidean) so we stop at the actual endpoint, not path end.
    {
        double decelRate = 60.0; // in/s² — tuned decel capability
        double maxStopVel = std::sqrt(2.0 * decelRate * std::max(to_in(distToGoal), 0.0));
        desiredVel = std::min(desiredVel, maxStopVel);
    }

    // Minimum creep velocity so we always reach the goal
    double minVel = 4.0; // in/s — enough to overcome friction
    desiredVel = std::max(desiredVel, minVel);

    if (m_reversed) desiredVel = -desiredVel;

    // ── 4. Dynamic lookahead based on local curvature ──────────────────
    //   High curvature (tight turn)  → short lookahead (more responsive)
    //   Low  curvature (straight)    → long  lookahead (smoother)
    if (m_cfg.dynamicLookahead && !m_trajStates.empty()) {
        Length sampleDist = progress;
        sampleDist = units::clamp(sampleDist, 0_in, m_totalPathLength);
        auto sample = Trajectory(m_trajStates).sampleByDistance(sampleDist);
        double curv = std::fabs(sample.curvature.internal());
        // Map curvature [dynCurvLow..dynCurvHigh] → lookahead [dynLookMax..dynLookMin]
        double t = std::clamp((curv - m_cfg.dynCurvLow) / (m_cfg.dynCurvHigh - m_cfg.dynCurvLow + 1e-6), 0.0, 1.0);
        m_lookahead = m_cfg.dynLookMax - (m_cfg.dynLookMax - m_cfg.dynLookMin) * t;
    } else {
        m_lookahead = m_cfg.lookahead;
    }

    units::Pose target;
    computeLookaheadPoint(pose, target);

    // ── 5. Curvature from pure pursuit geometry ────────────────────────
    Angle heading = units::constrainAngle180(pose.orientation);
    double cosH = units::cos(heading);
    double sinH = units::sin(heading);
    double dx = to_in(target.x - pose.x);
    double dy = to_in(target.y - pose.y);
    double xLocal =  cosH * dx + sinH * dy;  // forward
    double yLocal = -sinH * dx + cosH * dy;  // left
    if (m_reversed) xLocal = -xLocal;
    double L2 = xLocal * xLocal + yLocal * yLocal;
    double curvature = (L2 > 1e-6) ? (2.0 * yLocal / L2) : 0.0;
    curvature = std::clamp(curvature, -0.5, 0.5);

    // Fade curvature to zero near the goal to prevent end-of-path oversteer
    if (distRemaining < 10_in) {
        curvature *= std::clamp(to_in(distRemaining) / 10.0, 0.0, 1.0);
    }

    // ── 6. Wheel speeds ────────────────────────────────────────────────
    double track = to_in(m_driveConfig.trackWidth);
    double omega = desiredVel * curvature;
    double vL = desiredVel - omega * track / 2.0;
    double vR = desiredVel + omega * track / 2.0;

    // ── 7. Feedforward → motor commands (kV + kS only, no kA) ──────────
    auto sgn = [](double v){ return (std::fabs(v) > 0.01) ? (v > 0 ? 1.0 : -1.0) : 0.0; };
    double kVv = m_driveConfig.kV.internal();
    double kSv = m_driveConfig.kS.internal();
    double cmdL = (kVv * vL + kSv * sgn(vL)) / 12.0;
    double cmdR = (kVv * vR + kSv * sgn(vR)) / 12.0;
    m_leftMotors.move(Number(std::clamp(cmdL, -1.0, 1.0)));
    m_rightMotors.move(Number(std::clamp(cmdR, -1.0, 1.0)));

    return true;
}

void ppTask(void* param) {
    PurePursuitController* self = static_cast<PurePursuitController*>(param);
    while (self->isFollowing()) {
        self->update(10_msec);
        pros::delay(10);
    }
    self->m_taskRunning = false;  // mark dead before RTOS task exits
}

bool PurePursuitController::followPath(bool async, bool reversedDrive) {
    if (m_path.size() < 2) return false;
    m_isFollowing = true;
    if (m_trajStates.empty()) m_reversed = reversedDrive;
    m_runTime = 0_sec;
    m_lastDistEnd = 9999_in;   // large so first update always shows "getting closer"
    m_distIncreasingCount = 0;
    m_lastPoseForSpeed = m_poseProvider();
    m_trajTimeIndex = 0;
    m_lastClosestIndex = 0;
    if (async) {
        if (m_task) {
            if (m_taskRunning) m_task->remove();
            delete m_task;
        }
        m_taskRunning = true;
        m_task = new pros::Task(ppTask, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PurePursuit");
        return true;
    } else {
        while (m_isFollowing) {
            update(10_msec);
            pros::delay(10);
        }
        return true;
    }
}

void PurePursuitController::stop() {
    m_isFollowing = false;
    m_leftMotors.brake();
    m_rightMotors.brake();
    if (m_task) {
        // Give the task time to see m_isFollowing==false and exit its loop
        pros::delay(30);
        // Only remove() if the RTOS task is still alive
        if (m_taskRunning) {
            m_task->remove();
            m_taskRunning = false;
        }
        delete m_task;
        m_task = nullptr;
    }
}

} // namespace motion
