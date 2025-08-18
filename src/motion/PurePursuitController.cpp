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
    Length lookahead,
    Length waypointTolerance
) : m_leftMotors(leftMotors), m_rightMotors(rightMotors), m_driveConfig(driveConfig),
    m_poseProvider(poseProvider), m_lookahead(lookahead),
    m_waypointTolerance(waypointTolerance) {}

void PurePursuitController::setPath(const std::vector<units::Pose>& path) {
    m_path = path;
    m_lastClosestIndex = 0;
    // Precompute cumulative lengths
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
    // Build path (poses) from trajectory states if not manually provided
    m_path.clear();
    m_path.reserve(m_trajStates.size());
    for (auto& s : m_trajStates) m_path.push_back(s.pose);
    m_lastClosestIndex = 0;
    // Build cumulative lengths using distance field if available else compute
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
    // (trajectory debug logging suppressed)
    // Derive reversed mode from first state's velocity sign (if negative)
    if (!m_trajStates.empty()) {
        // Determine reversed by first non-zero velocity sign
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
    int start, end;
    if (m_reversed) {
        start = std::max(0, m_lastClosestIndex - 25);
        end = std::min<int>(m_path.size() - 2, m_lastClosestIndex + 2);
    } else {
        start = std::max(0, m_lastClosestIndex - 2);
        end = std::min<int>(m_path.size() - 2, m_lastClosestIndex + 25);
    }
    for (int i = start; i <= end; ++i) {
        // Distance from point current to segment i-(i+1)
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
    // Project current point onto segment
    auto& p0 = m_path[segIndex]; auto& p1 = m_path[segIndex+1];
    double x0=to_in(p0.x), y0=to_in(p0.y); double x1=to_in(p1.x), y1=to_in(p1.y);
    double dx=x1-x0, dy=y1-y0; double segLen2 = dx*dx+dy*dy + 1e-9; double t = ((to_in(current.x)-x0)*dx + (to_in(current.y)-y0)*dy)/segLen2; t = std::clamp(t,0.0,1.0);
    Length segLen = from_in(std::sqrt(segLen2));
    Length along = m_cumulativeLengths[segIndex] + segLen * t;
    alongDistToEnd = (m_totalPathLength - along);
    // (suppressed periodic debug logging)
    return along;
}

// Compute lookahead point along path at distance m_lookahead from current pose.
// Returns true if a point was found; outTarget set to last waypoint otherwise.
bool PurePursuitController::computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget) {
    if (m_path.size() < 2) { outTarget = current; return false; }
    double L = to_in(m_lookahead);
    // Search forward from closest segment index
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
    // If no intersection found ahead, use final waypoint
    outTarget = m_path.back();
    return true;
}

bool PurePursuitController::update(Time dt) {
    if (!m_isFollowing) return false;
    if (m_path.empty()) { stop(); return false; }

    auto pose = m_poseProvider();
    // --- Exit / completion logic ---
    m_runTime += dt;
    // Choose goal based on direction
    const units::Pose& goal = m_reversed ? m_path.front() : m_path.back();
    Length dxEnd = goal.x - pose.x;
    Length dyEnd = goal.y - pose.y;
    Length distEnd = units::sqrt(units::square(dxEnd) + units::square(dyEnd));

    // Final heading enforcement optional
    Angle finalHeading = goal.orientation;
    Angle headingError = units::constrainAngle180(finalHeading - pose.orientation);
    bool atHeading = !m_requireFinalHeading || units::abs(headingError) < from_stDeg(5);
    // Overshoot detection: if closest segment index is last and distance increases for a few cycles, stop
    if (m_runTime == dt) { // first iteration after reset
        m_lastDistEnd = distEnd;
        m_distIncreasingCount = 0;
    }
    if (distEnd > m_lastDistEnd + 0.5_in && m_lastClosestIndex >= (int)m_path.size()-2) {
        m_distIncreasingCount++;
    } else {
        m_distIncreasingCount = 0;
    }
    m_lastDistEnd = distEnd;

    // Path progress & adaptive slowdown (use along-path remaining primarily)
    Length distRemainingAlong = 0_in; Length progress = computePathProgress(pose, m_lastClosestIndex, distRemainingAlong);
    // For reverse, redefine remaining distance as distance back to start (progress already measured from start)
    if (m_reversed) {
        distRemainingAlong = progress; // distance from start to projection
    }
    // Measured linear speed from pose delta
    Length dPose = units::sqrt(units::square(pose.x - m_lastPoseForSpeed.x) + units::square(pose.y - m_lastPoseForSpeed.y));
    m_lastPoseForSpeed = pose;
    // Convert to in/s given dt
    double speedInPerS = 0.0;
    if (dt > 0_msec) speedInPerS = to_in(dPose) / to_sec(dt);

    // Adaptive velocity scale down to zero over last 18 inches
    double vScale = 1.0;
    if (distRemainingAlong < 18_in) {
        vScale = std::clamp(to_in(distRemainingAlong) / 18.0, 0.0, 1.0);
    }

    // Apply physics-based stopping distance speed cap: v <= sqrt(2 * a_max * remaining)
    if (m_enableStoppingDistanceDecel && m_maxTrajAccel > 0_inps2) {
        double aMax = to_inps2(m_maxTrajAccel);
        double rem = to_in(distRemainingAlong);
        double stopLimit = std::sqrt(std::max(0.0, 2 * aMax * rem)); // in/s
        // We'll enforce this after selecting desiredVel
        // Store for debug if needed (reuse m_lastDistEnd for not adding new member)
    }

    // Position reached if Euclidean OR along-path under tolerance (direction-sensitive)
    // Tighten along-path tolerance (half waypoint tol) to reduce overshoot
    bool atPos = (distEnd < m_waypointTolerance) || (distRemainingAlong < (m_waypointTolerance * 0.5));
    bool lowVelocity = speedInPerS < 1.0; // in/s threshold

    bool timeout = m_runTime > 6_sec; // overall timeout safeguard
    // Overshoot abort only if heading already satisfied OR pivot not required; otherwise allow pivot to finish.
    bool overshootAbort = (m_distIncreasingCount > 4) && (atHeading || !m_finalPivotEnabled);
    if ((atPos && atHeading) || overshootAbort || timeout) { stop(); return false; }

    units::Pose target;
    // Dynamic lookahead adjustment based on current trajectory curvature (if enabled)
    if (m_useDynamicLookahead) {
        double curvAbs = 0.0;
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            curvAbs = std::fabs(m_trajStates[m_trajTimeIndex].curvature.internal());
        } else if (m_lastClosestIndex+1 < (int)m_path.size()) {
            // Approximate curvature from three points if trajectory unavailable
            int i0 = std::max(0, m_lastClosestIndex-1);
            int i1 = m_lastClosestIndex;
            int i2 = std::min<int>(m_path.size()-1, m_lastClosestIndex+1);
            Curvature cval = utils::calculateCurvature(m_path[i0], m_path[i1], m_path[i2]);
            curvAbs = std::fabs(cval.internal());
        }
        double tNorm;
        if (curvAbs <= m_dynCurvLow) tNorm = 0.0;
        else if (curvAbs >= m_dynCurvHigh) tNorm = 1.0;
        else tNorm = (curvAbs - m_dynCurvLow) / (m_dynCurvHigh - m_dynCurvLow);
        // Inverse relation: tighter curve (higher tNorm) => shorter lookahead
        Length dynL = m_dynLookMax - (m_dynLookMax - m_dynLookMin) * tNorm;
        m_lookahead = dynL; // overwrite for this cycle
    }
    // Additional startup alignment: if early in run and heading error large, shrink lookahead to improve turning response
    if (m_runTime < 0.5_sec) {
        // Approximate desired heading from segment end point ahead
        if (m_lastClosestIndex+1 < (int)m_path.size()) {
            auto &p0a = m_path[m_lastClosestIndex];
            auto &p1a = m_path[m_lastClosestIndex+1];
            Angle desiredHeading = units::atan2(p1a.y - p0a.y, p1a.x - p0a.x); // standard frame
            Angle err = units::constrainAngle180(desiredHeading - pose.orientation);
            if (units::abs(err) > from_stDeg(8)) {
                m_lookahead = units::min(m_lookahead, m_useDynamicLookahead ? m_dynLookMin : (m_lookahead * 0.5));
            }
        }
    }
    if (!m_reversed) {
        computeLookaheadPoint(pose, target);
    } else {
        // For reversed, we want lookahead behind current direction (towards start). Build a temporary reversed lookahead.
        // Simple approach: create a mirrored path order on-the-fly for lookahead computation.
        // Copy logic but walking segments backward.
        double Lh = to_in(m_lookahead);
        int segIndex = findClosestSegmentIndex(pose); // still using forward indexing; adjust search direction below
        bool found = false;
        for (int i = segIndex; i >= 0; --i) {
            auto p1 = m_path[i+1]; // forward segment p_i -> p_{i+1}; reversed direction uses p_{i+1} -> p_i
            auto p0 = m_path[i];
            double x1 = to_in(p1.x), y1 = to_in(p1.y);
            double x0 = to_in(p0.x), y0 = to_in(p0.y);
            double dx = x0 - x1; double dy = y0 - y1; // reversed direction vector
            double fx = to_in(pose.x) - x1; double fy = to_in(pose.y) - y1;
            double a = dx*dx + dy*dy;
            double b = 2*(dx*fx + dy*fy);
            double c = fx*fx + fy*fy - Lh*Lh;
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
                double lx = x1 + t*dx;
                double ly = y1 + t*dy;
                target = units::Pose(from_in(lx), from_in(ly), 0_stDeg);
                found = true; break;
            }
        }
        if (!found) target = m_path.front();
    }

    // Transform target to robot frame
    // Normalize heading assuming IMU may provide compass orientation; converting compass->standard is idempotent if already standard
    // Pose orientation is expected in standard math frame already; ensure normalization only.
    Angle heading = units::constrainAngle180(pose.orientation);
    double cosH = units::cos(heading);
    double sinH = units::sin(heading);
    double dx = to_in(target.x - pose.x);
    double dy = to_in(target.y - pose.y);
    double xR =  cosH*dx + sinH*dy; // forward robot frame
    double yR = -sinH*dx + cosH*dy;
    double xF = m_reversed ? -xR : xR; // reflected forward axis when reversing
    double yF = yR; // cross-track error in robot frame (left +, right - if using standard frame derivation)

    // Pure pursuit curvature using virtual forward frame (xF,yF)
    double L = std::max(1e-3, std::hypot(xF, yF));
    double curvature = (2 * yF) / (L * L);
    // Suppress curvature near end to avoid spin (scale with remaining distance)
    if (distRemainingAlong < m_curvatureSuppressDistance && m_curvatureSuppressDistance > 0_in) {
        double denom = to_in(m_curvatureSuppressDistance);
        double factor = std::clamp(to_in(distRemainingAlong) / (denom <= 1e-6 ? 1.0 : denom), 0.0, 1.0);
        curvature *= factor;
    }
    curvature = std::clamp(curvature, -1.0, 1.0);

    // Stanley correction (adds heading rate based on cross track error yR and heading error)
    double stanleyTerm = 0.0; // capture for logging
    double vMagForStanley = 0.0; // speed used in Stanley term denominator
    if (m_stanleyGain.internal() > 1e-6) {
        // Cross track error is yF (lateral in virtual forward frame). Stanley adds term: delta = atan(k * e / (v + soft))
        double vMag = std::fabs(to_inps(m_manualBaseSpeed));
        // If trajectory present, prefer its instantaneous speed magnitude
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            vMag = std::fabs(to_inps(m_trajStates[m_trajTimeIndex].velocity));
        }
        double soft;
        if (m_useStanleyPercent) {
            // Softening proportional to current speed: soft = percent * vMag
            soft = m_stanleySofteningPercent.internal() * std::max(4.0, vMag); // ensure some minimum scaling base
        } else {
            soft = to_inps(m_stanleySofteningVel);
        }
        // Apply end-of-path damping to Stanley gain if enabled
        double gainEff = m_stanleyGain.internal();
        if (m_useStanleyDamping) {
            double remIn = to_in(distRemainingAlong);
            if (remIn < to_in(m_stanleyDampStartDistance)) {
                double t = std::clamp(remIn / to_in(m_stanleyDampStartDistance), 0.0, 1.0); // 1 -> far, 0 -> at goal
                double scale = m_stanleyDampMinScale + (1.0 - m_stanleyDampMinScale) * t;   // linear ramp
                gainEff *= scale;
            }
        }
        stanleyTerm = std::atan(gainEff * yF / (vMag + soft));
        // Quiet zone: if close to end and nearly centered laterally, scale down Stanley to prevent oscillation
        if (distRemainingAlong < m_stanleyQuietDistance && std::fabs(yF) < m_stanleyQuietYF) {
            stanleyTerm *= m_stanleyQuietScale;
        }
    vMagForStanley = vMag;
        // Convert heading offset to equivalent curvature addition: curvature += (delta / Lapprox)
        // Use track projection: omega_extra = omegaGain * delta, then curvature_extra = omega_extra / v
        if (std::fabs(vMag) > 1e-3) {
            double omegaExtra = m_stanleyOmegaGain.internal() * stanleyTerm;
            curvature += omegaExtra / vMag;
        }
        m_lastStanleyGainEffective = gainEff;
    }

    // Determine desired linear velocity & acceleration from trajectory if available
    // Default desired velocity if no trajectory; will be overridden when trajectory states exist
    LinearVelocity desiredVel = m_manualBaseSpeed * vScale;
    LinearAcceleration desiredAcc = 0_inps2;
    if (!m_trajStates.empty()) {
        // Advance trajectory time index according to elapsed run time
        Time tNow = m_runTime; // accumulated before this call
        while (m_trajTimeIndex+1 < m_trajStates.size() && m_trajStates[m_trajTimeIndex+1].timestamp <= tNow) {
            m_trajTimeIndex++;
        }
        desiredVel = m_trajStates[m_trajTimeIndex].velocity;
        desiredAcc = m_trajStates[m_trajTimeIndex].acceleration;
    }

    // Velocity limiting strategies
    double desSign = (to_inps(desiredVel) >= 0) ? 1.0 : -1.0;
    double desMag = std::fabs(to_inps(desiredVel));
    double remIn = std::max(0.0, to_in(distRemainingAlong));

    // 1) Ramp-down clamp already represented by vScale earlier -> optionally apply to trajectory velocity
    desMag *= vScale;

    // 2) Stopping distance limit
    if (m_enableStoppingDistanceDecel && m_maxTrajAccel > 0_inps2) {
        double aMax = to_inps2(m_maxTrajAccel);
        double stopLimit = std::sqrt(std::max(0.0, 2 * aMax * remIn));
        if (desMag > stopLimit) desMag = stopLimit;
    }

    // 3) Final creep zone cap (ensures low speed near end for precision)
    if (distRemainingAlong < m_finalCreepDistance) {
        desMag = std::min(desMag, (double)to_inps(m_finalCreepMaxSpeed));
    }

    desiredVel = from_inps(desSign * desMag);

    // Mild end-phase heading assist (pre-pivot) to reduce residual heading error without fully zeroing linear motion.
    if (m_endHeadingAssistEnabled && !atPos && distRemainingAlong < m_endHeadingAssistStartDist && m_requireFinalHeading) {
        Angle hErr = units::constrainAngle180(finalHeading - pose.orientation);
        double eH = hErr.internal();
        double deH = 0.0;
        if (dt > 0_msec) deH = (eH - m_prevEndAssistHeadingError.internal()) / to_sec(dt);
        m_prevEndAssistHeadingError = hErr;
        double omegaAssist = m_endHeadingAssistKp * eH - m_endHeadingAssistKd * deH;
        omegaAssist = std::clamp(omegaAssist, -m_endHeadingAssistMaxOmega, m_endHeadingAssistMaxOmega);
        // Blend assist into curvature-derived omega by translating to curvature delta (omega / v) if velocity reasonable.
        double vNow = to_inps(desiredVel);
        if (std::fabs(vNow) > 1e-3) {
            curvature += omegaAssist / vNow;
            curvature = std::clamp(curvature, -1.0, 1.0);
        }
    }

    // Heading priority scaling (option 2): reduce linear velocity & lookahead when heading error large near end.
    m_lastHeadingPriorityScale = 1.0;
    if (m_headingPriorityEnabled && !m_trajStates.empty()) { /* applies even with trajectory velocities */ }
    if (m_headingPriorityEnabled && !atPos && distRemainingAlong < m_headingPriorityDistance) {
        Angle hErr = units::constrainAngle180(finalHeading - pose.orientation);
        Angle absErr = units::abs(hErr);
        if (absErr > m_headingPriorityStartError) {
            double startR = m_headingPriorityStartError.internal();
            double maxR = m_headingPriorityMaxError.internal();
            double e = absErr.internal();
            double frac = std::clamp((e - startR) / std::max(1e-6, maxR - startR), 0.0, 1.0);
            double scale = m_headingPriorityMinScale + (1.0 - m_headingPriorityMinScale) * (1.0 - frac); // larger error -> closer to minScale
            // Apply to linear velocity magnitude (preserve sign) and lookahead (floor 2")
            double vNow = to_inps(desiredVel);
            desiredVel = from_inps(vNow * scale);
            m_lookahead = units::max(2_in, m_lookahead * scale);
            m_lastHeadingPriorityScale = scale;
        }
    }

    if (atPos) {
        // Only remove linear velocity; keep possibility to rotate (pivot logic may still run)
        desiredVel = 0_inps; desiredAcc = 0_inps2; }

    // Wheel speed computation
    double track = to_in(m_driveConfig.trackWidth);
    double v = to_inps(desiredVel); // base linear (may be overridden by trajectory direct mode)
    double omegaPP = v * curvature; // pure pursuit induced angular rate
    double vLeft = 0.0, vRight = 0.0;
    bool directTrajMode = !m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size();
    // If we have reached positional goal, stop using trajectory per-wheel velocities to avoid "second lap" drift.
    if (atPos) directTrajMode = false;
    double wheelAccelLeft = 0.0, wheelAccelRight = 0.0; // in/s^2
    // Debug feedforward component storage
    double ffVLeft=0, ffALeft=0, ffSLeft=0, ffVRight=0, ffARight=0, ffSRight=0, ffTotLeft=0, ffTotRight=0;
    // Optional: open-loop scaling (set true temporarily for diagnostics)
    static bool kDirectOpenLoopTest = false; // toggle to test pure ratio output
    if (directTrajMode) {
        // Directly apply trajectory-provided per-wheel velocities (requested behavior)
        const auto &cur = m_trajStates[m_trajTimeIndex];
        vLeft = to_inps(cur.leftWheelVelocity);
        vRight = to_inps(cur.rightWheelVelocity);
        if (m_trajTimeIndex > 0) {
            const auto &prev = m_trajStates[m_trajTimeIndex-1];
            Time dtw = cur.timestamp - prev.timestamp;
            double dts = to_sec(dtw);
            if (dts > 1e-4) {
                wheelAccelLeft = (to_inps(cur.leftWheelVelocity) - to_inps(prev.leftWheelVelocity)) / dts;
                wheelAccelRight = (to_inps(cur.rightWheelVelocity) - to_inps(prev.rightWheelVelocity)) / dts;
            }
        }
    } else {
        // Legacy blended mode (pure pursuit curvature + optional trajectory angular component)
        double baseOmegaTraj = 0.0;
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            const auto &ts = m_trajStates[m_trajTimeIndex];
            double l = to_inps(ts.leftWheelVelocity);
            double r = to_inps(ts.rightWheelVelocity);
            if (std::fabs(l) > 1e-4 || std::fabs(r) > 1e-4) {
                double vTraj = (l + r) / 2.0;
                v = vTraj;
                baseOmegaTraj = (r - l) / track;
            }
        }
        double omega = baseOmegaTraj + omegaPP;
        vLeft = v - omega * track / 2.0;
        vRight = v + omega * track / 2.0;
        // Approximate shared acceleration for feedforward if desiredAcc present
        wheelAccelLeft = wheelAccelRight = to_inps2(desiredAcc);
    }

    // Final heading correction simplified: pivot after position goal or if already in early pivot.
    if (m_requireFinalHeading && atPos && !atHeading) {
        m_inFinalPivot = true; // mark pivot active for logging
        // Treat optional final pivot parameters only as gains; ignore advanced gating.
        Angle hErr = units::constrainAngle180(finalHeading - pose.orientation);
        double e = hErr.internal();
        double de = 0.0;
        if (m_finalPivotEnabled && dt > 0_msec) {
            de = (e - m_prevPivotHeadingError.internal()) / to_sec(dt);
        }
        m_prevPivotHeadingError = hErr;
        double kP = m_finalPivotEnabled ? m_finalPivotKp : 2.0;
        double kD = m_finalPivotEnabled ? m_finalPivotKd : 0.0;
    double omegaCmd = kP * e - kD * de; // Integral disabled for stability
        double maxOmega = m_finalPivotEnabled ? m_finalPivotMaxOmega : 2.5;
        omegaCmd = std::clamp(omegaCmd, -maxOmega, maxOmega);
        // Additional smooth taper near tolerance to avoid residual overshoot with higher Kd
        double tol = (m_finalPivotEnabled ? m_finalPivotTolerance : from_stDeg(3)).internal();
        double magE = std::fabs(e);
        if (magE < tol * 4) { // start tapering within 4x tolerance
            double t = std::clamp(magE / (tol * 4), 0.0, 1.0); // 1 far, 0 at goal
            omegaCmd *= (0.35 + 0.65 * t); // reduce to 35% when at tolerance
        }
        vLeft = -omegaCmd * track / 2.0;
        vRight = omegaCmd * track / 2.0;
        wheelAccelLeft = wheelAccelRight = 0.0;
        desiredVel = 0_inps;
        directTrajMode = false;
    // Stability check
    Angle tolAng = m_finalPivotEnabled ? m_finalPivotTolerance : from_stDeg(3);
    if (units::abs(hErr) < tolAng) {
            m_finalPivotStableCount++;
        } else {
            m_finalPivotStableCount = 0;
        }
        if (m_finalPivotStableCount >= (m_finalPivotEnabled ? m_finalPivotStableCyclesRequired : 8)) {
            atHeading = true; // stop condition will trigger next loop
        }
    } else {
        // Not in pivot; reset stability counter and flag so future pivot isn't biased.
    m_finalPivotStableCount = 0;
    m_inFinalPivot = false;
    }

    // Revised feedforward: apply kS once to average linear component so curvature (wheel difference) isn't penalized by double kS.
    Number leftCmd{0.0}, rightCmd{0.0};
    if (kDirectOpenLoopTest) {
        // Preserve simple open loop diagnostic mode
        static double maxObs = 1e-6;
        maxObs = std::max(maxObs, std::fabs(vLeft));
        maxObs = std::max(maxObs, std::fabs(vRight));
        double pctL = (maxObs < 1e-6) ? 0.0 : (vLeft / (maxObs * 1.1));
        double pctR = (maxObs < 1e-6) ? 0.0 : (vRight / (maxObs * 1.1));
        leftCmd = units::clamp(Number(pctL), Number(-1.0), Number(1.0));
        rightCmd = units::clamp(Number(pctR), Number(-1.0), Number(1.0));
    } else {
        double vAvg = 0.5 * (vLeft + vRight);
        double aAvg = 0.5 * (wheelAccelLeft + wheelAccelRight);
        double dvL = vLeft - vAvg;
        double dvR = vRight - vAvg;
        double daL = wheelAccelLeft - aAvg;
        double daR = wheelAccelRight - aAvg;
        // Base (shared) components
        double baseV = m_driveConfig.kV.internal() * vAvg;
        double baseA = m_driveConfig.kA.internal() * aAvg;
        double baseS = 0.0;
        // If average motion significant apply kS once; for pivots (avg ~0, opposite wheels) apply per-wheel kS so turning still gets static compensation.
        bool pivotLike = std::fabs(vAvg) < 1e-3 && (std::fabs(vLeft - vRight) > 1e-2);
        if (!pivotLike) {
            if (std::fabs(vAvg) > 1e-3) baseS = m_driveConfig.kS.internal() * (vAvg >= 0 ? 1.0 : -1.0);
        }
        // Differential components (no kS here)
        double diffVL = m_driveConfig.kV.internal() * dvL;
        double diffVR = m_driveConfig.kV.internal() * dvR;
        double diffAL = m_driveConfig.kA.internal() * daL;
        double diffAR = m_driveConfig.kA.internal() * daR;
        double voltsL, voltsR;
        if (pivotLike) {
            // Each wheel gets its own kS with its sign in a pivot.
            double sL = (std::fabs(vLeft) > 1e-3) ? m_driveConfig.kS.internal() * (vLeft >= 0 ? 1.0 : -1.0) : 0.0;
            double sR = (std::fabs(vRight) > 1e-3) ? m_driveConfig.kS.internal() * (vRight >= 0 ? 1.0 : -1.0) : 0.0;
            voltsL = (m_driveConfig.kV.internal()*vLeft) + (m_driveConfig.kA.internal()*wheelAccelLeft) + sL;
            voltsR = (m_driveConfig.kV.internal()*vRight) + (m_driveConfig.kA.internal()*wheelAccelRight) + sR;
            ffVLeft = m_driveConfig.kV.internal()*vLeft; ffALeft = m_driveConfig.kA.internal()*wheelAccelLeft; ffSLeft = sL; ffTotLeft = voltsL;
            ffVRight = m_driveConfig.kV.internal()*vRight; ffARight = m_driveConfig.kA.internal()*wheelAccelRight; ffSRight = sR; ffTotRight = voltsR;
        } else {
            voltsL = baseV + baseA + baseS + diffVL + diffAL;
            voltsR = baseV + baseA + baseS + diffVR + diffAR;
            ffVLeft = baseV + diffVL; ffALeft = baseA + diffAL; ffSLeft = baseS; ffTotLeft = voltsL;
            ffVRight = baseV + diffVR; ffARight = baseA + diffAR; ffSRight = baseS; ffTotRight = voltsR;
        }
        leftCmd = units::clamp(Number(voltsL / 12.0), Number(-1.0), Number(1.0));
        rightCmd = units::clamp(Number(voltsR / 12.0), Number(-1.0), Number(1.0));
    }

    m_leftMotors.move(leftCmd);
    m_rightMotors.move(rightCmd);

    static int dbg = 0;
    if (++dbg == 1) {
        // printf("[PP] Debug v2 ACTIVE\n"); // suppressed
    }
    if (false && dbg % 25 == 0) { // disabled periodic debug logging
        double headingDegStdRaw = to_stDeg(pose.orientation);
        double headingDegStd = to_stDeg(heading);
        double headingDegCompass = to_cDeg(pose.orientation);
        double softDbg = m_useStanleyPercent ? (m_stanleySofteningPercent.internal()*100.0) : to_inps(m_stanleySofteningVel);
        double trajVL = 0, trajVR = 0, omegaTraj = 0, omegaPPdbg = omegaPP;
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            trajVL = to_inps(m_trajStates[m_trajTimeIndex].leftWheelVelocity);
            trajVR = to_inps(m_trajStates[m_trajTimeIndex].rightWheelVelocity);
            omegaTraj = (trajVR - trajVL) / to_in(m_driveConfig.trackWidth);
        }
        double actVL = to_inps(utils::toLinear(m_leftMotors.getActualVelocity(), m_driveConfig.wheelDiameter));
        double actVR = to_inps(utils::toLinear(m_rightMotors.getActualVelocity(), m_driveConfig.wheelDiameter));
        // Clamp index printouts to valid range to avoid huge garbage numbers if uninitialized
        int segIdx = std::clamp(m_lastClosestIndex, 0, (int)m_path.size()-1);
        int segMax = std::max(0, (int)m_path.size()-1);
        if (m_logMode == 1) { // verbose
            if (directTrajMode) {
                printf("[PP2] %s DIRECT t=%.2f pos=(%.2f,%.2f) hStd=%.1f vL=%.2f vR=%.2f aL=%.2f aR=%.2f cmdL=%.3f cmdR=%.3f actL=%.2f actR=%.2f seg=%d/%d dRem=%.2f curv=%.4f kStan=%.3f FF(L v=%.2f a=%.2f s=%.2f tot=%.2f | R v=%.2f a=%.2f s=%.2f tot=%.2f) dV=%.3f dPct=%.3f hpSc=%.2f fp=%d hErr=%.1f\n",
                    m_reversed?"REV":"FWD", to_sec(m_runTime), to_in(pose.x), to_in(pose.y), headingDegStd,
                    vLeft, vRight, wheelAccelLeft, wheelAccelRight, leftCmd.internal(), rightCmd.internal(), actVL, actVR,
                    segIdx, segMax, to_in(distRemainingAlong), curvature, m_lastStanleyGainEffective,
                    ffVLeft, ffALeft, ffSLeft, ffTotLeft, ffVRight, ffARight, ffSRight, ffTotRight,
                    (ffTotLeft-ffTotRight), (leftCmd.internal()-rightCmd.internal()), m_lastHeadingPriorityScale, m_inFinalPivot?1:0, to_stDeg(units::constrainAngle180(finalHeading - pose.orientation)));
            } else {
                printf("[PP2] %s BLEND t=%.2f pos=(%.2f,%.2f) hStd=%.1f curv=%.4f kStan=%.3f v=%.2f cmdL=%.2f cmdR=%.2f actL=%.2f actR=%.2f seg=%d/%d dRem=%.2f vSc=%.2f oPP=%.3f oT=%.3f baseK=%.2f sft=%s%.1f trajL=%.2f trajR=%.2f hpSc=%.2f fp=%d hErr=%.1f\n",
                    m_reversed?"REV":"FWD", to_sec(m_runTime), to_in(pose.x), to_in(pose.y), headingDegStd,
                    curvature, m_lastStanleyGainEffective, v, vLeft, vRight, actVL, actVR, segIdx, segMax, to_in(distRemainingAlong), vScale,
                    omegaPPdbg, omegaTraj, m_stanleyGain.internal(), m_useStanleyPercent?"%":"", softDbg,
                    trajVL, trajVR, m_lastHeadingPriorityScale, m_inFinalPivot?1:0, to_stDeg(units::constrainAngle180(finalHeading - pose.orientation)));
            }
        } else { // basic
            // Include odometry pose (x,y) and heading (std & compass) to help diagnose end overshoot / correction behavior.
            double hStdDeg = headingDegStd; // standard math (CCW from +X)
            double hCompDeg = headingDegCompass; // compass (0=N increasing CW)
            // Basic log suppressed
        }
    }
    return true;
}

static void ppTask(void* param) {
    PurePursuitController* self = static_cast<PurePursuitController*>(param);
    while (self->isFollowing()) {
        self->update(10_msec);
        pros::delay(10);
    }
}

bool PurePursuitController::followPath(bool async, bool reversedDrive) {
    if (m_path.size() < 2) return false;
    m_isFollowing = true;
    // Only override reversed if no trajectory (manual path). If trajectory set, its first state velocity defines direction.
    if (m_trajStates.empty()) m_reversed = reversedDrive;
    m_runTime = 0_sec;
    m_lastDistEnd = 0_in;
    m_distIncreasingCount = 0;
    m_lastPoseForSpeed = m_poseProvider();
    m_trajTimeIndex = 0;
    if (m_reversed) {
        m_lastClosestIndex = std::max(0, (int)m_path.size() - 2); // start searching from end
    } else {
        m_lastClosestIndex = 0;
    }
    if (async) {
        if (m_task) { m_task->remove(); delete m_task; }
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
    if (m_task) { m_task->remove(); delete m_task; m_task = nullptr; }
}

} // namespace motion
