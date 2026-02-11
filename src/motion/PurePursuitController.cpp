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
    int start, end;
    if (m_reversed) {
        start = std::max(0, m_lastClosestIndex - 25);
        end = std::min<int>(m_path.size() - 2, m_lastClosestIndex + 2);
    } else {
        start = std::max(0, m_lastClosestIndex - 2);
        end = std::min<int>(m_path.size() - 2, m_lastClosestIndex + 25);
    }
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
    if (!m_isFollowing) return false;
    if (m_path.empty()) { stop(); return false; }

    auto pose = m_poseProvider();
    m_runTime += dt;

    // --- Goal & distance computation ---
    const units::Pose& goal = m_reversed ? m_path.front() : m_path.back();
    Length dxEnd = goal.x - pose.x;
    Length dyEnd = goal.y - pose.y;
    Length distEnd = units::sqrt(units::square(dxEnd) + units::square(dyEnd));

    Angle finalHeading = goal.orientation;
    Angle headingError = units::constrainAngle180(finalHeading - pose.orientation);
    bool atHeading = !m_requireFinalHeading || units::abs(headingError) < from_stDeg(5);

    // Overshoot detection
    if (m_runTime == dt) { m_lastDistEnd = distEnd; m_distIncreasingCount = 0; }
    if (distEnd > m_lastDistEnd + 0.5_in && m_lastClosestIndex >= (int)m_path.size()-2) {
        m_distIncreasingCount++;
    } else {
        m_distIncreasingCount = 0;
    }
    m_lastDistEnd = distEnd;

    // Path progress
    Length distRemainingAlong = 0_in;
    Length progress = computePathProgress(pose, m_lastClosestIndex, distRemainingAlong);
    if (m_reversed) distRemainingAlong = progress;

    // Measured speed
    Length dPose = units::sqrt(units::square(pose.x - m_lastPoseForSpeed.x) + units::square(pose.y - m_lastPoseForSpeed.y));
    m_lastPoseForSpeed = pose;
    double speedInPerS = (dt > 0_msec) ? (to_in(dPose) / to_sec(dt)) : 0.0;

    // Velocity ramp-down over last 18 inches
    double vScale = 1.0;
    if (distRemainingAlong < 18_in) {
        vScale = std::clamp(to_in(distRemainingAlong) / 18.0, 0.0, 1.0);
    }

    // Arrival check
    bool atPos = (distEnd < m_cfg.waypointTolerance) || (distRemainingAlong < (m_cfg.waypointTolerance * 0.5));
    bool timeout = m_runTime > 6_sec;
    bool overshootAbort = (m_distIncreasingCount > 4) && (atHeading || !m_cfg.finalPivot);
    if ((atPos && atHeading) || overshootAbort || timeout) { stop(); return false; }

    // --- Lookahead ---
    units::Pose target;
    if (m_cfg.dynamicLookahead) {
        double curvAbs = 0.0;
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            curvAbs = std::fabs(m_trajStates[m_trajTimeIndex].curvature.internal());
        } else if (m_lastClosestIndex+1 < (int)m_path.size()) {
            int i0 = std::max(0, m_lastClosestIndex-1);
            int i1 = m_lastClosestIndex;
            int i2 = std::min<int>(m_path.size()-1, m_lastClosestIndex+1);
            curvAbs = std::fabs(utils::calculateCurvature(m_path[i0], m_path[i1], m_path[i2]).internal());
        }
        double tNorm;
        if (curvAbs <= m_cfg.dynCurvLow) tNorm = 0.0;
        else if (curvAbs >= m_cfg.dynCurvHigh) tNorm = 1.0;
        else tNorm = (curvAbs - m_cfg.dynCurvLow) / (m_cfg.dynCurvHigh - m_cfg.dynCurvLow);
        m_lookahead = m_cfg.dynLookMax - (m_cfg.dynLookMax - m_cfg.dynLookMin) * tNorm;
    }
    // Startup alignment: shrink lookahead if heading error large
    if (m_runTime < 0.5_sec && m_lastClosestIndex+1 < (int)m_path.size()) {
        auto &p0a = m_path[m_lastClosestIndex];
        auto &p1a = m_path[m_lastClosestIndex+1];
        Angle desiredHeading = units::atan2(p1a.y - p0a.y, p1a.x - p0a.x);
        Angle err = units::constrainAngle180(desiredHeading - pose.orientation);
        if (units::abs(err) > from_stDeg(8)) {
            m_lookahead = units::min(m_lookahead, m_cfg.dynamicLookahead ? m_cfg.dynLookMin : (m_lookahead * 0.5));
        }
    }

    if (!m_reversed) {
        computeLookaheadPoint(pose, target);
    } else {
        double Lh = to_in(m_lookahead);
        int segIndex = findClosestSegmentIndex(pose);
        bool found = false;
        for (int i = segIndex; i >= 0; --i) {
            auto p1 = m_path[i+1]; auto p0 = m_path[i];
            double x1 = to_in(p1.x), y1 = to_in(p1.y);
            double x0 = to_in(p0.x), y0 = to_in(p0.y);
            double dx = x0 - x1; double dy = y0 - y1;
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
                target = units::Pose(from_in(x1 + t*dx), from_in(y1 + t*dy), 0_stDeg);
                found = true; break;
            }
        }
        if (!found) target = m_path.front();
    }

    // --- Robot-frame transform ---
    Angle heading = units::constrainAngle180(pose.orientation);
    double cosH = units::cos(heading);
    double sinH = units::sin(heading);
    double dx = to_in(target.x - pose.x);
    double dy = to_in(target.y - pose.y);
    double xR =  cosH*dx + sinH*dy;
    double yR = -sinH*dx + cosH*dy;
    double xF = m_reversed ? -xR : xR;
    double yF = yR;

    // --- Curvature ---
    double L = std::max(1e-3, std::hypot(xF, yF));
    double curvature = (2 * yF) / (L * L);
    if (distRemainingAlong < m_cfg.curvatureSuppressDist && m_cfg.curvatureSuppressDist > 0_in) {
        double denom = to_in(m_cfg.curvatureSuppressDist);
        curvature *= std::clamp(to_in(distRemainingAlong) / (denom <= 1e-6 ? 1.0 : denom), 0.0, 1.0);
    }
    curvature = std::clamp(curvature, -1.0, 1.0);

    // --- Stanley cross-track correction ---
    if (m_cfg.stanleyGain > 1e-6) {
        double lateralErr = yF;
        double vMag = std::fabs(to_inps(m_cfg.manualBaseSpeed));
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            vMag = std::fabs(to_inps(m_trajStates[m_trajTimeIndex].velocity));
        }
        double soft;
        if (m_stanleyUseVelMode) {
            soft = to_inps(m_stanleySofteningVel);
        } else {
            soft = m_cfg.stanleySoftening * std::max(4.0, vMag);
        }
        double gainEff = m_cfg.stanleyGain;
        if (m_cfg.stanleyDamping) {
            double remIn = to_in(distRemainingAlong);
            if (remIn < to_in(m_cfg.stanleyDampStart)) {
                double t = std::clamp(remIn / to_in(m_cfg.stanleyDampStart), 0.0, 1.0);
                gainEff *= m_cfg.stanleyDampMin + (1.0 - m_cfg.stanleyDampMin) * t;
            }
        }
        double stanleyTerm = std::atan(gainEff * lateralErr / (vMag + soft));
        if (distRemainingAlong < m_cfg.stanleyQuietDist && std::fabs(lateralErr) < m_cfg.stanleyQuietYF) {
            stanleyTerm *= m_cfg.stanleyQuietScale;
        }
        if (std::fabs(vMag) > 1e-3) {
            curvature += m_cfg.stanleyOmegaGain * stanleyTerm / vMag;
        }
        m_lastStanleyGainEffective = gainEff;
    }

    // --- Desired velocity ---
    LinearVelocity desiredVel = m_cfg.manualBaseSpeed * vScale;
    LinearAcceleration desiredAcc = 0_inps2;
    if (!m_trajStates.empty()) {
        Time tNow = m_runTime;
        while (m_trajTimeIndex+1 < m_trajStates.size() && m_trajStates[m_trajTimeIndex+1].timestamp <= tNow) {
            m_trajTimeIndex++;
        }
        desiredVel = m_trajStates[m_trajTimeIndex].velocity;
        desiredAcc = m_trajStates[m_trajTimeIndex].acceleration;
    }

    // Velocity limiting
    double desSign = (to_inps(desiredVel) >= 0) ? 1.0 : -1.0;
    double desMag = std::fabs(to_inps(desiredVel));
    double remIn = std::max(0.0, to_in(distRemainingAlong));

    desMag *= vScale;

    if (m_cfg.stoppingDistanceDecel && m_maxTrajAccel > 0_inps2) {
        double stopLimit = std::sqrt(std::max(0.0, 2 * to_inps2(m_maxTrajAccel) * remIn));
        if (desMag > stopLimit) desMag = stopLimit;
    }

    if (distRemainingAlong < m_cfg.finalCreepDist) {
        desMag = std::min(desMag, (double)to_inps(m_cfg.finalCreepSpeed));
    }

    desiredVel = from_inps(desSign * desMag);

    // --- Pre-pivot heading assist ---
    if (m_cfg.headingAssist && !atPos && distRemainingAlong < m_cfg.headingAssistDist && m_requireFinalHeading) {
        Angle hErr = units::constrainAngle180(finalHeading - pose.orientation);
        double eH = hErr.internal();
        double deH = (dt > 0_msec) ? (eH - m_prevEndAssistHeadingError.internal()) / to_sec(dt) : 0.0;
        m_prevEndAssistHeadingError = hErr;
        double omegaAssist = m_cfg.headingAssistKp * eH - m_cfg.headingAssistKd * deH;
        omegaAssist = std::clamp(omegaAssist, -m_cfg.headingAssistMaxOmega, m_cfg.headingAssistMaxOmega);
        double vNow = to_inps(desiredVel);
        if (std::fabs(vNow) > 1e-3) {
            curvature += omegaAssist / vNow;
            curvature = std::clamp(curvature, -1.0, 1.0);
        }
    }

    // --- Heading priority: slow down when heading error large near end ---
    m_lastHeadingPriorityScale = 1.0;
    if (m_cfg.headingPriority && !atPos && distRemainingAlong < m_cfg.headingPriorityDist) {
        Angle hErr = units::constrainAngle180(finalHeading - pose.orientation);
        Angle absErr = units::abs(hErr);
        if (absErr > m_cfg.headingPriorityStart) {
            double startR = m_cfg.headingPriorityStart.internal();
            double maxR = m_cfg.headingPriorityMax.internal();
            double e = absErr.internal();
            double frac = std::clamp((e - startR) / std::max(1e-6, maxR - startR), 0.0, 1.0);
            double scale = m_cfg.headingPriorityMin + (1.0 - m_cfg.headingPriorityMin) * (1.0 - frac);
            double vNow = to_inps(desiredVel);
            desiredVel = from_inps(vNow * scale);
            m_lookahead = units::max(2_in, m_lookahead * scale);
            m_lastHeadingPriorityScale = scale;
        }
    }

    if (atPos) { desiredVel = 0_inps; desiredAcc = 0_inps2; }

    // --- Wheel speeds ---
    double track = to_in(m_driveConfig.trackWidth);
    double v = to_inps(desiredVel);
    double omegaPP = v * curvature;
    double vLeft = 0.0, vRight = 0.0;
    bool directTrajMode = !m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size();
    if (atPos) directTrajMode = false;
    double wheelAccelLeft = 0.0, wheelAccelRight = 0.0;

    if (directTrajMode) {
        const auto &cur = m_trajStates[m_trajTimeIndex];
        vLeft = to_inps(cur.leftWheelVelocity);
        vRight = to_inps(cur.rightWheelVelocity);
        if (m_trajTimeIndex > 0) {
            const auto &prev = m_trajStates[m_trajTimeIndex-1];
            double dts = to_sec(cur.timestamp - prev.timestamp);
            if (dts > 1e-4) {
                wheelAccelLeft = (to_inps(cur.leftWheelVelocity) - to_inps(prev.leftWheelVelocity)) / dts;
                wheelAccelRight = (to_inps(cur.rightWheelVelocity) - to_inps(prev.rightWheelVelocity)) / dts;
            }
        }
    } else {
        double baseOmegaTraj = 0.0;
        if (!m_trajStates.empty() && m_trajTimeIndex < m_trajStates.size()) {
            const auto &ts = m_trajStates[m_trajTimeIndex];
            double l = to_inps(ts.leftWheelVelocity);
            double r = to_inps(ts.rightWheelVelocity);
            if (std::fabs(l) > 1e-4 || std::fabs(r) > 1e-4) {
                v = (l + r) / 2.0;
                baseOmegaTraj = (r - l) / track;
            }
        }
        double omega = baseOmegaTraj + omegaPP;
        vLeft = v - omega * track / 2.0;
        vRight = v + omega * track / 2.0;
        wheelAccelLeft = wheelAccelRight = to_inps2(desiredAcc);
    }

    // --- Final pivot heading correction ---
    if (m_requireFinalHeading && atPos && !atHeading) {
        m_inFinalPivot = true;
        Angle hErr = units::constrainAngle180(finalHeading - pose.orientation);
        double e = hErr.internal();
        double de = 0.0;
        if (m_cfg.finalPivot && dt > 0_msec) {
            de = (e - m_prevPivotHeadingError.internal()) / to_sec(dt);
        }
        m_prevPivotHeadingError = hErr;
        double kP = m_cfg.finalPivot ? m_cfg.finalPivotKp : 2.0;
        double kD = m_cfg.finalPivot ? m_cfg.finalPivotKd : 0.0;
        double omegaCmd = kP * e - kD * de;
        double maxOmega = m_cfg.finalPivot ? m_cfg.finalPivotMaxOmega : 2.5;
        omegaCmd = std::clamp(omegaCmd, -maxOmega, maxOmega);
        double tol = (m_cfg.finalPivot ? m_cfg.finalPivotTolerance : from_stDeg(3)).internal();
        double magE = std::fabs(e);
        if (magE < tol * 4) {
            omegaCmd *= (0.35 + 0.65 * std::clamp(magE / (tol * 4), 0.0, 1.0));
        }
        vLeft = -omegaCmd * track / 2.0;
        vRight = omegaCmd * track / 2.0;
        wheelAccelLeft = wheelAccelRight = 0.0;
        desiredVel = 0_inps;
        directTrajMode = false;

        Angle tolAng = m_cfg.finalPivot ? m_cfg.finalPivotTolerance : from_stDeg(3);
        if (units::abs(hErr) < tolAng) {
            m_finalPivotStableCount++;
        } else {
            m_finalPivotStableCount = 0;
        }
        int cyclesNeeded = m_cfg.finalPivot ? m_cfg.finalPivotStableCycles : 8;
        if (m_finalPivotStableCount >= cyclesNeeded) {
            atHeading = true;
        }
    } else {
        m_finalPivotStableCount = 0;
        m_inFinalPivot = false;
    }

    // --- Feedforward & motor output ---
    Number leftCmd{0.0}, rightCmd{0.0};
    double vAvg = 0.5 * (vLeft + vRight);
    double aAvg = 0.5 * (wheelAccelLeft + wheelAccelRight);
    bool pivotLike = std::fabs(vAvg) < 1e-3 && (std::fabs(vLeft - vRight) > 1e-2);

    if (pivotLike) {
        auto sgnFF = [&](double vel) { return (std::fabs(vel) > 1e-3) ? m_driveConfig.kS.internal() * (vel >= 0 ? 1.0 : -1.0) : 0.0; };
        double voltsL = m_driveConfig.kV.internal()*vLeft + m_driveConfig.kA.internal()*wheelAccelLeft + sgnFF(vLeft);
        double voltsR = m_driveConfig.kV.internal()*vRight + m_driveConfig.kA.internal()*wheelAccelRight + sgnFF(vRight);
        leftCmd = units::clamp(Number(voltsL / 12.0), Number(-1.0), Number(1.0));
        rightCmd = units::clamp(Number(voltsR / 12.0), Number(-1.0), Number(1.0));
    } else {
        double dvL = vLeft - vAvg, dvR = vRight - vAvg;
        double daL = wheelAccelLeft - aAvg, daR = wheelAccelRight - aAvg;
        double baseV = m_driveConfig.kV.internal() * vAvg;
        double baseA = m_driveConfig.kA.internal() * aAvg;
        double baseS = (std::fabs(vAvg) > 1e-3) ? m_driveConfig.kS.internal() * (vAvg >= 0 ? 1.0 : -1.0) : 0.0;
        double voltsL = baseV + baseA + baseS + m_driveConfig.kV.internal()*dvL + m_driveConfig.kA.internal()*daL;
        double voltsR = baseV + baseA + baseS + m_driveConfig.kV.internal()*dvR + m_driveConfig.kA.internal()*daR;
        leftCmd = units::clamp(Number(voltsL / 12.0), Number(-1.0), Number(1.0));
        rightCmd = units::clamp(Number(voltsR / 12.0), Number(-1.0), Number(1.0));
    }

    m_leftMotors.move(leftCmd);
    m_rightMotors.move(rightCmd);

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
    if (m_trajStates.empty()) m_reversed = reversedDrive;
    m_runTime = 0_sec;
    m_lastDistEnd = 0_in;
    m_distIncreasingCount = 0;
    m_lastPoseForSpeed = m_poseProvider();
    m_trajTimeIndex = 0;
    if (m_reversed) {
        m_lastClosestIndex = std::max(0, (int)m_path.size() - 2);
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
