#include "motion/PurePursuitController.hpp"
#include "api.h" // provides pros::Task and delay
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
    LinearVelocity cruiseVelocity,
    Length waypointTolerance
) : m_leftMotors(leftMotors), m_rightMotors(rightMotors), m_driveConfig(driveConfig),
    m_poseProvider(poseProvider), m_lookahead(lookahead), m_cruiseVelocity(cruiseVelocity),
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
    // Debug dump first few trajectory states
    int dumpN = std::min<size_t>(5, m_trajStates.size());
    for (int i=0;i<dumpN;i++) {
        auto &st = m_trajStates[i];
        printf("[TRAJ] i=%d t=%.2f x=%.2f y=%.2f vel=%.2f acc=%.2f curv=%.4f\n", i,
            to_sec(st.timestamp), to_in(st.pose.x), to_in(st.pose.y), to_inps(st.velocity), to_inps2(st.acceleration), st.curvature.internal());
    }
    // Derive reversed mode from first state's velocity sign (if negative)
    if (!m_trajStates.empty()) {
        // Determine reversed by first non-zero velocity sign
        m_reversed = false;
        for (auto &s : m_trajStates) {
            double vv = to_inps(s.velocity);
            if (std::fabs(vv) > 1e-3) { m_reversed = vv < 0; break; }
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
    return along;
}

bool PurePursuitController::computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget) {
    if (m_path.size() < 2) return false;
    double L = to_in(m_lookahead);
    int segIndex = findClosestSegmentIndex(current);
    if (!m_reversed) {
        for (int i = segIndex; i < (int)m_path.size() - 1; ++i) {
            auto p0 = m_path[i];
            auto p1 = m_path[i+1];
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
    } else {
        // Simplified reverse: reflect the lookahead search by operating on a reversed index-space without duplicating math
        // Create synthetic point by mirroring path order: treat current segment index from end
        for (int i = segIndex; i < (int)m_path.size()-1; ++i) { // search forward in index, but we pick points behind robot due to reversed flag later
            auto p0 = m_path[i];
            auto p1 = m_path[i+1];
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

    // Position reached if Euclidean OR along-path under tolerance (direction-sensitive)
    bool atPos = (distEnd < m_waypointTolerance) || (distRemainingAlong < m_waypointTolerance);
    bool lowVelocity = speedInPerS < 1.0; // in/s threshold

    bool timeout = m_runTime > 6_sec; // shorter overall timeout
    // Stop immediately when position & heading reached (don't wait for lowVelocity, reduces overshoot)
    if ((atPos && atHeading) || m_distIncreasingCount > 4 || timeout) { stop(); return false; }

    units::Pose target;
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
    Angle heading = from_cDeg(to_cDeg(pose.orientation));
    double cosH = units::cos(heading);
    double sinH = units::sin(heading);
    double dx = to_in(target.x - pose.x);
    double dy = to_in(target.y - pose.y);
    double xR =  cosH*dx + sinH*dy; // forward robot frame
    double yR = -sinH*dx + cosH*dy;
    double xF = m_reversed ? -xR : xR; // reflected forward axis when reversing
    double yF = yR;

    // Pure pursuit curvature using virtual forward frame (xF,yF)
    double L = std::max(1e-3, std::hypot(xF, yF));
    double curvature = (2 * yF) / (L * L);
    // Suppress curvature near end to avoid spin (scale with remaining distance)
    if (distRemainingAlong < 6_in) {
        double factor = std::clamp(to_in(distRemainingAlong) / 6.0, 0.0, 1.0);
        curvature *= factor;
    }
    curvature = std::clamp(curvature, -1.0, 1.0);

    // Stanley correction (adds heading rate based on cross track error yR and heading error)
    if (m_stanleyGain.internal() > 1e-6) {
        // Cross track error is yR (lateral in robot frame). Desired heading is line from robot to lookahead point.
        // Heading error already implicit in pure pursuit; Stanley adds term: delta = atan(k * e / (v + soft))
    double vMag = std::fabs(to_inps(m_cruiseVelocity));
        double soft = to_inps(m_stanleySoftening);
    double stanleyTerm = std::atan(m_stanleyGain.internal() * yF / (vMag + soft));
        // Convert heading offset to equivalent curvature addition: curvature += (delta / Lapprox)
        // Use track projection: omega_extra = omegaGain * delta, then curvature_extra = omega_extra / v
        if (std::fabs(vMag) > 1e-3) {
            double omegaExtra = m_stanleyOmegaGain.internal() * stanleyTerm;
            curvature += omegaExtra / vMag;
        }
    }

    // Determine desired linear velocity & acceleration from trajectory if available
    LinearVelocity desiredVel = m_cruiseVelocity * vScale;
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

    // Additional end-of-path velocity clamp (helps mitigate open-loop coasting)
    Length remainingForClamp = distRemainingAlong;
    double remIn = std::max(0.0, to_in(remainingForClamp));
    double clampDist = 18.0; // inches over which to linearly ramp down max velocity
    double maxMag = (remIn >= clampDist) ? std::fabs(to_inps(desiredVel)) : std::fabs(to_inps(desiredVel)) * (remIn / clampDist);
    if (remIn < clampDist) {
        double curMag = std::fabs(to_inps(desiredVel));
        if (curMag > maxMag) {
            double sign = (to_inps(desiredVel) >= 0) ? 1.0 : -1.0;
            desiredVel = from_inps(sign * maxMag);
        }
    }
    // If atPos reached, force zero velocity
    if (atPos) { desiredVel = 0_inps; desiredAcc = 0_inps2; }

    // Compute wheel speeds (linear velocity + differential based on curvature)
    double v = to_inps(desiredVel); // may be negative for reverse trajectories
    double omega = v * curvature; // rad/s approx (since v in in/s and curvature rad/in)

    // Differential drive inverse kinematics
    double track = to_in(m_driveConfig.trackWidth);
    double vLeft = v - omega * track / 2.0;
    double vRight = v + omega * track / 2.0;

    // Convert to percent using simple linear scaling with kV, kS (ignore accel here)
    auto calcFF = [&](double linVelInPerS) {
        LinearVelocity lv = from_inps(linVelInPerS);
        Number volts = m_driveConfig.kV * to_mps(lv);
        // Add static friction if moving
        if (std::fabs(linVelInPerS) > 1e-3) volts += m_driveConfig.kS * units::sgn(lv);
        // Simple accel feedforward using average of desiredAcc
        volts += m_driveConfig.kA * to_mps2(desiredAcc);
        double percent = volts.internal() / 12.0;
        return units::clamp(Number(percent), Number(-1.0), Number(1.0));
    };

    Number leftCmd = calcFF(vLeft);
    Number rightCmd = calcFF(vRight);

    m_leftMotors.move(leftCmd);
    m_rightMotors.move(rightCmd);

    static int dbg = 0;
    if (++dbg % 10 == 0) {
        double headingDegStdRaw = to_stDeg(pose.orientation);
        double headingDegStd = to_stDeg(heading);
        double headingDegCompass = to_cDeg(pose.orientation);
        double tgtHeadingDeg = headingDegStd; // orientation target not enforced
    printf("[PP] %s x=%.2f y=%.2f hRaw=%.1f hStd=%.1f hComp=%.1f tx=%.2f ty=%.2f xR=%.2f yR=%.2f xF=%.2f yF=%.2f curv=%.4f v=%.2f a=%.2f l=%.2f r=%.2f seg=%d/%d k=%.2f soft=%.2f dEnd=%.2f rem=%.2f vSc=%.2f atPos=%d atHead=%d lowV=%d inc=%d t=%.2f spd=%.2f rev=%d traj=%d ti=%zu tt=%.2f\n",
            m_reversed ? "REV" : "FWD",
            to_in(pose.x), to_in(pose.y), headingDegStdRaw, headingDegStd, headingDegCompass,
            to_in(target.x), to_in(target.y), xR, yR, xF, yF, curvature, v, to_inps2(desiredAcc), vLeft, vRight, m_lastClosestIndex, (int)m_path.size()-1,
            m_stanleyGain.internal(), to_inps(m_stanleySoftening), to_in(distEnd), to_in(distRemainingAlong), vScale,
            atPos, atHeading, lowVelocity, m_distIncreasingCount, to_sec(m_runTime), speedInPerS, m_reversed ? 1:0, m_trajStates.empty()?0:1, m_trajTimeIndex, m_trajStates.empty()?0.0:to_sec(m_trajStates[m_trajTimeIndex].timestamp));
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
