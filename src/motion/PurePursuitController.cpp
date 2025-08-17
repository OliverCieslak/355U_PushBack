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
}

void PurePursuitController::setTrajectory(const motion::Trajectory& traj) {
    m_trajStates = traj.getStates();
    // Build path (poses) from trajectory states if not manually provided
    m_path.clear();
    m_path.reserve(m_trajStates.size());
    for (auto& s : m_trajStates) m_path.push_back(s.pose);
    m_lastClosestIndex = 0;
}

int PurePursuitController::findClosestSegmentIndex(const units::Pose& current) {
    if (m_path.size() < 2) return 0;
    int bestIndex = m_lastClosestIndex;
    Length bestDist = 1e9_in;
    int start = std::max(0, m_lastClosestIndex - 5);
    int end = std::min<int>(m_path.size() - 2, m_lastClosestIndex + 15);
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

bool PurePursuitController::computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget) {
    if (m_path.size() < 2) return false;
    double L = to_in(m_lookahead);
    int segIndex = findClosestSegmentIndex(current);
    for (int i = segIndex; i < (int)m_path.size() - 1; ++i) {
        auto p0 = m_path[i];
        auto p1 = m_path[i+1];
        // Represent as p0 + d*(p1-p0)
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
    // Fall back to final point
    outTarget = m_path.back();
    return true;
}

bool PurePursuitController::update(Time dt) {
    if (!m_isFollowing) return false;
    if (m_path.empty()) { stop(); return false; }

    auto pose = m_poseProvider();
    // Check final goal proximity
    Length dxEnd = m_path.back().x - pose.x;
    Length dyEnd = m_path.back().y - pose.y;
    Length distEnd = units::sqrt(units::square(dxEnd) + units::square(dyEnd));
    if (distEnd < m_waypointTolerance) { stop(); return false; }

    units::Pose target;
    computeLookaheadPoint(pose, target);

    // Transform target to robot frame
    Angle heading = pose.orientation; // assume already standard
    double cosH = units::cos(heading);
    double sinH = units::sin(heading);
    double dx = to_in(target.x - pose.x);
    double dy = to_in(target.y - pose.y);
    double xR =  cosH*dx + sinH*dy;
    double yR = -sinH*dx + cosH*dy;

    // Pure pursuit curvature = 2*y / L^2
    double L = std::max(1e-3, std::hypot(xR, yR));
    double curvature = (2 * yR) / (L * L);

    // Stanley correction (adds heading rate based on cross track error yR and heading error)
    if (m_stanleyGain.internal() > 1e-6) {
        // Cross track error is yR (lateral in robot frame). Desired heading is line from robot to lookahead point.
        // Heading error already implicit in pure pursuit; Stanley adds term: delta = atan(k * e / (v + soft))
        double vMag = std::fabs(to_inps(m_cruiseVelocity));
        double soft = to_inps(m_stanleySoftening);
        double stanleyTerm = std::atan(m_stanleyGain.internal() * yR / (vMag + soft));
        // Convert heading offset to equivalent curvature addition: curvature += (delta / Lapprox)
        // Use track projection: omega_extra = omegaGain * delta, then curvature_extra = omega_extra / v
        if (std::fabs(vMag) > 1e-3) {
            double omegaExtra = m_stanleyOmegaGain.internal() * stanleyTerm * (m_reversed ? -1.0 : 1.0); // reverse sign if backing
            curvature += omegaExtra / (vMag * (m_reversed ? -1.0 : 1.0));
        }
    }

    // Determine desired linear velocity & acceleration from trajectory if available
    LinearVelocity desiredVel = m_cruiseVelocity;
    LinearAcceleration desiredAcc = 0_inps2;
    if (!m_trajStates.empty()) {
        // Approximate distance traveled along path using closest path point index cumulative distance if provided
        // Find nearest state (by index) using lastClosestIndex
        int idx = m_lastClosestIndex;
        if (idx >= 0 && idx < (int)m_trajStates.size()) {
            desiredVel = m_trajStates[idx].velocity;
            desiredAcc = m_trajStates[idx].acceleration;
        }
    }

    // Compute wheel speeds (linear velocity + differential based on curvature)
    double v = to_inps(desiredVel);
    if (m_reversed) v = -v; // reverse base velocity
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

    // If reversed, swap wheel commands to maintain correct curvature while backing up
    if (m_reversed) {
        m_leftMotors.move(rightCmd);
        m_rightMotors.move(leftCmd);
    } else {
        m_leftMotors.move(leftCmd);
        m_rightMotors.move(rightCmd);
    }

    static int dbg = 0;
    if (++dbg % 10 == 0) {
        printf("[PP] %s x=%.2f y=%.2f tx=%.2f ty=%.2f xR=%.2f yR=%.2f curv=%.4f v=%.2f a=%.2f l=%.2f r=%.2f seg=%d/%d k=%.2f soft=%.2f\n",
            m_reversed ? "REV" : "FWD",
            to_in(pose.x), to_in(pose.y), to_in(target.x), to_in(target.y), xR, yR, curvature, v, to_inps2(desiredAcc), vLeft, vRight, m_lastClosestIndex, (int)m_path.size()-1,
            m_stanleyGain.internal(), to_inps(m_stanleySoftening));
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
    m_reversed = reversedDrive;
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
