#include "odometry/TwoWheelOdometry.hpp"

namespace odometry {

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

TwoWheelOdometry::TwoWheelOdometry(
    lemlib::Encoder& verticalEncoder,
    lemlib::Encoder& horizontalEncoder,
    lemlib::IMU& imu,
    Length verticalWheelDiam,
    Length horizontalWheelDiam,
    Length verticalOffset,
    Length horizontalOffset,
    const units::Pose& initialPose
) : m_verticalEncoder(&verticalEncoder),
    m_horizontalEncoder(&horizontalEncoder),
    m_imu(&imu),
    m_verticalWheelDiam(verticalWheelDiam),
    m_horizontalWheelDiam(horizontalWheelDiam),
    m_verticalOffset(verticalOffset),
    m_horizontalOffset(horizontalOffset),
    m_pose(initialPose),
    m_prevPose(initialPose),
    m_imuOffset(0.0)
{
    m_prevVerticalPos   = m_verticalEncoder->getAngle();
    m_prevHorizontalPos = m_horizontalEncoder->getAngle();
    m_prevHeading       = m_imu->getRotation();
}

TwoWheelOdometry::~TwoWheelOdometry() { stop(); }

// ---------------------------------------------------------------------------
// Task management
// ---------------------------------------------------------------------------

void TwoWheelOdometry::start() {
    if (!m_isRunning) {
        m_updateTask = new pros::Task(updateTaskTrampoline, this,
                                      TASK_PRIORITY_DEFAULT,
                                      TASK_STACK_DEPTH_DEFAULT,
                                      "2-Wheel Odom");
        m_isRunning = true;
    }
}

void TwoWheelOdometry::stop() {
    if (m_isRunning) {
        m_isRunning = false;
        if (m_updateTask) { delete m_updateTask; m_updateTask = nullptr; }
    }
}

void TwoWheelOdometry::updateTaskTrampoline(void* param) {
    static_cast<TwoWheelOdometry*>(param)->taskUpdate();
}

void TwoWheelOdometry::taskUpdate() {
    uint32_t now = pros::millis();
    while (m_isRunning) {
        update();
        pros::Task::delay_until(&now, 10); // wake every 10ms from last wake
    }
}

// ---------------------------------------------------------------------------
// Core update
// ---------------------------------------------------------------------------

units::Pose TwoWheelOdometry::update() {
    // ---- sample sensors ----
    uint32_t now = pros::micros();
    Angle curVertPos  = m_verticalEncoder->getAngle();
    Angle curHorizPos = m_horizontalEncoder->getAngle();
    Angle curHeading  = units::constrainAngle180(
        m_imu->getRotation() + from_stDeg(m_imuOffset));

    Angle vertDelta  = curVertPos  - m_prevVerticalPos;
    Angle horizDelta = curHorizPos - m_prevHorizontalPos;
    Angle deltaTheta = curHeading  - m_prevHeading;

    // ---- RAII mutex guard ----
    struct Guard {
        pros::Mutex& m; Guard(pros::Mutex& mx) : m(mx) { m.take(20); }
        ~Guard() { m.give(); }
    } guard(m_mutex);

    // ---- convert encoder deltas to linear distances ----
    Length vertRadius  = m_verticalWheelDiam   / 2.0;
    Length horizRadius = m_horizontalWheelDiam / 2.0;
    Length rawForward  = to_stRad(vertDelta)  * vertRadius;
    Length rawLateral  = to_stRad(horizDelta) * horizRadius;

    // ---- noise gate ----
    if (units::abs(rawForward) < 0.2_mm &&
        units::abs(rawLateral) < 0.2_mm &&
        units::abs(deltaTheta) < 0.001_stDeg) {
        m_prevVerticalPos   = curVertPos;
        m_prevHorizontalPos = curHorizPos;
        m_prevHeading       = curHeading;
        return m_pose;
    }

    // ---- compensate for wheel offsets ----
    //
    // When the robot turns through Δθ, each offset tracking wheel traces
    // an arc around the center of rotation.
    //
    // Vertical wheel (measures forward):
    //   The wheel is at a lateral offset d_v from center.
    //   Extra arc it reads = d_v · Δθ
    //   True center forward distance = raw - d_v · Δθ
    //
    // Horizontal wheel (measures lateral):
    //   The wheel is at a longitudinal offset d_h from center.
    //   Extra arc it reads = d_h · Δθ
    //   True center lateral distance = raw - d_h · Δθ
    //
    double dThetaRad = to_stRad(deltaTheta);
    Length forwardDist = rawForward - m_verticalOffset   * dThetaRad;
    Length lateralDist = rawLateral - m_horizontalOffset * dThetaRad;

    // ---- local displacement in the midpoint-heading frame ----
    //
    // For a robot that simultaneously moves forward by s_fwd and laterally
    // by s_lat while turning through Δθ, the chord displacement in the
    // midpoint-heading frame is:
    //
    //   local_x (forward)  = s_fwd · sinc(Δθ/2)
    //   local_y (lateral)  = s_lat · sinc(Δθ/2)
    //
    // Both components use sinc(Δθ/2) because the midpoint rotation absorbs
    // the directional change, leaving a straight chord in that frame.
    //
    Length localForward = 0_m;
    Length localLateral = 0_m;

    if (units::abs(deltaTheta) < 0.001_stDeg) {
        localForward = forwardDist;
        localLateral = lateralDist;
    } else {
        double sincHalf = utils::sinc(deltaTheta / 2.0);
        localForward = forwardDist * sincHalf;
        localLateral = lateralDist * sincHalf;
    }

    // ---- rotate into global frame using midpoint angle ----
    Angle midAngle = m_pose.orientation + deltaTheta / 2.0;
    double cosH = units::cos(midAngle);
    double sinH = units::sin(midAngle);

    // Rotation matrix applied to (forward, lateral) in the midpoint frame:
    //   Δx_global = forward·cos(mid) - lateral·sin(mid)
    //   Δy_global = forward·sin(mid) + lateral·cos(mid)
    m_pose.x += localForward * cosH - localLateral * sinH;
    m_pose.y += localForward * sinH + localLateral * cosH;
    m_pose.orientation = curHeading;

    // ---- velocity estimation ----
    uint32_t dtUs = now - m_lastUpdateTime;
    if (dtUs > 5000) {
        Length dx = m_pose.x - m_prevPose.x;
        Length dy = m_pose.y - m_prevPose.y;
        m_linearVelocity  = units::sqrt(dx * dx + dy * dy) / from_usec(dtUs);
        Angle dTheta = units::constrainAngle180(
            from_stDeg(to_stDeg(m_pose.orientation) - to_stDeg(m_prevPose.orientation)));
        m_angularVelocity = dTheta / from_usec(dtUs);
        m_prevPose        = m_pose;
        m_lastUpdateTime  = now;
    }

    units::Pose out = m_pose;

    // ---- save for next iteration ----
    m_prevVerticalPos   = curVertPos;
    m_prevHorizontalPos = curHorizPos;
    m_prevHeading       = curHeading;

    return out;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

units::Pose TwoWheelOdometry::getPose() const {
    m_mutex.take(20);
    units::Pose p = m_pose;
    m_mutex.give();
    return p;
}

void TwoWheelOdometry::resetPose(const units::Pose& pose) {
    m_mutex.take(20);
    m_pose              = pose;
    m_prevVerticalPos   = m_verticalEncoder->getAngle();
    m_prevHorizontalPos = m_horizontalEncoder->getAngle();
    m_imuOffset         = to_stDeg(units::constrainAngle180(
                              pose.orientation - m_imu->getRotation()));
    m_prevHeading       = pose.orientation;
    m_prevPose          = pose;
    m_mutex.give();
}

std::pair<LinearVelocity, AngularVelocity> TwoWheelOdometry::getVelocity() const {
    m_mutex.take(20);
    auto v = std::make_pair(m_linearVelocity, m_angularVelocity);
    m_mutex.give();
    return v;
}

void TwoWheelOdometry::printDiagnostics() const {
    m_mutex.take(20);
    std::cout << "[2-Wheel Odom] Pose: ("
              << to_in(m_pose.x) << ", "
              << to_in(m_pose.y) << ", "
              << to_cDeg(m_pose.orientation) << ")\n";
    if (m_verticalEncoder)
        std::cout << "  Vertical Encoder: "
                  << to_stDeg(m_verticalEncoder->getAngle()) << " deg\n";
    if (m_horizontalEncoder)
        std::cout << "  Horizontal Encoder: "
                  << to_stDeg(m_horizontalEncoder->getAngle()) << " deg\n";
    if (m_imu)
        std::cout << "  IMU Heading: "
                  << to_cDeg(m_imu->getRotation()) << " deg\n";
    std::cout << "  Linear Vel: "  << m_linearVelocity.internal()  << " in/s\n";
    std::cout << "  Angular Vel: " << m_angularVelocity.internal() * 180.0 / M_PI
              << " deg/s\n";
    m_mutex.give();
}

} // namespace odometry
