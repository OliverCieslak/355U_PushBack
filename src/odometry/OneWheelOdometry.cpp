#include "odometry/OneWheelOdometry.hpp"

namespace odometry {

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

OneWheelOdometry::OneWheelOdometry(
    lemlib::Encoder& forwardEncoder,
    lemlib::IMU& imu,
    Length wheelDiameter,
    Length forwardOffset,
    const units::Pose& initialPose
) : m_forwardEncoder(&forwardEncoder),
    m_imu(&imu),
    m_wheelDiameter(wheelDiameter),
    m_forwardOffset(forwardOffset),
    m_pose(initialPose),
    m_prevPose(initialPose),
    m_imuOffset(0.0)
{
    m_prevForwardPos = m_forwardEncoder->getAngle();
    m_prevHeading    = m_imu->getRotation();
}

OneWheelOdometry::~OneWheelOdometry() { stop(); }

// ---------------------------------------------------------------------------
// Task management
// ---------------------------------------------------------------------------

void OneWheelOdometry::start() {
    if (!m_isRunning) {
        m_updateTask = new pros::Task(updateTaskTrampoline, this,
                                      TASK_PRIORITY_DEFAULT,
                                      TASK_STACK_DEPTH_DEFAULT,
                                      "1-Wheel Odom");
        m_isRunning = true;
    }
}

void OneWheelOdometry::stop() {
    if (m_isRunning) {
        m_isRunning = false;
        if (m_updateTask) { delete m_updateTask; m_updateTask = nullptr; }
    }
}

void OneWheelOdometry::updateTaskTrampoline(void* param) {
    static_cast<OneWheelOdometry*>(param)->taskUpdate();
}

void OneWheelOdometry::taskUpdate() {
    uint32_t now = pros::millis();
    while (m_isRunning) {
        update();
        pros::Task::delay_until(&now, 10); // wake every 10ms from last wake
    }
}

// ---------------------------------------------------------------------------
// Core update
// ---------------------------------------------------------------------------

units::Pose OneWheelOdometry::update() {
    // ---- sample sensors ----
    uint32_t now = pros::micros();
    Angle currentForwardPos = m_forwardEncoder->getAngle();
    Angle currentHeading    = units::constrainAngle180(
        m_imu->getRotation() + from_stDeg(m_imuOffset));

    Angle forwardDelta = currentForwardPos - m_prevForwardPos;
    Angle deltaTheta   = currentHeading - m_prevHeading;

    // ---- RAII mutex guard ----
    struct Guard {
        pros::Mutex& m; Guard(pros::Mutex& mx) : m(mx) { m.take(20); }
        ~Guard() { m.give(); }
    } guard(m_mutex);

    // ---- convert encoder delta to linear distance ----
    Length wheelRadius     = m_wheelDiameter / 2.0;
    Length rawForwardDist  = to_stRad(forwardDelta) * wheelRadius;

    // ---- noise gate ----
    if (units::abs(rawForwardDist) < 0.2_mm &&
        units::abs(deltaTheta) < 0.001_stDeg) {
        m_prevForwardPos = currentForwardPos;
        m_prevHeading    = currentHeading;
        return m_pose;
    }

    // ---- compensate for wheel offset ----
    // When the robot turns, an offset tracking wheel traces an arc of
    // radius = offset around the center of rotation.  The extra distance
    // the wheel reads is:  offset * Δθ   (arc = r·θ).
    // Subtract this to get the true forward displacement of the robot center.
    Length forwardDist = rawForwardDist - m_forwardOffset * to_stRad(deltaTheta);

    // ---- local displacement in the midpoint-heading frame ----
    //  • forward component  = s · sinc(Δθ/2)
    //  • lateral component  = 0  (no horizontal encoder)
    // With only one wheel, lateral slip is unobservable.
    Length localX = forwardDist;
    if (units::abs(deltaTheta) >= 0.001_stDeg) {
        localX = Length(forwardDist * utils::sinc(deltaTheta / 2.0));
    }

    // ---- rotate into global frame using midpoint angle ----
    Angle midAngle  = m_pose.orientation + deltaTheta / 2.0;
    double cosH = units::cos(midAngle);
    double sinH = units::sin(midAngle);

    m_pose.x += localX * cosH;
    m_pose.y += localX * sinH;
    m_pose.orientation = currentHeading;

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
    m_prevForwardPos = currentForwardPos;
    m_prevHeading    = currentHeading;

    return out;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

units::Pose OneWheelOdometry::getPose() const {
    m_mutex.take(20);
    units::Pose p = m_pose;
    m_mutex.give();
    return p;
}

void OneWheelOdometry::resetPose(const units::Pose& pose) {
    m_mutex.take(20);
    m_pose           = pose;
    m_prevForwardPos = m_forwardEncoder->getAngle();
    m_imuOffset      = to_stDeg(units::constrainAngle180(
                           pose.orientation - m_imu->getRotation()));
    m_prevHeading    = pose.orientation;
    m_prevPose       = pose;
    m_mutex.give();
}

std::pair<LinearVelocity, AngularVelocity> OneWheelOdometry::getVelocity() const {
    m_mutex.take(20);
    auto v = std::make_pair(m_linearVelocity, m_angularVelocity);
    m_mutex.give();
    return v;
}

void OneWheelOdometry::printDiagnostics() const {
    m_mutex.take(20);
    std::cout << "[1-Wheel Odom] Pose: ("
              << to_in(m_pose.x) << ", "
              << to_in(m_pose.y) << ", "
              << to_cDeg(m_pose.orientation) << ")\n";
    if (m_forwardEncoder)
        std::cout << "  Forward Encoder: "
                  << to_stDeg(m_forwardEncoder->getAngle()) << " deg\n";
    if (m_imu)
        std::cout << "  IMU Heading: "
                  << to_cDeg(m_imu->getRotation()) << " deg\n";
    std::cout << "  Linear Vel: "  << m_linearVelocity.internal()  << " in/s\n";
    std::cout << "  Angular Vel: " << m_angularVelocity.internal() * 180.0 / M_PI
              << " deg/s\n";
    m_mutex.give();
}

} // namespace odometry
