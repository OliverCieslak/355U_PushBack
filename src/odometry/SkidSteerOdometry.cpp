#include "odometry/SkidSteerOdometry.hpp"

namespace odometry {
void SkidSteerOdometry::printDiagnostics() const {
    m_mutex.take(20);
    std::cout << "[Odometry Diagnostics] Pose: (" << to_in(m_pose.x) << ", " << to_in(m_pose.y) << ", " << to_cDeg(m_pose.orientation) << ")\n";
    if (m_leftEncoder && m_rightEncoder) {
        std::cout << "  Left Encoder Angle: " << to_stDeg(m_leftEncoder->getAngle()) << " deg\n";
        std::cout << "  Right Encoder Angle: " << to_stDeg(m_rightEncoder->getAngle()) << " deg\n";
    }
    if (m_imu) {
        std::cout << "  IMU Heading: " << to_cDeg(m_imu->getRotation()) << " deg\n";
    }
    std::cout << "  Linear Velocity: " << m_linearVelocity.internal() << " in/s\n";
    std::cout << "  Angular Velocity: " << m_angularVelocity.internal() * 180.0 / M_PI << " deg/s\n";
    m_mutex.give();
}

SkidSteerOdometry::SkidSteerOdometry(
    lemlib::Encoder& leftEncoder,
    lemlib::Encoder& rightEncoder,
    lemlib::IMU& imu,
    Length trackWidth,
    Length wheelDiameter,
    const units::Pose& initialPose
) : m_leftEncoder(&leftEncoder),
    m_rightEncoder(&rightEncoder),
    m_imu(&imu),
    m_trackWidth(trackWidth),
    m_wheelDiameter(wheelDiameter),
    m_pose(initialPose) {
    // Initialize previous encoder readings
    m_prevLeftPosition = m_leftEncoder->getAngle();
    m_prevRightPosition = m_rightEncoder->getAngle();
    m_prevHeading = m_imu->getRotation();
}

SkidSteerOdometry::SkidSteerOdometry(
    lemlib::MotorGroup& leftMotors,
    lemlib::MotorGroup& rightMotors,
    lemlib::IMU& imu,
    Length trackWidth,
    Length wheelDiameter,
    const units::Pose& initialPose
) : m_leftEncoder(&leftMotors),
    m_rightEncoder(&rightMotors),
    m_imu(&imu),
    m_trackWidth(trackWidth),
    m_wheelDiameter(wheelDiameter),
    m_pose(initialPose) {
    // Initialize previous encoder readings
    m_prevLeftPosition = m_leftEncoder->getAngle();
    m_prevRightPosition = m_rightEncoder->getAngle();
    m_prevHeading = m_imu->getRotation();
    m_imuOffset = 0.0; // Initialize m_imuOffset
}

SkidSteerOdometry::~SkidSteerOdometry() {
    stop(); // Ensure the task is stopped when the object is destroyed
}

void SkidSteerOdometry::start() {
    if (!m_isRunning) {
        m_updateTask = new pros::Task(updateTaskTrampoline, this, TASK_PRIORITY_DEFAULT,
                                      TASK_STACK_DEPTH_DEFAULT, "Odometry Update");
        m_isRunning = true;
    }
}

void SkidSteerOdometry::stop() {
    if (m_isRunning) {
        m_isRunning = false;
        if (m_updateTask != nullptr) {
            delete m_updateTask;
            m_updateTask = nullptr;
        }
    }
}

void SkidSteerOdometry::updateTaskTrampoline(void* param) {
    SkidSteerOdometry* odometry = static_cast<SkidSteerOdometry*>(param);
    odometry->taskUpdate();
}

void SkidSteerOdometry::taskUpdate() {
    uint32_t now = pros::millis();
    while (m_isRunning) {
        update();
        pros::Task::delay_until(&now, 10); // wake every 10ms from last wake
    }
}

units::Pose SkidSteerOdometry::update() {
    // Record the time immediately before reading sensors for velocity calculations
    uint32_t currentSensorTime = pros::micros();

    // Get current encoder and IMU readings
    Angle currentLeftPosition = m_leftEncoder->getAngle();
    Angle currentRightPosition = m_rightEncoder->getAngle();
    // Use IMU standard angle plus standard-degree offset, normalized to [-180, 180)
    Angle currentHeading = units::constrainAngle180(m_imu->getRotation() + from_stDeg(m_imuOffset));

    // Calculate changes in encoder readings
    Angle leftDelta = currentLeftPosition - m_prevLeftPosition;
    Angle rightDelta = currentRightPosition - m_prevRightPosition;

    // Lock mutex before updating pose (RAII pattern)
    struct MutexGuard {
        pros::Mutex& mutex;
        MutexGuard(pros::Mutex& m) : mutex(m) { mutex.take(20); }
        ~MutexGuard() { mutex.give(); }
    } guard(m_mutex);
    
    // Calculate wheel distances
    Length wheelRadius = m_wheelDiameter / 2.0;
    Length leftDistance = to_stRad(leftDelta) * wheelRadius;
    Length rightDistance = to_stRad(rightDelta) * wheelRadius;

    // Use pure geometric approach - no scaling factors
    Length arcLength = (leftDistance + rightDistance) / 2.0;
    Angle arcAngle = currentHeading - m_pose.orientation;
    
    // Calculate local movement in the midpoint-heading frame
    // When rotating by the midpoint angle (θ₀ + Δθ/2), the chord from
    // start to end is purely along the forward axis of that frame:
    //   chord = 2R·sin(Δθ/2) = s·sinc(Δθ/2)  where s = arc length
    // The lateral component is exactly zero in the midpoint frame.
    units::Vector2D<Length> localMovement;
    
    if (units::abs(arcAngle) < 0.001_stDeg) {
        // Robot moved straight
        localMovement = {arcLength, 0_m};
    } else {
        // Robot moved in an arc — chord length in midpoint frame
        Length chordLength = arcLength * utils::sinc(arcAngle / 2.0);
        localMovement = {chordLength, 0_m};
    }
    
    // Calculate global movement using midpoint angle for better accuracy
    Angle midpointAngle = m_pose.orientation + arcAngle / 2.0;
    double cosHeading = units::cos(midpointAngle);
    double sinHeading = units::sin(midpointAngle);
    
    // Apply rotation matrix
    m_pose.x += localMovement.x * cosHeading - localMovement.y * sinHeading;
    m_pose.y += localMovement.x * sinHeading + localMovement.y * cosHeading;
    m_pose.orientation = currentHeading;
    
    // Calculate velocities if enough time has passed
    uint32_t deltaTimeUs = currentSensorTime - m_lastUpdateTime;
    if (deltaTimeUs > 5000) { // Only update velocity if at least 5ms has passed
        // Calculate linear velocity
        Length dx = m_pose.x - m_prevPose.x;
        Length dy = m_pose.y - m_prevPose.y;
        Length distance = units::sqrt(dx * dx + dy * dy);
        m_linearVelocity = distance / from_usec(deltaTimeUs);
        
        // Calculate angular velocity (handle angle wrapping)
        Angle dtheta = from_stDeg(to_stDeg(m_pose.orientation) - to_stDeg(m_prevPose.orientation));
        // Normalize the angle difference to -180 to 180 degrees
        dtheta = units::constrainAngle180(dtheta);
        m_angularVelocity = dtheta / from_usec(deltaTimeUs);
        
        // Update stored values for next calculation
        m_prevPose = m_pose;
        m_lastUpdateTime = currentSensorTime;
    }
    
    // Make a copy of pose before unlocking
    units::Pose currentPose = m_pose;
    
    // Mutex released automatically by guard

    // Save current readings for next update
    m_prevLeftPosition = currentLeftPosition;
    m_prevRightPosition = currentRightPosition;
    m_prevHeading = currentHeading;

    return currentPose;
}

std::pair<LinearVelocity, AngularVelocity> SkidSteerOdometry::getVelocity() const {
    m_mutex.take(20);
    auto velocities = std::make_pair(m_linearVelocity, m_angularVelocity);
    m_mutex.give();
    return velocities;
}

units::Pose SkidSteerOdometry::getPose() const {
    // Lock mutex before accessing pose
    m_mutex.take(20);
    units::Pose currentPose = m_pose;
    m_mutex.give();
    return currentPose;
}

void SkidSteerOdometry::resetPose(const units::Pose& pose) {
    m_mutex.take(20);
    m_pose = pose;
    m_prevLeftPosition = m_leftEncoder->getAngle();
    m_prevRightPosition = m_rightEncoder->getAngle();
    // m_imu->setRotation(pose.orientation); // Disabled due to hang
    // Store IMU offset in standard degrees for consistent math
    m_imuOffset = to_stDeg(units::constrainAngle180(pose.orientation - m_imu->getRotation()));
    m_prevHeading = pose.orientation;
    m_mutex.give();
}

units::Vector2D<Length> SkidSteerOdometry::calculateDisplacement(
    Angle leftDelta, Angle rightDelta, Angle headingDelta
) {
    Length wheelRadius = m_wheelDiameter / 2.0;
    Length leftDistance = to_stRad(leftDelta) * wheelRadius;
    Length rightDistance = to_stRad(rightDelta) * wheelRadius;
    Length arcLength = (leftDistance + rightDistance) / 2.0;

    if (units::abs(headingDelta) < 0.001_stDeg) {
        return units::Vector2D<Length>(arcLength, 0_m);
    }
    
    // Chord length in the midpoint-heading frame (lateral = 0)
    Length chordLength = arcLength * utils::sinc(headingDelta / 2.0);
    return units::Vector2D<Length>(chordLength, 0_m);
}

} // namespace odometry
