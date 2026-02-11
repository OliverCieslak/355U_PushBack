#pragma once

#include "hardware/Encoder/Encoder.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "utils/Utils.hpp"

namespace odometry {

/**
 * @brief Odometry using one tracking wheel and an IMU
 *
 * The single tracking wheel is mounted parallel to the robot's forward
 * direction (a "vertical" tracking wheel). It measures forward/backward
 * displacement while the IMU provides heading. Because there is no lateral
 * encoder, lateral slip is invisible to this system — but it is still far
 * more accurate than using drive motor encoders because tracking wheels
 * do not suffer from wheel scrub during turns.
 *
 * Coordinate convention (standard math):
 *   +X = right, +Y = forward, θ = 0 along +X, positive CCW
 *
 * The tracking wheel may be offset from the robot's center of rotation.
 * If so, provide the signed lateral offset (positive = wheel is to the
 * right of center, negative = to the left). The algorithm will compensate
 * for the arc the wheel traces during a turn.
 */
class OneWheelOdometry {
public:
    /**
     * @brief Construct a new One-Wheel + IMU Odometry object
     *
     * @param forwardEncoder   Encoder on the forward-facing tracking wheel
     * @param imu              V5 Inertial Sensor for heading
     * @param wheelDiameter    Diameter of the tracking wheel
     * @param forwardOffset    Signed lateral offset of the tracking wheel
     *                         from the robot's center of rotation
     *                         (positive = right of center)
     * @param initialPose      Starting pose of the robot
     */
    OneWheelOdometry(
        lemlib::Encoder& forwardEncoder,
        lemlib::IMU& imu,
        Length wheelDiameter,
        Length forwardOffset = 0_in,
        const units::Pose& initialPose = units::Pose()
    );

    ~OneWheelOdometry();

    /** @brief Start the background update task (10 ms period) */
    void start();

    /** @brief Stop the background update task */
    void stop();

    /** @brief Perform one odometry update and return the new pose */
    units::Pose update();

    /** @brief Get the current pose (thread-safe) */
    units::Pose getPose() const;

    /** @brief Reset the pose to a known value */
    void resetPose(const units::Pose& pose);

    /** @brief Get the current linear and angular velocities */
    std::pair<LinearVelocity, AngularVelocity> getVelocity() const;

    /** @brief Print diagnostic information */
    void printDiagnostics() const;

private:
    lemlib::Encoder* m_forwardEncoder;
    lemlib::IMU* m_imu;
    Length m_wheelDiameter;
    Length m_forwardOffset;   // lateral offset from center of rotation
    units::Pose m_pose;

    mutable pros::Mutex m_mutex;

    Angle m_prevForwardPos = 0_stRad;
    Angle m_prevHeading = 0_stRad;
    double m_imuOffset = 0.0;

    // Task management
    pros::Task* m_updateTask = nullptr;
    bool m_isRunning = false;
    static void updateTaskTrampoline(void* param);
    void taskUpdate();

    // Velocity tracking
    uint32_t m_lastUpdateTime = 0;
    LinearVelocity m_linearVelocity = 0_mps;
    AngularVelocity m_angularVelocity = 0_radps;
    units::Pose m_prevPose;
};

} // namespace odometry
