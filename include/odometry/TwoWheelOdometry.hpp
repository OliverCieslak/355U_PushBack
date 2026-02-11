#pragma once

#include "hardware/Encoder/Encoder.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "utils/Utils.hpp"

namespace odometry {

/**
 * @brief Odometry using two tracking wheels and an IMU
 *
 * Uses one vertical (forward-facing) tracking wheel and one horizontal
 * (sideways-facing) tracking wheel, plus an IMU for heading.
 *
 * This is the gold-standard for VEX odometry — the vertical wheel
 * measures forward/backward displacement, the horizontal wheel measures
 * lateral displacement (strafing / turning slip), and the IMU gives a
 * stable heading reference. Together they fully observe the robot's 2D
 * motion without any of the wheel-scrub problems of drive motor encoders.
 *
 * Coordinate convention (standard math):
 *   +X = right, +Y = forward, θ = 0 along +X, positive CCW
 *
 * Both tracking wheels may be offset from the robot's center of rotation:
 *   • verticalOffset  — signed lateral distance of the vertical wheel
 *                        from the center (positive = right of center)
 *   • horizontalOffset — signed longitudinal distance of the horizontal
 *                         wheel from the center (positive = in front of center)
 *
 * The algorithm compensates for the arcs traced by offset wheels during turns.
 */
class TwoWheelOdometry {
public:
    /**
     * @brief Construct a new Two-Wheel + IMU Odometry object
     *
     * @param verticalEncoder    Encoder on the forward-facing tracking wheel
     * @param horizontalEncoder  Encoder on the sideways-facing tracking wheel
     * @param imu                V5 Inertial Sensor for heading
     * @param verticalWheelDiam  Diameter of the vertical tracking wheel
     * @param horizontalWheelDiam Diameter of the horizontal tracking wheel
     * @param verticalOffset     Lateral offset of vertical wheel from center
     *                           (positive = right of center)
     * @param horizontalOffset   Longitudinal offset of horizontal wheel from center
     *                           (positive = in front of center)
     * @param initialPose        Starting pose of the robot
     */
    TwoWheelOdometry(
        lemlib::Encoder& verticalEncoder,
        lemlib::Encoder& horizontalEncoder,
        lemlib::IMU& imu,
        Length verticalWheelDiam,
        Length horizontalWheelDiam,
        Length verticalOffset = 0_in,
        Length horizontalOffset = 0_in,
        const units::Pose& initialPose = units::Pose()
    );

    ~TwoWheelOdometry();

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
    lemlib::Encoder* m_verticalEncoder;
    lemlib::Encoder* m_horizontalEncoder;
    lemlib::IMU* m_imu;
    Length m_verticalWheelDiam;
    Length m_horizontalWheelDiam;
    Length m_verticalOffset;     // lateral offset of vertical wheel
    Length m_horizontalOffset;   // longitudinal offset of horizontal wheel
    units::Pose m_pose;

    mutable pros::Mutex m_mutex;

    Angle m_prevVerticalPos = 0_stRad;
    Angle m_prevHorizontalPos = 0_stRad;
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
