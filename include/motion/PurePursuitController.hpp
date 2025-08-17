#pragma once

#include <vector>
#include <functional>
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp" // for angle literals like 0_stDeg
#include "control/DifferentialDriveConfig.hpp"
#include "utils/Utils.hpp"
#include "hardware/Motor/MotorGroup.hpp" // full definition needed
#include "motion/Trajectory.hpp"

namespace motion {

class PurePursuitController {
public:
    PurePursuitController(
        lemlib::MotorGroup& leftMotors,
        lemlib::MotorGroup& rightMotors,
        const control::DifferentialDriveConfig& driveConfig,
        std::function<units::Pose()> poseProvider,
        Length lookahead = 6_in,
        LinearVelocity cruiseVelocity = 12_inps,
        Length waypointTolerance = 1_in
    );

    void setPath(const std::vector<units::Pose>& path);
    void setTrajectory(const motion::Trajectory& traj); // supplies velocity/accel profiles
    void setLookahead(Length L) { m_lookahead = L; }
    void setCruiseVelocity(LinearVelocity v) { m_cruiseVelocity = v; }
    void setWaypointTolerance(Length tol) { m_waypointTolerance = tol; }
    void setReversed(bool r) { m_reversed = r; }
    bool isReversed() const { return m_reversed; }
    // Stanley controller tuning: gain (dimensionless), softening (in/s for low-speed), omega gain (rad/s per rad)
    void setStanleyGains(Number gain, LinearVelocity softening, Number omegaGain = Number(1.0)) {
        m_stanleyGain = gain; m_stanleySoftening = softening; m_stanleyOmegaGain = omegaGain;
    }

    // Synchronous follow (blocking)
    // reversedDrive = true will drive the path in reverse (robot backing along path)
    bool followPath(bool async = false, bool reversedDrive = false);

    bool isFollowing() const { return m_isFollowing; }
    void stop();

    // Expose update so background task lambda can call it without friendship
    bool update(Time dt);

private:
    lemlib::MotorGroup& m_leftMotors;
    lemlib::MotorGroup& m_rightMotors;
    control::DifferentialDriveConfig m_driveConfig;
    std::function<units::Pose()> m_poseProvider;

    std::vector<units::Pose> m_path; // only x,y used; orientation optional
    std::vector<motion::TrajectoryState> m_trajStates; // optional detailed states for velocity/accel
    size_t m_trajTimeIndex = 0; // index of last trajectory state whose timestamp <= run time
    std::vector<Length> m_cumulativeLengths; // cumulative path lengths per waypoint
    Length m_totalPathLength = 0_in;
    Length m_lookahead;
    LinearVelocity m_cruiseVelocity;
    Length m_waypointTolerance;

    bool m_isFollowing = false;
    int m_lastClosestIndex = 0;
    bool m_reversed = false; // drive backwards along path if true
    bool m_requireFinalHeading = false; // optional heading enforcement
    units::Pose m_lastPose; // for velocity estimation
    // Per-run state
    Time m_runTime = 0_sec;
    Length m_lastDistEnd = 0_in;
    int m_distIncreasingCount = 0;
    units::Pose m_lastPoseForSpeed; // previous pose for speed calc
    // Stanley parameters
    Number m_stanleyGain = Number(0.0);
    LinearVelocity m_stanleySoftening = 1_inps; // avoids division by zero at low speeds
    Number m_stanleyOmegaGain = Number(1.0);

    pros::Task* m_task = nullptr;

    int findClosestSegmentIndex(const units::Pose& current);
    bool computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget);
    Length computePathProgress(const units::Pose& current, int segIndex, Length& alongDistToEnd);
public:
    void setRequireFinalHeading(bool require) { m_requireFinalHeading = require; }
};

} // namespace motion
