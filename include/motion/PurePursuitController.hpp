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

/**
 * @brief Consolidated tuning constants for PurePursuitController.
 *
 * All path-following tuning lives here. Construct with defaults and override
 * only the fields you care about. Pass to the controller via setConfig().
 */
struct PurePursuitConfig {
    // ── Lookahead ──────────────────────────────────────────────────────
    Length lookahead          = 6_in;   ///< Base lookahead distance
    Length waypointTolerance  = 1_in;   ///< Position "arrived" radius

    // Dynamic lookahead: shrinks lookahead in tight curves
    bool   dynamicLookahead  = true;
    Length dynLookMin         = 8_in;   ///< Lookahead at max curvature
    Length dynLookMax         = 14_in;  ///< Lookahead on straights
    double dynCurvLow         = 1.0;    ///< Curvature treated as straight
    double dynCurvHigh        = 12.0;   ///< Curvature treated as tight

    // ── Stanley cross-track correction ─────────────────────────────────
    double stanleyGain        = 0.25;   ///< Cross-track gain (k)
    double stanleySoftening   = 0.30;   ///< Softening as fraction of speed
    double stanleyOmegaGain   = 1.0;    ///< Omega multiplier

    // Stanley end-of-path damping (prevents fishtail near goal)
    bool   stanleyDamping     = true;
    Length stanleyDampStart    = 10_in;  ///< Begin ramping gain down here
    double stanleyDampMin     = 0.10;   ///< Minimum gain fraction at goal

    // Stanley quiet zone (suppress when nearly centered near end)
    Length stanleyQuietDist   = 6_in;
    double stanleyQuietYF     = 1.2;    ///< Lateral error threshold (in)
    double stanleyQuietScale  = 0.12;   ///< Scale factor when in quiet zone

    // ── End-of-path heading ────────────────────────────────────────────
    // Heading priority: slow down when heading error is large near end
    bool   headingPriority    = true;
    Length headingPriorityDist = 16_in;
    Angle  headingPriorityStart = from_stDeg(5);  ///< Begin slowing above this
    Angle  headingPriorityMax   = from_stDeg(45); ///< Error for max slowdown
    double headingPriorityMin   = 0.15;           ///< Min velocity scale

    // Pre-pivot heading assist: blends small omega before positional arrival
    bool   headingAssist      = true;
    Length headingAssistDist   = 12_in;
    double headingAssistKp    = 1.0;
    double headingAssistKd    = 0.25;
    double headingAssistMaxOmega = 1.4;

    // Final pivot: PD turn-in-place after positional arrival
    bool   finalPivot         = true;
    Angle  finalPivotTolerance = from_stDeg(3);
    double finalPivotKp       = 2.0;
    double finalPivotKd       = 0.6;
    double finalPivotMaxOmega = 1.7;    ///< Max rotational speed (rad/s)
    int    finalPivotStableCycles = 4;  ///< Cycles within tolerance to finish

    // ── Deceleration / creep ───────────────────────────────────────────
    Length curvatureSuppressDist = 6_in; ///< Fade curvature near goal
    Length finalCreepDist      = 8_in;   ///< Max-speed zone near end
    LinearVelocity finalCreepSpeed = 14_inps;

    // ── Misc ───────────────────────────────────────────────────────────
    LinearVelocity manualBaseSpeed = 12_inps; ///< Speed when no trajectory
    bool   stoppingDistanceDecel  = true;     ///< Physics-based decel clamp
};

class PurePursuitController {
public:
    PurePursuitController(
        lemlib::MotorGroup& leftMotors,
        lemlib::MotorGroup& rightMotors,
        const control::DifferentialDriveConfig& driveConfig,
        std::function<units::Pose()> poseProvider,
        const PurePursuitConfig& config = PurePursuitConfig{}
    );

    /// Stop any running task and clean up
    ~PurePursuitController();

    // ── Path / trajectory ──────────────────────────────────────────────
    void setPath(const std::vector<units::Pose>& path);
    void setTrajectory(const motion::Trajectory& traj);

    // ── Configuration ──────────────────────────────────────────────────
    /// Replace all tuning constants at once.
    void setConfig(const PurePursuitConfig& config);
    /// Read-only access to current config.
    const PurePursuitConfig& getConfig() const { return m_cfg; }

    // Convenience setters for the most common tweaks
    void setLookahead(Length L) { m_cfg.lookahead = L; m_lookahead = L; }
    void setWaypointTolerance(Length tol) { m_cfg.waypointTolerance = tol; }
    void setReversed(bool r) { m_reversed = r; }
    bool isReversed() const { return m_reversed; }
    void setRequireFinalHeading(bool require) { m_requireFinalHeading = require; }
    /// Set Stanley gains (velocity-softening mode, for back-compat).
    void setStanleyGains(Number gain, LinearVelocity softeningVel, Number omegaGain = Number(1.0));

    // ── Execution ──────────────────────────────────────────────────────
    bool followPath(bool async = false, bool reversedDrive = false);
    bool isFollowing() const { return m_isFollowing; }
    void stop();

    // Expose update so background task can call it
    bool update(Time dt);
    bool m_taskRunning = false;  // true while the RTOS task is alive

private:
    lemlib::MotorGroup& m_leftMotors;
    lemlib::MotorGroup& m_rightMotors;
    control::DifferentialDriveConfig m_driveConfig;
    std::function<units::Pose()> m_poseProvider;

    PurePursuitConfig m_cfg; ///< All tuning constants

    // ── Path state ─────────────────────────────────────────────────────
    std::vector<units::Pose> m_path;
    std::vector<motion::TrajectoryState> m_trajStates;
    size_t m_trajTimeIndex = 0;
    std::vector<Length> m_cumulativeLengths;
    Length m_totalPathLength = 0_in;
    Length m_lookahead;  // runtime lookahead (may differ from m_cfg during dynamic adjustment)

    // ── Per-run state ──────────────────────────────────────────────────
    bool m_isFollowing = false;
    int m_lastClosestIndex = 0;
    bool m_reversed = false;
    bool m_requireFinalHeading = false;
    Time m_runTime = 0_sec;
    Length m_lastDistEnd = 0_in;
    int m_distIncreasingCount = 0;
    units::Pose m_lastPoseForSpeed;
    LinearAcceleration m_maxTrajAccel = 0_inps2;

    // Pivot runtime state
    int    m_finalPivotStableCount = 0;
    bool   m_inFinalPivot = false;
    Angle  m_prevPivotHeadingError = 0_stDeg;
    Angle  m_prevEndAssistHeadingError = 0_stDeg;

    // Debug / diagnostics (not tuning)
    double m_lastStanleyGainEffective = 0.0;
    double m_lastHeadingPriorityScale = 1.0;

    // For setStanleyGains() back-compat: when called, overrides percent mode
    bool m_stanleyUseVelMode = false;
    LinearVelocity m_stanleySofteningVel = 1_inps;

    pros::Task* m_task = nullptr;

    // ── Internal helpers ───────────────────────────────────────────────
    int findClosestSegmentIndex(const units::Pose& current);
    bool computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget);
    Length computePathProgress(const units::Pose& current, int segIndex, Length& alongDistToEnd);
};

} // namespace motion
