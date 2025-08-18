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
        Length waypointTolerance = 1_in
    );

    void setPath(const std::vector<units::Pose>& path);
    void setTrajectory(const motion::Trajectory& traj); // supplies velocity/accel profiles
    void setLookahead(Length L) { m_lookahead = L; }
    // Enable curvature-based dynamic lookahead between minL and maxL mapping curvature range [curvLow, curvHigh]
    void enableDynamicLookahead(Length minL, Length maxL, double curvLow, double curvHigh) {
        m_dynLookMin = minL; m_dynLookMax = maxL; m_dynCurvLow = curvLow; m_dynCurvHigh = curvHigh; m_useDynamicLookahead = true; }
    void disableDynamicLookahead() { m_useDynamicLookahead = false; }
    void setWaypointTolerance(Length tol) { m_waypointTolerance = tol; }
    void setReversed(bool r) { m_reversed = r; }
    bool isReversed() const { return m_reversed; }
    // Stanley controller tuning traditional: softening specified as linear velocity (in/s)
    void setStanleyGains(Number gain, LinearVelocity softeningVel, Number omegaGain = Number(1.0)) {
        m_stanleyGain = gain; m_stanleySofteningVel = softeningVel; m_stanleyOmegaGain = omegaGain; m_useStanleyPercent = false;
    }
    // Alternate Stanley tuning: softening specified as a percentage (fraction) of current speed
    void setStanleyGainsPercent(Number gain, Number softeningPercent, Number omegaGain = Number(1.0)) {
        m_stanleyGain = gain; m_stanleySofteningPercent = softeningPercent; m_stanleyOmegaGain = omegaGain; m_useStanleyPercent = true;
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
    Length m_lookahead; // runtime initial lookahead (set by constructor)
    // Manual-path base speed (used only if no trajectory velocities provided)
    LinearVelocity m_manualBaseSpeed = 12_inps;
    Length m_waypointTolerance;

    bool m_isFollowing = false;
    int m_lastClosestIndex = 0;
    bool m_reversed = false; // drive backwards along path if true
    bool m_requireFinalHeading = false; // optional heading enforcement
    // Dynamic lookahead config
    bool m_useDynamicLookahead = true; // enabled by default (tuned)
    Length m_dynLookMin = 8_in;  // tuned dynamic lookahead min
    Length m_dynLookMax = 14_in; // tuned dynamic lookahead max
    double m_dynCurvLow = 1.0;   // tuned nearly straight curvature threshold
    double m_dynCurvHigh = 12.0; // tuned tight curvature threshold
    units::Pose m_lastPose; // for velocity estimation
    // Per-run state
    Time m_runTime = 0_sec;
    Length m_lastDistEnd = 0_in;
    int m_distIncreasingCount = 0;
    units::Pose m_lastPoseForSpeed; // previous pose for speed calc
    // Stanley parameters
    Number m_stanleyGain = Number(0.25); // default tuned gain
    // Stanley softening options
    LinearVelocity m_stanleySofteningVel = 1_inps; // (unused when percent enabled)
    Number m_stanleySofteningPercent = Number(0.30); // tuned percent softening
    bool m_useStanleyPercent = true; // enable percent mode by default
    Number m_stanleyOmegaGain = Number(1.0);
    // Optional damping of Stanley gain near end of path to reduce final heading overshoot/pivot
    bool m_useStanleyDamping = true;
    Length m_stanleyDampStartDistance = 10_in; // damping window
    double m_stanleyDampMinScale = 0.10;        // min effective gain scale
    double m_lastStanleyGainEffective = 0.0;   // for debug logging
    int m_logMode = 0; // default basic logging (tuned)
    // Curvature suppression near end (to avoid spin); configurable distance
    Length m_curvatureSuppressDistance = 6_in;
    // End-of-path Stanley quiet zone to prevent fishtail (scale Stanley term when nearly centered)
    Length m_stanleyQuietDistance = 6_in;
    double m_stanleyQuietYF = 1.2; // in
    double m_stanleyQuietScale = 0.12;

    // Heading priority (reduce linear velocity & lookahead when heading error persists near end)
    bool m_headingPriorityEnabled = true; // tuned: enable by default
    Length m_headingPriorityDistance = 16_in; // start heading priority window
    Angle  m_headingPriorityStartError = from_stDeg(5); // begin scaling when >5 deg
    Angle  m_headingPriorityMaxError = from_stDeg(45);  // max error for scaling range
    double m_headingPriorityMinScale = 0.15;            // min velocity/lookahead scale
    double m_lastHeadingPriorityScale = 1.0; // for logging

    // Final heading pivot (option 1) configuration
    bool   m_finalPivotEnabled = true;          // tuned: enable by default
    Length m_finalPivotTriggerDistance = 4_in;  // start pivot within 4"
    Angle  m_finalPivotTolerance = from_stDeg(3); // acceptable final heading error
    double m_finalPivotKp = 2.0;                // tuned P gain
    double m_finalPivotKd = 0.6;                // tuned D gain
    double m_finalPivotKi = 0.0;                // integral disabled
    double m_finalPivotMaxOmega = 1.7;          // tuned max rotational speed (rad/s)
    int    m_finalPivotStableCyclesRequired = 4; // tuned stability cycles
    int    m_finalPivotStableCount = 0;
    bool   m_inFinalPivot = false;
    Angle  m_prevPivotHeadingError = 0_stDeg;
    double m_finalPivotIntegral = 0.0; // accumulated error
    Length m_finalPivotPosRadius = 1.25_in; // tuned tighter positional radius for pivot

    // Gentle end-phase heading assist (pre-pivot) blending a small omega before position completion
    bool   m_endHeadingAssistEnabled = true;  // tuned assist enabled
    Length m_endHeadingAssistStartDist = 12_in;
    double m_endHeadingAssistKp = 1.0;
    double m_endHeadingAssistKd = 0.25;
    double m_endHeadingAssistMaxOmega = 1.4;
    Angle  m_prevEndAssistHeadingError = 0_stDeg;

    // Deceleration / finish tuning
    LinearAcceleration m_maxTrajAccel = 0_inps2; // populated from trajectory
    bool m_enableStoppingDistanceDecel = true; // retain adaptive decel
    Length m_finalCreepDistance = 8_in;        // tuned final creep distance
    LinearVelocity m_finalCreepMaxSpeed = 14_inps; // tuned creep max speed

    pros::Task* m_task = nullptr;

    int findClosestSegmentIndex(const units::Pose& current);
    bool computeLookaheadPoint(const units::Pose& current, units::Pose& outTarget);
    Length computePathProgress(const units::Pose& current, int segIndex, Length& alongDistToEnd);
public:
    void setRequireFinalHeading(bool require) { m_requireFinalHeading = require; }
    void setLogMode(int mode) { m_logMode = (mode<=0)?0:1; }
    // Enable linear ramp damping of Stanley gain as remaining distance goes from startDist -> 0.
    void enableStanleyEndDamping(Length startDist, double minScale) {
        m_useStanleyDamping = true; m_stanleyDampStartDistance = startDist; m_stanleyDampMinScale = std::clamp(minScale, 0.0, 1.0); }
    void disableStanleyEndDamping() { m_useStanleyDamping = false; }
    void setCurvatureSuppressionDistance(Length dist) { m_curvatureSuppressDistance = units::max(0_in, dist); }
    void setStanleyQuietZone(Length dist, double yFThresh, double scale) { m_stanleyQuietDistance = dist; m_stanleyQuietYF = std::max(0.0, yFThresh); m_stanleyQuietScale = std::clamp(scale, 0.0, 1.0); }
    // Enable heading priority behavior (option 2): inside 'distance', if |heading error| > startErr, scale linear speed & lookahead.
    void enableHeadingPriority(Length distance, Angle startErr, Angle maxErr, double minScale) {
        m_headingPriorityEnabled = true;
        m_headingPriorityDistance = distance;
        m_headingPriorityStartError = startErr;
        m_headingPriorityMaxError = (units::abs(maxErr) < units::abs(startErr)) ? startErr : maxErr; // ensure ordering
        m_headingPriorityMinScale = std::clamp(minScale, 0.0, 1.0);
    }
    void disableHeadingPriority() { m_headingPriorityEnabled = false; }
    double getLastHeadingPriorityScale() const { return m_lastHeadingPriorityScale; }
    void setFinalCreep(Length dist, LinearVelocity maxSpeed) { m_finalCreepDistance = dist; m_finalCreepMaxSpeed = maxSpeed; }
    void enableFinalPivot(Length triggerDist, Angle tolerance, double kP, double maxOmega, int stableCycles=6, double kD=0.0, Length posRadius=2_in, double kI=0.0) {
        m_finalPivotEnabled = true; m_finalPivotTriggerDistance = triggerDist; m_finalPivotTolerance = tolerance; m_finalPivotKp = kP; m_finalPivotMaxOmega = maxOmega; m_finalPivotStableCyclesRequired = std::max(1, stableCycles); m_finalPivotKd = kD; m_finalPivotPosRadius = posRadius; m_finalPivotKi = kI;
    }
    void disableFinalPivot() { m_finalPivotEnabled = false; }
    void setFinalPivotPosRadius(Length r) { m_finalPivotPosRadius = r; }
    // Enable a mild heading PD assist prior to pivot to reduce residual heading error.
    void enableEndHeadingAssist(Length startDist, double kP, double kD, double maxOmega) {
        m_endHeadingAssistEnabled = true; m_endHeadingAssistStartDist = startDist; m_endHeadingAssistKp = kP; m_endHeadingAssistKd = kD; m_endHeadingAssistMaxOmega = maxOmega; }
    void disableEndHeadingAssist() { m_endHeadingAssistEnabled = false; }
};

} // namespace motion
