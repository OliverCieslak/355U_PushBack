#pragma once

#include "control/PIDDriveController.hpp"
#include "units/units.hpp"
#include <string>
#include <vector>

namespace tuning {

/**
 * @brief Result of an autotuning session
 */
struct AutoTuneResult {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double criticalGain = 0;      ///< Ku – gain at sustained oscillation
    double oscillationPeriod = 0;  ///< Tu – period of oscillation (seconds)
    bool   success = false;
    std::string message;
};

/**
 * @brief Safety / behaviour configuration for the autotuner
 *
 * kpStart defaults to 0 which means "use the current hardcoded value".
 */
struct AutoTuneConfig {
    // --- voltage & motion limits ---
    Number maxTestVoltage    = 6.0;       ///< Maximum voltage during tuning (prevents runaway)
    Length maxLinearOvershoot = 12_in;     ///< Abort if linear overshoot exceeds this
    Angle  maxAngularOvershoot = 45_stDeg; ///< Abort if angular overshoot exceeds this

    // --- kP ramp parameters ---
    double kpStart   = 0.0;   ///< Starting kP (0 = use current hardcoded value)
    double kpStep    = 0.02;  ///< Increment per iteration
    double kpMax     = 5.0;   ///< Maximum kP to try before giving up

    // --- oscillation detection ---
    int    requiredCycles  = 3;       ///< Number of zero-crossings to confirm oscillation
    Time   perTrialTimeout = 4_sec;   ///< Timeout for each individual kP trial
    Time   settleDelay     = 750_msec; ///< Pause between trials for the robot to stop

    // --- Ziegler-Nichols multipliers (conservative) ---
    double znKpFactor = 0.45;  ///< Fraction of Ku for kP (classic Z-N = 0.6)
    double znKiFactor = 1.2;   ///< kI = znKiFactor * kP / Tu
    double znKdFactor = 0.075; ///< kD = znKdFactor * kP * Tu

    // --- Linear test parameters ---
    Length linearTestDistance = 24_in;

    // --- Angular test parameters ---
    Angle  angularTestAngle = 90_stDeg;

    // --- Heading correction sweep parameters ---
    double headingKpMin   = 0.02;  ///< Smallest heading kP to try
    double headingKpMax   = 1.0;   ///< Largest heading kP to try
    int    headingKpSteps = 10;    ///< Number of evenly-spaced values to test
    Length headingTestDistance = 48_in; ///< Drive distance per trial (longer = more signal)
    Number headingBaseVoltage  = 4.0;  ///< Constant forward voltage during heading trials
    Angle  headingAbortThresh = 30_stDeg; ///< Abort trial if heading deviates this much
};

/**
 * @brief Ziegler-Nichols relay-method PID autotuner for the drive controller
 *
 * Instead of exhaustively searching a grid of parameters (which causes many
 * uncontrolled movements), this tuner:
 *   1. Starts with kI = kD = 0 and the current hardcoded kP.
 *   2. Ramps kP upward from there until the system begins to oscillate
 *      with a consistent period.  The voltage is always clamped to a safe maximum.
 *   3. Records the critical gain (Ku) and oscillation period (Tu).
 *   4. Uses the classic Ziegler-Nichols formulas (with conservative multipliers)
 *      to compute kP, kI, kD.
 *   5. Runs a single validation move with the computed gains.
 *
 * All telemetry is printed to stdout (the PROS terminal).
 *
 * Two separate routines are provided: one for linear (drive straight) and one
 * for angular (turn in place).
 */
class PIDAutoTuner {
public:
    /**
     * @brief Construct a PIDAutoTuner
     *
     * @param controller  The PIDDriveController whose gains will be tuned
     */
    explicit PIDAutoTuner(control::PIDDriveController& controller);

    /**
     * @brief Autotune linear (drive-straight) PID gains
     *
     * The robot will make small forward moves with increasing kP until
     * oscillation is detected, then compute and validate gains.
     *
     * @param cfg  Tuning configuration / safety limits
     * @return AutoTuneResult with the computed gains
     */
    AutoTuneResult autotuneLinear(const AutoTuneConfig& cfg = AutoTuneConfig());

    /**
     * @brief Autotune angular (turn-in-place) PID gains
     *
     * The robot will make small turns with increasing kP until oscillation
     * is detected, then compute and validate gains.
     *
     * @param cfg  Tuning configuration / safety limits
     * @return AutoTuneResult with the computed gains
     */
    AutoTuneResult autotuneAngular(const AutoTuneConfig& cfg = AutoTuneConfig());

    /**
     * @brief Autotune heading correction kP by sweep
     *
     * Drives the robot forward at a fixed base voltage multiple times, each
     * with a different heading-correction kP (kI=kD=0).  Measures RMS heading
     * error over each trial and picks the kP with the lowest score.  Only kP
     * is tuned — heading correction doesn't benefit from kI/kD in practice.
     *
     * @param cfg  Tuning configuration / safety limits
     * @return AutoTuneResult with the best kP (kI=kD=0)
     */
    AutoTuneResult autotuneHeadingCorrection(const AutoTuneConfig& cfg = AutoTuneConfig());

    /**
     * @brief Run linear, angular, and heading-correction autotuning sequentially
     *
     * @param cfg  Tuning configuration / safety limits
     */
    void autotuneBoth(const AutoTuneConfig& cfg = AutoTuneConfig());

    /**
     * @brief Save the tuned gains to the SD card
     * @return true on success
     */
    bool saveResults();

private:
    control::PIDDriveController& m_controller;

    AutoTuneResult m_linearResult;
    AutoTuneResult m_angularResult;
    AutoTuneResult m_headingResult;

    /**
     * @brief Core oscillation-detection routine for linear motion.
     *
     * Drives forward `distance` with the given kP (kI=kD=0).  Monitors the
     * error signal for zero-crossings.  Returns the period of oscillation if
     * enough crossings are detected, or 0 if the motion settles/times out.
     */
    double probeLinearOscillation(
        double kp, Length distance, Number maxVoltage,
        Time timeout, int requiredCycles,
        Length maxOvershoot, bool& aborted);

    /**
     * @brief Core oscillation-detection routine for angular motion.
     */
    double probeAngularOscillation(
        double kp, Angle angle, Number maxVoltage,
        Time timeout, int requiredCycles,
        Angle maxOvershoot, bool& aborted);

    /**
     * @brief Drive forward and score heading-correction quality.
     *
     * Drives at a constant base voltage for the given distance while the
     * heading-correction PID (kP only) tries to keep heading at 0.
     * Returns the RMS heading error in degrees (lower is better).
     * Returns -1 if aborted (heading exceeded threshold).
     */
    double scoreHeadingTrial(
        double kp, Length distance, Number baseVoltage,
        Time timeout, Angle abortThreshold);

    /**
     * @brief Apply Ziegler-Nichols formulas to compute PID gains
     */
    static AutoTuneResult computeZN(
        double Ku, double Tu, const AutoTuneConfig& cfg);
};

// ----- Free functions for the auton selector -----

/**
 * @brief Auton routine: autotune linear PID, then angular PID, save results
 */
void autonAutoTunePID();

} // namespace tuning
