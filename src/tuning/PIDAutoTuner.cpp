#include "tuning/PIDAutoTuner.hpp"
#include "odometry/SkidSteerOdometry.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <vector>

// Externs we need from main.cpp
extern control::PIDDriveController pidDriveController;
extern odometry::SkidSteerOdometry odometrySystem;
extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern double linearKp;
extern double linearKi;
extern double linearKd;
extern double angularKp;
extern double angularKi;
extern double angularKd;
extern double headingKp;
extern double headingKi;
extern double headingKd;

namespace tuning {

// -----------------------------------------------------------------------
//  Constructor
// -----------------------------------------------------------------------

PIDAutoTuner::PIDAutoTuner(control::PIDDriveController& controller)
    : m_controller(controller) {}

// -----------------------------------------------------------------------
//  Ziegler-Nichols computation
// -----------------------------------------------------------------------

AutoTuneResult PIDAutoTuner::computeZN(
    double Ku, double Tu, const AutoTuneConfig& cfg)
{
    AutoTuneResult r;
    r.criticalGain      = Ku;
    r.oscillationPeriod  = Tu;
    r.kP = cfg.znKpFactor * Ku;
    r.kI = cfg.znKiFactor * r.kP / Tu;
    r.kD = cfg.znKdFactor * r.kP * Tu;
    r.success = true;
    r.message = "Ziegler-Nichols tuning successful";
    return r;
}

// -----------------------------------------------------------------------
//  Probe for oscillation – LINEAR
// -----------------------------------------------------------------------

double PIDAutoTuner::probeLinearOscillation(
    double kp, Length distance, Number maxVoltage,
    Time timeout, int requiredCycles,
    Length maxOvershoot, bool& aborted)
{
    aborted = false;

    // Set gains: kP only, no kI or kD
    m_controller.setLinearGains(kp, 0.0, 0.0);

    // Reset odometry to origin facing forward
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    pros::delay(20); // let the reset propagate

    double targetIn = to_in(distance);

    // Tracking variables
    double prevError      = targetIn; // start far from target
    int    zeroCrossings  = 0;
    std::vector<uint32_t> crossingTimesMs;
    uint32_t startMs = pros::millis();

    // Manual PID loop at 10 ms
    control::PID pid(kp, 0.0, 0.0);
    pid.setOutputLimits(-maxVoltage, maxVoltage);

    uint32_t now = pros::millis();
    while (true) {
        uint32_t elapsed = pros::millis() - startMs;
        if (from_msec(elapsed) >= timeout) break;

        // Current position from odometry (we drive along +x at heading 0)
        units::Pose pose = odometrySystem.getPose();
        double currentIn = to_in(pose.x);
        double error     = targetIn - currentIn;

        // Safety: abort if overshoot exceeds limit
        if (std::abs(error) > to_in(maxOvershoot) + targetIn) {
            std::cout << "  [ABORT] Overshoot limit hit at kP=" << kp
                      << "  error=" << error << " in" << std::endl;
            aborted = true;
            leftMotors.move(0);
            rightMotors.move(0);
            return 0.0;
        }

        // Detect zero-crossing of error (sign change)
        if (prevError != 0.0 && error != 0.0) {
            bool signChanged = (prevError > 0.0 && error < 0.0) ||
                               (prevError < 0.0 && error > 0.0);
            if (signChanged) {
                zeroCrossings++;
                crossingTimesMs.push_back(pros::millis());
            }
        }
        prevError = error;

        // Enough crossings to measure period?
        if (zeroCrossings >= requiredCycles) {
            leftMotors.move(0);
            rightMotors.move(0);
            // Average half-period from successive crossings, period = 2 * half-period
            double totalHalfPeriodMs = 0;
            for (size_t i = 1; i < crossingTimesMs.size(); i++) {
                totalHalfPeriodMs += (crossingTimesMs[i] - crossingTimesMs[i - 1]);
            }
            double avgHalfPeriodMs = totalHalfPeriodMs / (crossingTimesMs.size() - 1);
            double periodSec = avgHalfPeriodMs * 2.0 / 1000.0;
            return periodSec;
        }

        // PID output (setpoint = targetIn, measurement = currentIn)
        double output = pid.calculate(currentIn, targetIn, (double)elapsed);
        output = std::max(-maxVoltage.internal(), std::min(maxVoltage.internal(), output));

        // Apply equal voltage to both sides (drive straight)
        leftMotors.move(output / 12.0);
        rightMotors.move(output / 12.0);

        pros::Task::delay_until(&now, 10);
    }

    // Timed out without enough crossings – no oscillation at this kP
    leftMotors.move(0);
    rightMotors.move(0);
    return 0.0;
}

// -----------------------------------------------------------------------
//  Probe for oscillation – ANGULAR
// -----------------------------------------------------------------------

double PIDAutoTuner::probeAngularOscillation(
    double kp, Angle angle, Number maxVoltage,
    Time timeout, int requiredCycles,
    Angle maxOvershoot, bool& aborted)
{
    aborted = false;

    // Set gains: kP only
    m_controller.setAngularGains(kp, 0.0, 0.0);

    // Reset odometry to origin
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    pros::delay(20);

    double targetDeg = to_stDeg(angle);

    double prevError     = targetDeg;
    int    zeroCrossings = 0;
    std::vector<uint32_t> crossingTimesMs;
    uint32_t startMs = pros::millis();

    control::PID pid(kp, 0.0, 0.0);
    pid.setOutputLimits(-maxVoltage, maxVoltage);

    uint32_t now = pros::millis();
    while (true) {
        uint32_t elapsed = pros::millis() - startMs;
        if (from_msec(elapsed) >= timeout) break;

        units::Pose pose = odometrySystem.getPose();
        double currentDeg = to_stDeg(pose.orientation);
        // Constrain error to [-180, 180]
        double error = targetDeg - currentDeg;
        while (error >  180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;

        // Safety
        if (std::abs(error) > to_stDeg(maxOvershoot) + std::abs(targetDeg)) {
            std::cout << "  [ABORT] Angular overshoot limit at kP=" << kp
                      << "  error=" << error << " deg" << std::endl;
            aborted = true;
            leftMotors.move(0);
            rightMotors.move(0);
            return 0.0;
        }

        // Zero-crossing detection
        if (prevError != 0.0 && error != 0.0) {
            bool signChanged = (prevError > 0.0 && error < 0.0) ||
                               (prevError < 0.0 && error > 0.0);
            if (signChanged) {
                zeroCrossings++;
                crossingTimesMs.push_back(pros::millis());
            }
        }
        prevError = error;

        if (zeroCrossings >= requiredCycles) {
            leftMotors.move(0);
            rightMotors.move(0);
            double totalHalfPeriodMs = 0;
            for (size_t i = 1; i < crossingTimesMs.size(); i++) {
                totalHalfPeriodMs += (crossingTimesMs[i] - crossingTimesMs[i - 1]);
            }
            double avgHalfPeriodMs = totalHalfPeriodMs / (crossingTimesMs.size() - 1);
            double periodSec = avgHalfPeriodMs * 2.0 / 1000.0;
            return periodSec;
        }

        // Differential drive: turn in place
        double output = pid.calculate(currentDeg, targetDeg, (double)elapsed);
        output = std::max(-maxVoltage.internal(), std::min(maxVoltage.internal(), output));
        leftMotors.move(-output / 12.0);
        rightMotors.move( output / 12.0);

        pros::Task::delay_until(&now, 10);
    }

    leftMotors.move(0);
    rightMotors.move(0);
    return 0.0;
}

// -----------------------------------------------------------------------
//  autotuneLinear
// -----------------------------------------------------------------------

AutoTuneResult PIDAutoTuner::autotuneLinear(const AutoTuneConfig& cfg) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  PID AUTOTUNE – LINEAR" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Distance : " << to_in(cfg.linearTestDistance) << " in" << std::endl;
    std::cout << "Max Volt : " << cfg.maxTestVoltage << std::endl;
    std::cout << "Max Overshoot : " << to_in(cfg.maxLinearOvershoot) << " in" << std::endl;

    // Start from the current hardcoded kP (or cfg override)
    double startKp = (cfg.kpStart > 0.0) ? cfg.kpStart : linearKp;
    // If the hardcoded kP is already very small, start from a small value
    if (startKp < 0.01) startKp = 0.02;

    std::cout << "Starting kP: " << startKp << "  step: " << cfg.kpStep
              << "  max: " << cfg.kpMax << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double Ku = 0.0;
    double Tu = 0.0;

    for (double kp = startKp; kp <= cfg.kpMax; kp += cfg.kpStep) {
        std::cout << "Testing kP = " << std::fixed << std::setprecision(4) << kp << " ... " << std::flush;

        bool aborted = false;
        double period = probeLinearOscillation(
            kp, cfg.linearTestDistance, cfg.maxTestVoltage,
            cfg.perTrialTimeout, cfg.requiredCycles,
            cfg.maxLinearOvershoot, aborted);

        if (aborted) {
            // The previous kP that didn't abort is our critical gain
            // (or this kP if it oscillated before aborting)
            if (Ku > 0.0 && Tu > 0.0) {
                std::cout << "Using previous oscillating kP as Ku." << std::endl;
                break;
            }
            std::cout << "Aborted, reducing step." << std::endl;
            // Back off and try smaller steps
            kp -= cfg.kpStep;
            // Try half-steps from here
            double halfStep = cfg.kpStep / 2.0;
            for (double kp2 = kp + halfStep; kp2 <= kp + cfg.kpStep; kp2 += halfStep) {
                std::cout << "  Retry kP = " << std::fixed << std::setprecision(4) << kp2 << " ... " << std::flush;
                bool ab2 = false;
                double p2 = probeLinearOscillation(
                    kp2, cfg.linearTestDistance, cfg.maxTestVoltage,
                    cfg.perTrialTimeout, cfg.requiredCycles,
                    cfg.maxLinearOvershoot, ab2);
                if (ab2) {
                    std::cout << "Aborted again." << std::endl;
                    break;
                }
                if (p2 > 0.0) {
                    Ku = kp2;
                    Tu = p2;
                    std::cout << "Oscillation! Tu=" << Tu << "s" << std::endl;
                    break;
                }
                std::cout << "No oscillation." << std::endl;
                // Let robot settle
                pros::delay(to_msec(cfg.settleDelay));
            }
            break;
        }

        if (period > 0.0) {
            Ku = kp;
            Tu = period;
            std::cout << "Oscillation detected! Tu=" << std::fixed << std::setprecision(4) << Tu << "s" << std::endl;
            break;
        }

        std::cout << "No oscillation (settled)." << std::endl;

        // Let robot come to rest, then drive back to start
        pros::delay(to_msec(cfg.settleDelay));
        // Drive back by resetting odom and driving negative
        odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
        // Use a gentle return with the current kP
        m_controller.setLinearGains(kp, 0.0, 0.1); // add a tiny kD for return trip
        m_controller.driveDistance(-cfg.linearTestDistance, cfg.maxTestVoltage, 5_sec, true);
        m_controller.stop();
        pros::delay(to_msec(cfg.settleDelay));
    }

    // Compute gains
    if (Ku <= 0.0 || Tu <= 0.0) {
        m_linearResult.success = false;
        m_linearResult.message = "Could not find critical gain (no oscillation detected)";
        std::cout << "\n[FAIL] " << m_linearResult.message << std::endl;
        // Restore original gains
        m_controller.setLinearGains(linearKp, linearKi, linearKd);
        return m_linearResult;
    }

    m_linearResult = computeZN(Ku, Tu, cfg);
    std::cout << "\n--- Linear Autotune Results ---" << std::endl;
    std::cout << "Critical Gain (Ku) : " << Ku << std::endl;
    std::cout << "Osc. Period  (Tu)  : " << Tu << " s" << std::endl;
    std::cout << "Computed kP        : " << m_linearResult.kP << std::endl;
    std::cout << "Computed kI        : " << m_linearResult.kI << std::endl;
    std::cout << "Computed kD        : " << m_linearResult.kD << std::endl;

    // Apply and validate with one test drive
    m_controller.setLinearGains(m_linearResult.kP, m_linearResult.kI, m_linearResult.kD);
    std::cout << "\nValidation drive..." << std::endl;
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    pros::delay(20);
    bool success = m_controller.driveDistance(cfg.linearTestDistance, cfg.maxTestVoltage, 5_sec, true);
    units::Pose finalPose = odometrySystem.getPose();
    double finalError = std::abs(to_in(cfg.linearTestDistance) - to_in(finalPose.x));
    std::cout << "Validation " << (success ? "PASSED" : "TIMED OUT")
              << "  final error: " << std::fixed << std::setprecision(2) << finalError << " in" << std::endl;

    // Drive back
    m_controller.driveDistance(-cfg.linearTestDistance, cfg.maxTestVoltage, 5_sec, true);
    m_controller.stop();

    return m_linearResult;
}

// -----------------------------------------------------------------------
//  autotuneAngular
// -----------------------------------------------------------------------

AutoTuneResult PIDAutoTuner::autotuneAngular(const AutoTuneConfig& cfg) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  PID AUTOTUNE – ANGULAR" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Angle    : " << to_stDeg(cfg.angularTestAngle) << " deg" << std::endl;
    std::cout << "Max Volt : " << cfg.maxTestVoltage << std::endl;

    double startKp = (cfg.kpStart > 0.0) ? cfg.kpStart : angularKp;
    if (startKp < 0.01) startKp = 0.02;

    std::cout << "Starting kP: " << startKp << "  step: " << cfg.kpStep
              << "  max: " << cfg.kpMax << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double Ku = 0.0;
    double Tu = 0.0;

    for (double kp = startKp; kp <= cfg.kpMax; kp += cfg.kpStep) {
        std::cout << "Testing kP = " << std::fixed << std::setprecision(4) << kp << " ... " << std::flush;

        bool aborted = false;
        double period = probeAngularOscillation(
            kp, cfg.angularTestAngle, cfg.maxTestVoltage,
            cfg.perTrialTimeout, cfg.requiredCycles,
            cfg.maxAngularOvershoot, aborted);

        if (aborted) {
            if (Ku > 0.0 && Tu > 0.0) {
                std::cout << "Using previous oscillating kP as Ku." << std::endl;
                break;
            }
            std::cout << "Aborted, reducing step." << std::endl;
            kp -= cfg.kpStep;
            double halfStep = cfg.kpStep / 2.0;
            for (double kp2 = kp + halfStep; kp2 <= kp + cfg.kpStep; kp2 += halfStep) {
                std::cout << "  Retry kP = " << std::fixed << std::setprecision(4) << kp2 << " ... " << std::flush;
                bool ab2 = false;
                double p2 = probeAngularOscillation(
                    kp2, cfg.angularTestAngle, cfg.maxTestVoltage,
                    cfg.perTrialTimeout, cfg.requiredCycles,
                    cfg.maxAngularOvershoot, ab2);
                if (ab2) {
                    std::cout << "Aborted again." << std::endl;
                    break;
                }
                if (p2 > 0.0) {
                    Ku = kp2;
                    Tu = p2;
                    std::cout << "Oscillation! Tu=" << Tu << "s" << std::endl;
                    break;
                }
                std::cout << "No oscillation." << std::endl;
                pros::delay(to_msec(cfg.settleDelay));
            }
            break;
        }

        if (period > 0.0) {
            Ku = kp;
            Tu = period;
            std::cout << "Oscillation detected! Tu=" << std::fixed << std::setprecision(4) << Tu << "s" << std::endl;
            break;
        }

        std::cout << "No oscillation (settled)." << std::endl;
        pros::delay(to_msec(cfg.settleDelay));
        // Turn back to origin
        odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
        // No need to turn back for angular – the next trial resets odom anyway
    }

    if (Ku <= 0.0 || Tu <= 0.0) {
        m_angularResult.success = false;
        m_angularResult.message = "Could not find critical gain (no oscillation detected)";
        std::cout << "\n[FAIL] " << m_angularResult.message << std::endl;
        m_controller.setAngularGains(angularKp, angularKi, angularKd);
        return m_angularResult;
    }

    m_angularResult = computeZN(Ku, Tu, cfg);
    std::cout << "\n--- Angular Autotune Results ---" << std::endl;
    std::cout << "Critical Gain (Ku) : " << Ku << std::endl;
    std::cout << "Osc. Period  (Tu)  : " << Tu << " s" << std::endl;
    std::cout << "Computed kP        : " << m_angularResult.kP << std::endl;
    std::cout << "Computed kI        : " << m_angularResult.kI << std::endl;
    std::cout << "Computed kD        : " << m_angularResult.kD << std::endl;

    // Validate
    m_controller.setAngularGains(m_angularResult.kP, m_angularResult.kI, m_angularResult.kD);
    std::cout << "\nValidation turn..." << std::endl;
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    pros::delay(20);
    bool success = m_controller.turnAngle(cfg.angularTestAngle, cfg.maxTestVoltage, 5_sec, true);
    units::Pose finalPose = odometrySystem.getPose();
    double finalError = std::abs(to_stDeg(cfg.angularTestAngle) - to_stDeg(finalPose.orientation));
    std::cout << "Validation " << (success ? "PASSED" : "TIMED OUT")
              << "  final error: " << std::fixed << std::setprecision(2) << finalError << " deg" << std::endl;

    // Turn back
    m_controller.turnAngle(-cfg.angularTestAngle, cfg.maxTestVoltage, 5_sec, true);
    m_controller.stop();

    return m_angularResult;
}

// -----------------------------------------------------------------------
//  scoreHeadingTrial – drive forward and measure heading quality
// -----------------------------------------------------------------------

double PIDAutoTuner::scoreHeadingTrial(
    double kp, Length distance, Number baseVoltage,
    Time timeout, Angle abortThreshold)
{
    // Reset pose – heading starts at 0
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    pros::delay(20);

    double targetIn = to_in(distance);
    double abortDeg = to_stDeg(abortThreshold);

    control::PID headingPid(kp, 0.0, 0.0);
    headingPid.setOutputLimits(-baseVoltage * 0.5, baseVoltage * 0.5);

    double sumSqError = 0.0;
    int    samples    = 0;
    uint32_t startMs  = pros::millis();
    uint32_t now      = pros::millis();

    while (true) {
        uint32_t elapsed = pros::millis() - startMs;
        if (from_msec(elapsed) >= timeout) break;

        units::Pose pose = odometrySystem.getPose();
        double drivenIn  = to_in(pose.x);

        // Reached the target distance — done
        if (drivenIn >= targetIn) break;

        double headingDeg = to_stDeg(pose.orientation);

        // Safety: abort if heading goes way off
        if (std::abs(headingDeg) > abortDeg) {
            leftMotors.move(0);
            rightMotors.move(0);
            return -1.0; // sentinel: trial aborted
        }

        // Accumulate RMS heading error
        sumSqError += headingDeg * headingDeg;
        samples++;

        // Heading correction output (target heading = 0)
        double correction = headingPid.calculate(headingDeg, 0.0, (double)elapsed);

        // Constant forward voltage ± heading correction
        double left  = baseVoltage.internal() - correction;
        double right = baseVoltage.internal() + correction;
        leftMotors.move(left / 12.0);
        rightMotors.move(right / 12.0);

        pros::Task::delay_until(&now, 10);
    }

    leftMotors.move(0);
    rightMotors.move(0);

    if (samples == 0) return -1.0;
    return std::sqrt(sumSqError / samples); // RMS heading error (degrees)
}

// -----------------------------------------------------------------------
//  autotuneHeadingCorrection – sweep kP, pick lowest RMS heading error
// -----------------------------------------------------------------------

AutoTuneResult PIDAutoTuner::autotuneHeadingCorrection(const AutoTuneConfig& cfg) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  PID AUTOTUNE – HEADING CORRECTION" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Drive distance : " << to_in(cfg.headingTestDistance) << " in" << std::endl;
    std::cout << "Base voltage   : " << cfg.headingBaseVoltage << std::endl;
    std::cout << "kP range       : [" << cfg.headingKpMin << ", " << cfg.headingKpMax
              << "]  steps=" << cfg.headingKpSteps << std::endl;
    std::cout << "Abort threshold: " << to_stDeg(cfg.headingAbortThresh) << " deg" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double bestKp    = headingKp; // fall back to current value
    double bestScore = 1e9;
    bool   anyValid  = false;

    double step = (cfg.headingKpSteps > 1)
        ? (cfg.headingKpMax - cfg.headingKpMin) / (cfg.headingKpSteps - 1)
        : 0.0;

    for (int i = 0; i < cfg.headingKpSteps; i++) {
        double kp = cfg.headingKpMin + step * i;

        std::cout << "Trial " << (i + 1) << "/" << cfg.headingKpSteps
                  << "  kP=" << std::fixed << std::setprecision(4) << kp << " ... " << std::flush;

        double rms = scoreHeadingTrial(
            kp, cfg.headingTestDistance, cfg.headingBaseVoltage,
            cfg.perTrialTimeout, cfg.headingAbortThresh);

        if (rms < 0.0) {
            std::cout << "ABORTED (heading exceeded threshold)" << std::endl;
        } else {
            std::cout << "RMS heading error = " << std::fixed << std::setprecision(3)
                      << rms << " deg" << std::endl;
            anyValid = true;
            if (rms < bestScore) {
                bestScore = rms;
                bestKp    = kp;
            }
        }

        // Pause and drive back
        pros::delay(to_msec(cfg.settleDelay));
        odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
        uint32_t revStart = pros::millis();
        uint32_t revNow   = pros::millis();
        while (pros::millis() - revStart < 2500) {
            units::Pose p = odometrySystem.getPose();
            if (to_in(p.x) <= -to_in(cfg.headingTestDistance) + 2.0) break;
            leftMotors.move(-cfg.headingBaseVoltage.internal() / 12.0);
            rightMotors.move(-cfg.headingBaseVoltage.internal() / 12.0);
            pros::Task::delay_until(&revNow, 10);
        }
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(to_msec(cfg.settleDelay));
    }

    if (!anyValid) {
        m_headingResult.success = false;
        m_headingResult.message = "All heading trials aborted";
        std::cout << "\n[FAIL] " << m_headingResult.message << std::endl;
        m_controller.setHeadingCorrectionGains(headingKp, headingKi, headingKd);
        return m_headingResult;
    }

    m_headingResult.kP      = bestKp;
    m_headingResult.kI      = 0.0;
    m_headingResult.kD      = 0.0;
    m_headingResult.success = true;
    m_headingResult.message = "Heading kP sweep successful";

    std::cout << "\n--- Heading Correction Results ---" << std::endl;
    std::cout << "Best kP          : " << bestKp << std::endl;
    std::cout << "Best RMS error   : " << std::fixed << std::setprecision(3) << bestScore << " deg" << std::endl;

    // Apply the winning gains
    m_controller.setHeadingCorrectionGains(bestKp, 0.0, 0.0);

    // Validation drive
    std::cout << "\nValidation drive..." << std::endl;
    double valScore = scoreHeadingTrial(
        bestKp, cfg.headingTestDistance, cfg.headingBaseVoltage,
        cfg.perTrialTimeout, cfg.headingAbortThresh);
    if (valScore < 0.0) {
        std::cout << "Validation ABORTED" << std::endl;
    } else {
        std::cout << "Validation RMS heading error: " << std::fixed << std::setprecision(3)
                  << valScore << " deg" << std::endl;
    }

    // Drive back
    uint32_t revStart = pros::millis();
    uint32_t revNow   = pros::millis();
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    while (pros::millis() - revStart < 2500) {
        units::Pose p = odometrySystem.getPose();
        if (to_in(p.x) <= -to_in(cfg.headingTestDistance) + 2.0) break;
        leftMotors.move(-cfg.headingBaseVoltage.internal() / 12.0);
        rightMotors.move(-cfg.headingBaseVoltage.internal() / 12.0);
        pros::Task::delay_until(&revNow, 10);
    }
    leftMotors.move(0);
    rightMotors.move(0);

    return m_headingResult;
}

// -----------------------------------------------------------------------
//  autotuneBoth
// -----------------------------------------------------------------------

void PIDAutoTuner::autotuneBoth(const AutoTuneConfig& cfg) {
    std::cout << "\n╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║    PID AUTOTUNER – FULL SEQUENCE     ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;
    std::cout << "Starting from hardcoded values:" << std::endl;
    std::cout << "  Linear  kP=" << linearKp << "  kI=" << linearKi << "  kD=" << linearKd << std::endl;
    std::cout << "  Angular kP=" << angularKp << "  kI=" << angularKi << "  kD=" << angularKd << std::endl;
    std::cout << "  Heading kP=" << headingKp << std::endl;

    autotuneLinear(cfg);

    std::cout << "\nPausing 2s before angular tuning..." << std::endl;
    pros::delay(2000);

    autotuneAngular(cfg);

    std::cout << "\nPausing 2s before heading correction tuning..." << std::endl;
    pros::delay(2000);

    autotuneHeadingCorrection(cfg);

    // Print summary
    std::cout << "\n╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║         AUTOTUNE SUMMARY             ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;

    if (m_linearResult.success) {
        std::cout << "Linear:  kP=" << m_linearResult.kP
                  << "  kI=" << m_linearResult.kI
                  << "  kD=" << m_linearResult.kD << std::endl;
    } else {
        std::cout << "Linear:  FAILED – " << m_linearResult.message << std::endl;
        std::cout << "  (original values restored)" << std::endl;
    }

    if (m_angularResult.success) {
        std::cout << "Angular: kP=" << m_angularResult.kP
                  << "  kI=" << m_angularResult.kI
                  << "  kD=" << m_angularResult.kD << std::endl;
    } else {
        std::cout << "Angular: FAILED – " << m_angularResult.message << std::endl;
        std::cout << "  (original values restored)" << std::endl;
    }

    if (m_headingResult.success) {
        std::cout << "Heading: kP=" << m_headingResult.kP << "  (P-only)" << std::endl;
    } else {
        std::cout << "Heading: FAILED – " << m_headingResult.message << std::endl;
        std::cout << "  (original values restored)" << std::endl;
    }

    // Save if anything succeeded
    if (m_linearResult.success || m_angularResult.success || m_headingResult.success) {
        saveResults();
    }
}

// -----------------------------------------------------------------------
//  saveResults
// -----------------------------------------------------------------------

bool PIDAutoTuner::saveResults() {
    if (!pros::usd::is_installed()) {
        std::cout << "[WARN] No SD card – cannot save autotune results." << std::endl;
        return false;
    }

    std::string path = "/usd/pid_autotune_results.txt";
    try {
        std::ofstream file(path);
        if (!file.is_open()) {
            std::cout << "[ERROR] Could not open " << path << " for writing." << std::endl;
            return false;
        }

        if (m_linearResult.success) {
            file << "linearKp=" << m_linearResult.kP << std::endl;
            file << "linearKi=" << m_linearResult.kI << std::endl;
            file << "linearKd=" << m_linearResult.kD << std::endl;
            file << "linearKu=" << m_linearResult.criticalGain << std::endl;
            file << "linearTu=" << m_linearResult.oscillationPeriod << std::endl;
        }
        if (m_angularResult.success) {
            file << "angularKp=" << m_angularResult.kP << std::endl;
            file << "angularKi=" << m_angularResult.kI << std::endl;
            file << "angularKd=" << m_angularResult.kD << std::endl;
            file << "angularKu=" << m_angularResult.criticalGain << std::endl;
            file << "angularTu=" << m_angularResult.oscillationPeriod << std::endl;
        }
        if (m_headingResult.success) {
            file << "headingKp=" << m_headingResult.kP << std::endl;
        }

        file.close();
        std::cout << "[OK] Results saved to " << path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Save failed: " << e.what() << std::endl;
        return false;
    }
}

// -----------------------------------------------------------------------
//  Free function for the auton selector
// -----------------------------------------------------------------------

void autonAutoTunePID() {
    // Start odometry
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();

    PIDAutoTuner tuner(pidDriveController);
    tuner.autotuneBoth();
}

} // namespace tuning
