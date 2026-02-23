#include "tuning/PIDTuningRoutines.hpp"
#include "control/PIDDriveController.hpp"
#include "odometry/OneWheelOdometry.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "units/units.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>

// ── Externs from main.cpp ──────────────────────────────────────────────
extern odometry::OneWheelOdometry odometrySystem;
extern control::PIDDriveController pidDriveController;
extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern lemlib::V5InertialSensor imu;

extern double linearKp;
extern double linearKi;
extern double linearKd;
extern double angularKp;
extern double angularKi;
extern double angularKd;
extern double headingKp;
extern double headingKi;
extern double headingKd;
extern Number kS;
extern Number kV;

// Helper: wrapped error in degrees (target - current), constrained to [-180, 180]
static double wrappedErrorDeg(Angle target, Angle current) {
    return to_stDeg(units::constrainAngle180(target - current));
}

// ═══════════════════════════════════════════════════════════════════════
//  Angular PID tuning – 90° turn tests (CW then CCW)
// ═══════════════════════════════════════════════════════════════════════
static void runOneTurn(double turnDeg, int turnNum) {
    Angle startHeading = odometrySystem.getPose().orientation;
    Angle turnAmount = from_stDeg(turnDeg);
    Angle targetHeading = startHeading + turnAmount;

    printf("--- Turn %d: %+.0f deg ---\n", turnNum, turnDeg);
    printf("  time_ms  heading  error  (all deg)\n");

    double peakOvershoot = 0.0;   // positive = past target in turning direction
    bool crossedTarget = false;
    uint32_t turnStart = pros::millis();
    uint32_t settledAt = 0;

    pidDriveController.turnAngle(turnAmount, 8.0, 4_sec, false);
    {
        uint32_t now = pros::millis();
        uint32_t deadline = now + 4000;
        int printCnt = 0;
        while (pros::millis() < deadline) {
            pidDriveController.update(10_msec);
            Angle heading = odometrySystem.getPose().orientation;
            double err = wrappedErrorDeg(targetHeading, heading);
            double headingDeg = to_stDeg(heading);

            if (printCnt++ % 5 == 0) {  // print every 50ms (every 5th loop)
                printf("  %5lu  %8.2f  %8.2f\n",
                       (unsigned long)(pros::millis() - turnStart), headingDeg, err);
            }

            // Track overshoot: when error sign flips vs initial sign, we crossed target
            double initialSign = (turnDeg > 0) ? 1.0 : -1.0;
            if (!crossedTarget && (err * initialSign < 0)) {
                crossedTarget = true;
            }
            if (crossedTarget) {
                double overshootNow = -err * initialSign;  // positive = past target
                if (overshootNow > peakOvershoot) peakOvershoot = overshootNow;
            }

            if (!pidDriveController.isMoving()) {
                if (settledAt == 0) settledAt = pros::millis();
                break;
            }
            pros::Task::delay_until(&now, 10);
        }
    }
    pidDriveController.stop();

    double finalErr = wrappedErrorDeg(targetHeading, odometrySystem.getPose().orientation);
    uint32_t settleTime = (settledAt > 0) ? (settledAt - turnStart) : (pros::millis() - turnStart);

    printf("\n  Turn %d Results:\n", turnNum);
    printf("    Final error: %+.1f deg  %s\n", finalErr,
           std::abs(finalErr) > 3.0 ? ">> ADD Ki or INCREASE Kp" : "(OK)");
    printf("    Overshoot:   %.1f deg  %s\n", peakOvershoot,
           peakOvershoot > 5.0 ? ">> REDUCE Kp or INCREASE Kd" : "(OK)");
    printf("    Settle:      %lu ms  %s\n", (unsigned long)settleTime,
           settleTime >= 4000 ? ">> TIMED OUT — Kp too low or kS deadzone" :
           settleTime > 2000 ? ">> SLOW — INCREASE Kp" : "(OK)");
    printf("    Settled:     %s\n", settledAt > 0 ? "YES" : "NO (timed out)");
}

void tunePID_Angular()
{
    printf("\n========================================\n");
    printf("  ANGULAR PID TUNING\n");
    printf("  Kp=%.4f  Ki=%.4f  Kd=%.1f  kS=%.4f (tapered)\n",
           angularKp, angularKi, angularKd, (double)kS);
    printf("========================================\n");
    printf("Robot will turn -90 (CW), pause, then +90 (CCW).\n\n");
    pros::delay(1000);

    // Reset to 0 compass = 90 stDeg. Turns of ±90 stay within [0, 180] stDeg — no wrap.
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    pros::delay(100);

    // Turn 1: -90 deg (CW) → goes from 90 stDeg toward 0 stDeg
    runOneTurn(-90.0, 1);
    pros::delay(1500);

    // Turn 2: +90 deg (CCW) → goes back toward 90 stDeg
    runOneTurn(+90.0, 2);

    printf("\n========================================\n");
    printf("  ANGULAR TUNING COMPLETE\n");
    printf("  Tuning tips:\n");
    printf("  - Overshoot > 5 deg: reduce Kp or increase Kd\n");
    printf("  - Undershoot / slow: increase Kp\n");
    printf("  - Steady error > 2 deg: add small Ki (try 0.001)\n");
    printf("  - Oscillation: increase Kd or reduce Kp\n");
    printf("========================================\n");
}

// ═══════════════════════════════════════════════════════════════════════
//  Linear PID tuning – 24" forward/backward
// ═══════════════════════════════════════════════════════════════════════
void tunePID_Linear()
{
    printf("\n========================================\n");
    printf("  LINEAR PID TUNING\n");
    printf("  Kp=%.4f  Ki=%.4f  Kd=%.4f  kV=%.4f  kS=%.4f\n",
           linearKp, linearKi, linearKd, (double)kV, (double)kS);
    printf("  Heading correction: Kp=%.4f  Ki=%.4f  Kd=%.4f\n",
           headingKp, headingKi, headingKd);
    printf("========================================\n");
    printf("Robot will drive 24\" forward, pause, then 24\" back.\n");
    printf("Needs ~3 feet clear ahead.\n\n");
    pros::delay(1000);

    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    pros::delay(100);

    // ── Drive 1: +24 inches forward ──
    printf("--- Drive 1: +24 in forward ---\n");
    printf("  time_ms    y_in   error_in  heading\n");

    double targetDist = 24.0;
    double peakY = 0.0;
    uint32_t driveStart = pros::millis();
    uint32_t settledAt = 0;

    pidDriveController.driveDistance(24_in, 10.0, 5_sec, false);
    {
        uint32_t now = pros::millis();
        uint32_t deadline = now + 5000;
        int printCnt = 0;
        while (pros::millis() < deadline) {
            pidDriveController.update(10_msec);
            units::Pose p = odometrySystem.getPose();
            double y = to_in(p.y);
            double heading = to_stDeg(p.orientation);
            if (printCnt++ % 5 == 0) {
                printf("  %5lu  %8.2f  %8.2f  %8.1f\n",
                       (unsigned long)(pros::millis() - driveStart), y, targetDist - y, heading);
            }
            if (y > peakY) peakY = y;
            if (!pidDriveController.isMoving()) {
                if (settledAt == 0) settledAt = pros::millis();
                break;
            }
            pros::Task::delay_until(&now, 10);
        }
    }
    pidDriveController.stop();

    units::Pose fp1 = odometrySystem.getPose();
    double finalY1 = to_in(fp1.y);
    double overshoot1 = peakY - targetDist;
    uint32_t settleTime1 = (settledAt > 0) ? (settledAt - driveStart) : (pros::millis() - driveStart);

    printf("\n  Drive 1 Results:\n");
    printf("    Final Y:     %.2f in (target %.1f)\n", finalY1, targetDist);
    printf("    Error:       %+.2f in\n", targetDist - finalY1);
    printf("    Overshoot:   %.2f in  %s\n", overshoot1,
           overshoot1 > 1.0 ? ">> REDUCE Kp or INCREASE Kd" :
           overshoot1 < 0 ? "(didn't reach target)" : "(OK)");
    printf("    Settle:      %lu ms  %s\n", (unsigned long)settleTime1,
           settleTime1 >= 5000 ? ">> TIMED OUT" :
           settleTime1 > 3000 ? ">> SLOW — INCREASE Kp" : "(OK)");
    printf("    Lateral X:   %.2f in  %s\n", to_in(fp1.x),
           std::abs(to_in(fp1.x)) > 1.5 ? ">> INCREASE headingKp" : "(OK)");
    printf("    Heading:     %.1f deg\n", to_stDeg(fp1.orientation));

    pros::delay(2000);

    // ── Drive 2: -24 inches backward ──
    printf("\n--- Drive 2: -24 in backward ---\n");
    printf("  time_ms    y_in   error_in  heading\n");

    double startY2 = to_in(odometrySystem.getPose().y);
    double target2 = startY2 - 24.0;
    double peakY2 = startY2;
    driveStart = pros::millis();
    settledAt = 0;

    pidDriveController.driveDistance(-24_in, 10.0, 5_sec, false);
    {
        uint32_t now = pros::millis();
        uint32_t deadline = now + 5000;
        int printCnt = 0;
        while (pros::millis() < deadline) {
            pidDriveController.update(10_msec);
            units::Pose p = odometrySystem.getPose();
            double y = to_in(p.y);
            double heading = to_stDeg(p.orientation);
            if (printCnt++ % 5 == 0) {
                printf("  %5lu  %8.2f  %8.2f  %8.1f\n",
                       (unsigned long)(pros::millis() - driveStart), y, target2 - y, heading);
            }
            if (y < peakY2) peakY2 = y;
            if (!pidDriveController.isMoving()) {
                if (settledAt == 0) settledAt = pros::millis();
                break;
            }
            pros::Task::delay_until(&now, 10);
        }
    }
    pidDriveController.stop();

    units::Pose fp2 = odometrySystem.getPose();
    double finalY2 = to_in(fp2.y);
    double overshoot2 = target2 - peakY2;
    uint32_t settleTime2 = (settledAt > 0) ? (settledAt - driveStart) : (pros::millis() - driveStart);

    printf("\n  Drive 2 Results:\n");
    printf("    Final Y:     %.2f in (target %.1f)\n", finalY2, target2);
    printf("    Error:       %+.2f in\n", target2 - finalY2);
    printf("    Overshoot:   %.2f in  %s\n", overshoot2,
           overshoot2 > 1.0 ? ">> REDUCE Kp or INCREASE Kd" : "(OK)");
    printf("    Settle:      %lu ms\n", (unsigned long)settleTime2);

    printf("\n========================================\n");
    printf("  LINEAR TUNING COMPLETE\n");
    printf("  Total lateral X drift: %.2f in\n", to_in(fp2.x));
    printf("  Final heading: %.1f deg\n", to_stDeg(fp2.orientation));
    printf("========================================\n");
}

// ═══════════════════════════════════════════════════════════════════════
//  Heading correction tuning – long straight drive
// ═══════════════════════════════════════════════════════════════════════
void tunePID_Heading()
{
    printf("\n========================================\n");
    printf("  HEADING CORRECTION TUNING\n");
    printf("  headingKp=%.4f  headingKi=%.4f  headingKd=%.4f\n",
           headingKp, headingKi, headingKd);
    printf("========================================\n");
    printf("Robot will drive 48\" forward. Watch lateral drift.\n");
    printf("Needs ~5 feet of clear space ahead.\n\n");
    pros::delay(1000);

    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    pros::delay(100);

    printf("--- Drive: 48 in forward ---\n");
    printf("  time_ms    x_in     y_in  heading\n");

    double peakDrift = 0.0;
    uint32_t driveStart = pros::millis();

    pidDriveController.driveDistance(48_in, 10.0, 6_sec, false);
    {
        uint32_t now = pros::millis();
        uint32_t deadline = now + 6000;
        int printCnt = 0;
        while (pros::millis() < deadline) {
            pidDriveController.update(10_msec);
            units::Pose p = odometrySystem.getPose();
            double x = to_in(p.x);
            double y = to_in(p.y);
            double heading = to_stDeg(p.orientation);
            if (printCnt++ % 10 == 0) {  // print every 100ms
                printf("  %5lu  %8.2f  %8.2f  %8.1f\n",
                       (unsigned long)(pros::millis() - driveStart), x, y, heading);
            }
            if (std::abs(x) > std::abs(peakDrift)) peakDrift = x;
            if (!pidDriveController.isMoving()) break;
            pros::Task::delay_until(&now, 10);
        }
    }
    pidDriveController.stop();

    units::Pose fp = odometrySystem.getPose();
    printf("\n  Results:\n");
    printf("    Final:   x=%.2f  y=%.2f  heading=%.1f\n",
           to_in(fp.x), to_in(fp.y), to_stDeg(fp.orientation));
    printf("    Peak lateral drift: %.2f in  %s\n", peakDrift,
           std::abs(peakDrift) > 2.0 ? ">> INCREASE headingKp" :
           std::abs(peakDrift) < 0.3 ? "(excellent)" : "(OK)");
    printf("========================================\n");
}
