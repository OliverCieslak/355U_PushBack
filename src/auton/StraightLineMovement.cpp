#include "auton/CompetitionAutons.hpp"
#include "pros/adi.hpp"
#include "auton/IntakeAndPistonState.hpp" // Add this include for SnailState definition
#include "control/DifferentialDriveConfig.hpp"

// Externs needed for raw encoder access (defined in main.cpp)
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
extern lemlib::V5RotationSensor verticalTrackingWheel;
extern lemlib::V5RotationSensor horizontalTrackingWheel;
extern lemlib::V5InertialSensor imu;
extern Length trackingWheelDiameter;

/**
 * Straight-line tracking wheel calibration.
 *
 * PURPOSE:
 *   The spin test (runSpinCalibration) measures wheel OFFSETS (how far each
 *   wheel is from the robot center).  This routine measures wheel DIAMETER
 *   accuracy — any diameter error scales linearly with distance, which is why
 *   long autonomous drives accumulate the most error.
 *
 * PROCEDURE:
 *   1. Place the robot against a known wall/line at the north end of the field
 *      so it will drive exactly 96" south.
 *   2. Run this routine.  The robot drives to the south wall.
 *   3. Read the printed output and update trackingWheelDiameter in main.cpp.
 *   4. After updating the diameter, re-run Spin Calibration to refresh offsets.
 *
 * WHAT IS MEASURED:
 *   - Vertical wheel raw travel vs. expected 96" → suggested corrected diameter.
 *   - Horizontal wheel drift during straight drive → should be ~0.
 *     Non-zero drift means the horizontal wheel is not perpendicular to the
 *     drive direction, or the robot veered (check heading at the end).
 */
void altTrackingWheelCalibration()
{
    const double EXPECTED_DIST_IN = 96.0; // 48 + 48 inches, field tile length

    printf("=== STRAIGHT-LINE TRACKING WHEEL CALIBRATION ===\n");
    printf("Robot will drive %.0f\" straight south (facing 180 cDeg).\n", EXPECTED_DIST_IN);
    printf("Place robot on the north wall tile edge. Starting in 1 s...\n");
    pros::delay(1000);

    // Zero encoders FIRST, then resetPose so it samples the zeroed values.
    // (Same order as runSpinCalibration — avoids phantom delta on first update.)
    verticalTrackingWheel.setAngle(0_stDeg);
    horizontalTrackingWheel.setAngle(0_stDeg);
    pros::delay(20);

    units::Pose initialPose = units::Pose(40_in, 48_in, from_cDeg(180));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    uint32_t t0 = pros::millis();

    printf("[%5lums] START x=%.1f y=%.1f hC=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(initialPose.x), to_in(initialPose.y), to_cDeg(initialPose.orientation));

    pidDriveController.driveToPoint({40_in, -48_in}, 9.0, 6.5_sec);
    pros::delay(300); // let robot settle before reading encoders

    // --- Raw encoder travel (independent of odometry's stored pose) ---
    Length wheelRadius = trackingWheelDiameter / 2.0;
    double vertTravelIn = to_in(to_stRad(verticalTrackingWheel.getAngle()) * wheelRadius);
    double horizTravelIn = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);

    auto finalPose = odometrySystem.getPose();
    double odomDeltaY = to_in(finalPose.y) - to_in(initialPose.y);   // expect -96
    double odomDeltaX = to_in(finalPose.x) - to_in(initialPose.x);   // expect  0
    double finalHeadingErr = to_cDeg(finalPose.orientation) - 180.0; // expect  0

    // IMU cumulative rotation during the drive (compass degrees, CW+).
    // Convert to standard radians (CCW+) the same way runSpinCalibration does.
    double imuRotCDeg = to_cDeg(imu.getRotation()); // absolute, not delta — subtract initial
    // We zeroed encoders but not the IMU; grab only the delta since start.
    // imu.getRotation() is cumulative, so delta = current - snapshot at start.
    // (We didn't snapshot it at start, so use heading error as a proxy here.)
    // Straight-line Δθ in standard radians — used to assess offset contribution.
    double deltaTheta_stRad = -finalHeadingErr * M_PI / 180.0;

    printf("[%5lums] END   x=%.2f y=%.2f hComp=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(finalPose.x), to_in(finalPose.y), to_cDeg(finalPose.orientation));

    printf("\n=== CALIBRATION RESULTS ===\n");
    printf("  Expected drive distance:    %.1f in\n", EXPECTED_DIST_IN);
    printf("  Vertical wheel travel:      %.3f in  (raw encoder * current radius)\n", vertTravelIn);
    printf("  Horizontal wheel travel:    %.3f in  (should be ~0 for straight drive)\n", horizTravelIn);
    printf("  Odometry Y delta:           %.3f in  (expected -%.0f)\n", odomDeltaY, EXPECTED_DIST_IN);
    printf("  Odometry X drift:           %.3f in  (should be ~0)\n", odomDeltaX);
    printf("  Final heading error:        %.2f cDeg  (Δθ ≈ %.4f rad)\n",
           finalHeadingErr, deltaTheta_stRad);

    // --- Diameter suggestion ---
    double currentDiamIn = to_in(trackingWheelDiameter);
    if (std::fabs(vertTravelIn) > 1.0)
    {
        double suggestedDiam = currentDiamIn * (EXPECTED_DIST_IN / std::fabs(vertTravelIn));
        double pctError = 100.0 * (suggestedDiam - currentDiamIn) / currentDiamIn;
        printf("\n  Current trackingWheelDiameter: %.4f in\n", currentDiamIn);
        printf("  >>> SUGGESTED DIAMETER       = %.4f in  (%+.2f%%) <<<\n",
               suggestedDiam, pctError);
        printf("  In main.cpp: Length trackingWheelDiameter = %.4f_in;\n", suggestedDiam);
    }
    else
    {
        printf("\n  WARNING: Vertical wheel travel too small — check encoder connection.\n");
    }

    // --- Horizontal wheel drift diagnosis ---
    printf("\n--- Horizontal Wheel Drift Diagnosis ---\n");
    printf("  Odom X drift:               %.3f in\n", odomDeltaX);
    printf("  Raw horiz encoder travel:   %.3f in\n", horizTravelIn);
    if (std::fabs(horizTravelIn) > 0.05)
    {
        // If the robot veered by Δθ, the offset correction removed:
        //   offsetContrib = horizontalOffset * Δθ_rad
        // The remainder is perpendicularity error or wheel diameter error.
        double currentHorizOffset = -4.081; // keep in sync with main.cpp
        double offsetContrib = currentHorizOffset * deltaTheta_stRad;
        double residual = horizTravelIn - offsetContrib;

        printf("  Δθ accumulated:             %.4f rad (%.2f cDeg)\n",
               deltaTheta_stRad, finalHeadingErr);
        printf("  Offset correction applied:  %.3f in  (horizontalOffset=%.3f × Δθ)\n",
               offsetContrib, currentHorizOffset);
        printf("  Residual (unexplained):     %.3f in\n", residual);
        printf("\n  Interpretation:\n");
        if (std::fabs(deltaTheta_stRad) < 0.01)
        {
            // Δθ ≈ 0: offset correction is negligible, residual = raw
            printf("  Δθ ≈ 0 → offset not a factor. Residual of %.3f in is entirely\n", residual);
            printf("  due to the horiz wheel being physically skewed on its mount.\n");
            printf("  ACTION: Physically rotate the horizontal wheel mount until\n");
            printf("          it reads < 0.05 in over a 96\" straight drive.\n");
        }
        else
        {
            printf("  Δθ is non-trivial. The %.3f in residual after offset correction\n", residual);
            printf("  may be wheel skew OR a noisy result (small Δθ → large offset sensitivity).\n");
            printf("  ACTION: Run Spin Calibration for a clean offset measurement,\n");
            printf("          then re-run this test to verify residual drops near 0.\n");
        }
    }
    else
    {
        printf("  Horizontal wheel drift OK (%.3f in) — perpendicularity looks good.\n", horizTravelIn);
    }

    // --- Physical measurement prompt ---
    printf("\n--- Physical Measurement Check ---\n");
    printf("  Measure the robot's actual lateral displacement with a tape measure.\n");
    printf("  Odom reported X drift: %.3f in\n", odomDeltaX);
    printf("  If physical drift >> odom drift:\n");
    printf("    - Δθ≈0 → horizontal wheel is skewed (fix mounting angle)\n");
    printf("    - Δθ≠0 → horizontal offset is wrong (run Spin Calibration)\n");
    printf("  If physical drift ≈ odom drift: horizontal tracking is good.\n");

    printf("\n  NEXT: Run Spin Calibration to get clean offset values.\n");
}