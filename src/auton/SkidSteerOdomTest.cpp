#include "auton/SkidSteerOdomTest.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "control/PIDDriveController.hpp"
#include "odometry/TwoWheelOdometry.hpp"
extern odometry::TwoWheelOdometry odometrySystem;
extern lemlib::V5RotationSensor verticalTrackingWheel;
extern lemlib::V5RotationSensor horizontalTrackingWheel;
extern lemlib::V5InertialSensor imu;
extern Length trackingWheelDiameter;
extern control::PIDDriveController pidDriveController;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;
extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::Motor prosLeft1;
extern pros::Motor prosLeft2;
extern pros::Motor prosLeft3;
extern pros::Motor prosRight1;
extern pros::Motor prosRight2;
extern pros::Motor prosRight3;

void runOdomTest()
{
  printf("Running odometry test...\n");
  // reset tracking wheel encoder FIRST so resetPose captures the zeroed value
  verticalTrackingWheel.setAngle(0_stDeg);
  horizontalTrackingWheel.setAngle(0_stDeg);
  pros::delay(20); // let sensor registers settle
  odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
  printf("Started odom....\n");
  odometrySystem.start();
  Time startTime = from_msec(pros::millis());
  Time testDuration = 15_sec;
  Length wheelRadius = trackingWheelDiameter / 2.0;

  while (from_msec(pros::millis()) - startTime < testDuration)
  {
      // Get current pose from odometry system
      units::Pose currentPose = odometrySystem.getPose();

      // Tracking wheel travel in inches
      double vertTravel = to_in(to_stRad(verticalTrackingWheel.getAngle()) * wheelRadius);

      // Raw PROS rotation sensor diagnostic (port 9)
      pros::Rotation rawRot(9);
      printf("\tVertWheel: %.2f in  (rawCdeg=%d, connected=%d)\n",
             vertTravel, rawRot.get_position(), rawRot.is_installed());
      printf("\tHorizWheel raw: (port20 cdeg=%d, connected=%d)\n",
             pros::Rotation(20).get_position(), pros::Rotation(20).is_installed());
      printf("\tProsLeftMotors: %.2f, ProsRightMotors: %.2f\n", prosLeftMotors.get_position(), prosRightMotors.get_position());
      // Print current position for debugging
      printf("Current Position: x=%.1f, y=%.1f, theta=%.2f\n", 
             to_in(currentPose.x), to_in(currentPose.y), to_cDeg(currentPose.orientation));

      // Wait for 1 second before next update
      pros::delay(1000);
  }

  printf("Odometry test completed after 15 seconds\n");
}

void runSpinCalibration()
{
  printf("=== SPIN-IN-PLACE OFFSET CALIBRATION (720 deg) ===\n");
  printf("Robot will spin 720 degrees continuously (2 full turns CW).\n");
  printf("More rotation = better signal-to-noise on offset measurement.\n");
  printf("DO NOT TOUCH THE ROBOT.\n");
  pros::delay(1000);

  // Reset encoders FIRST, then resetPose so it samples the zeroed values.
  // If resetPose runs first, it saves the old encoder positions; then
  // setAngle resets them to 0; the first update() sees a huge phantom delta.
  horizontalTrackingWheel.setAngle(0_stDeg);
  verticalTrackingWheel.setAngle(0_stDeg);
  pros::delay(20); // let encoder registers settle at 0

  // Snapshot the raw IMU rotation BEFORE the spin so we measure a true
  // delta, not cumulative-since-calibration.  If any prior auton ran, the
  // IMU is not at 0 and reading it raw at the end would give a wrong result.
  //
  // imu.getRotation() returns an Angle stored in standard-radians internally.
  // We extract the scalar in standard degrees for safe arithmetic on a delta
  // (NOT to_cDeg / to_stRad which apply 90−x / π/2−x heading transforms
  //  that are only valid for absolute headings, not for rotation deltas).
  double imuStartStDeg = imu.getRotation().internal() * (180.0 / M_PI);

  odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
  odometrySystem.start();
  pros::delay(100);

  Length wheelRadius = trackingWheelDiameter / 2.0;

  const double TARGET_ROT_DEG = 720.0;
  // 8 × 90° = 720° total. Each step stays within ±180° so constrainAngle180
  // inside turnToHeading never wraps the error to ~0 and quits immediately.
  // Robot fully settles between steps so encoder reads are clean.
  printf("  %6s  %7s  %7s  %7s  %7s  %7s\n",
         "rot°", "horiz", "vert", "odom_x", "odom_y", "head°");

  for (int i = 1; i <= 8; i++) {
      pidDriveController.turnAngle(90_stDeg, 6.0, 5_sec, true);
      pros::delay(300);

      double rotSoFar = (imu.getRotation().internal() * (180.0 / M_PI)) - imuStartStDeg;
      double h = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);
      double v = to_in(to_stRad(verticalTrackingWheel.getAngle())   * wheelRadius);
      units::Pose p = odometrySystem.getPose();
      printf("  %6.1f  %7.3f  %7.3f  %7.3f  %7.3f  %7.1f\n",
             rotSoFar, h, v, to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
  }
  pros::delay(300); // let robot fully settle before sampling

  double finalHorizTravel = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);
  double finalVertTravel  = to_in(to_stRad(verticalTrackingWheel.getAngle())  * wheelRadius);

  // Compute the rotation delta in standard degrees (CCW positive).
  // Using raw .internal() / (π/180) avoids the heading-transform pitfall.
  double imuEndStDeg    = imu.getRotation().internal() * (180.0 / M_PI);
  double actualRotStDeg = imuEndStDeg - imuStartStDeg;  // CCW positive
  double actualRotRad   = actualRotStDeg * M_PI / 180.0;

  printf("\n=== CALIBRATION RESULTS ===\n");
  printf("  Expected rotation:     720.0 stDeg  (%.4f rad)\n", 720.0 * M_PI / 180.0);
  printf("  IMU measured rotation: %.2f stDeg  (%.4f rad)\n",
         actualRotStDeg, actualRotRad);
  printf("  IMU closure error:     %.2f stDeg  (ideal = 0)\n",
         std::fabs(actualRotStDeg) - TARGET_ROT_DEG);
  printf("  Horizontal wheel travel: %.4f in\n", finalHorizTravel);
  printf("  Vertical wheel travel:   %.4f in\n", finalVertTravel);

  double horizOffset = finalHorizTravel / actualRotRad;
  double vertOffset  = finalVertTravel  / actualRotRad;

  printf("\n  >>> HORIZONTAL OFFSET = %.4f in <<<\n", horizOffset);
  printf("  >>> VERTICAL OFFSET   = %.4f in <<<\n",   vertOffset);
  printf("\n  In main.cpp TwoWheelOdometry constructor:\n");
  printf("    3rd-to-last arg (vertOffset):  %.4f_in\n",  vertOffset);
  printf("    2nd-to-last arg (horizOffset): %.4f_in\n",  horizOffset);

  units::Pose fp = odometrySystem.getPose();
  printf("\n  Odom final pose: x=%.2f y=%.2f heading=%.1f cDeg\n",
         to_in(fp.x), to_in(fp.y), to_cDeg(fp.orientation));
  printf("  (x/y should be near 0,0 — any drift indicates residual offset error)\n");
}