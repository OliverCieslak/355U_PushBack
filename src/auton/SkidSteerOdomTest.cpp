#include "auton/SkidSteerOdomTest.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
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
  odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
  // reset tracking wheel encoder so deltas are easy to read
  verticalTrackingWheel.setAngle(0_stDeg);
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

      printf("\tVertWheel: %.2f in\n", vertTravel);
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
  printf("=== SPIN-IN-PLACE OFFSET CALIBRATION ===\n");
  printf("Robot will turn 4x90 = 360 degrees.\n");
  printf("DO NOT TOUCH THE ROBOT.\n");
  pros::delay(1000);

  // Reset encoders FIRST, then resetPose so it samples the zeroed values.
  // If resetPose runs first, it saves the old encoder positions; then
  // setAngle resets them to 0; the first update() sees a huge phantom delta.
  horizontalTrackingWheel.setAngle(0_stDeg);
  verticalTrackingWheel.setAngle(0_stDeg);
  odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
  odometrySystem.start();
  pros::delay(100);

  Length wheelRadius = trackingWheelDiameter / 2.0;

  for (int i = 1; i <= 4; i++) {
      printf("  Turn %d/4...\n", i);
      pidDriveController.turnAngle(90_stDeg, 8.0, 5_sec, true);
      pros::delay(500);

      double horizTravel = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);
      double vertTravel = to_in(to_stRad(verticalTrackingWheel.getAngle()) * wheelRadius);
      units::Pose p = odometrySystem.getPose();
      printf("  After %d deg: horiz=%.2f vert=%.2f | heading=%.1f | x=%.1f y=%.1f\n",
             i * 90, horizTravel, vertTravel, to_cDeg(p.orientation),
             to_in(p.x), to_in(p.y));
  }

  pros::delay(500);

  double finalHorizTravel = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);
  double finalVertTravel = to_in(to_stRad(verticalTrackingWheel.getAngle()) * wheelRadius);
  // Get cumulative rotation as compass degrees (CW positive).
  // Convert to standard radians by negating (compass CW+ → standard CCW+).
  // NOTE: Do NOT use to_stRad() here — that applies the absolute angle
  // transform (π/2 - value), which adds a bogus π/2 offset to what is
  // actually a rotation DELTA, not an absolute heading.
  double actualRotCDeg = to_cDeg(imu.getRotation());
  double actualRotStDeg = -actualRotCDeg;              // standard degrees (CCW+)
  double actualRotRad   = actualRotStDeg * M_PI / 180.0; // standard radians

  printf("\n=== CALIBRATION RESULTS ===\n");
  printf("  IMU measured rotation: %.1f stDeg / %.1f cDeg (%.4f rad)\n", actualRotStDeg, actualRotCDeg, actualRotRad);
  printf("  Horizontal wheel travel: %.3f in\n", finalHorizTravel);
  printf("  Vertical wheel travel:   %.3f in\n", finalVertTravel);
  double horizOffset = finalHorizTravel / actualRotRad;
  double vertOffset = finalVertTravel / actualRotRad;
  printf("\n  >>> HORIZONTAL OFFSET = %.3f inches <<<\n", horizOffset);
  printf("  >>> VERTICAL OFFSET   = %.3f inches <<<\n", vertOffset);
  printf("\n  Plug these into TwoWheelOdometry constructor in main.cpp\n");
  units::Pose fp = odometrySystem.getPose();
  printf("  Final pose: x=%.1f y=%.1f heading=%.1f cDeg (%.1f stDeg)\n",
         to_in(fp.x), to_in(fp.y), to_cDeg(fp.orientation), to_stDeg(fp.orientation));
}