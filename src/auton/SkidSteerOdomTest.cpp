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
  printf("Robot will turn 4x90 = 360 degrees using PID.\n");
  printf("DO NOT TOUCH THE ROBOT.\n");
  pros::delay(1000);

  odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
  horizontalTrackingWheel.setAngle(0_stDeg);
  odometrySystem.start();
  pros::delay(100);

  Length wheelRadius = trackingWheelDiameter / 2.0;

  // Do 4 x 90 degree turns = 360 total
  for (int i = 1; i <= 4; i++) {
      printf("  Turn %d/4...\n", i);
      pidDriveController.turnAngle(90_stDeg, 8.0, 3_sec, true);
      pros::delay(250);

      double vertTravel = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);
      units::Pose p = odometrySystem.getPose();
      printf("  After %d deg: vert=%.2f in | heading=%.1f | x=%.1f y=%.1f\n",
             i * 90, vertTravel, to_cDeg(p.orientation),
             to_in(p.x), to_in(p.y));
  }

  pros::delay(500);

  // Read final values
  double finalVertTravel = to_in(to_stRad(horizontalTrackingWheel.getAngle()) * wheelRadius);
  // We commanded exactly 360 deg
  double totalRotRad = 4.0 * to_stRad(90_stDeg);

  printf("\n=== CALIBRATION RESULTS ===\n");
  printf("  Commanded rotation: 360 deg (%.4f rad)\n", totalRotRad);
  printf("  Horizontal wheel travel: %.3f in\n", finalVertTravel);
  double horizOffset = finalVertTravel / totalRotRad;
  printf("\n  >>> HORIZONTAL OFFSET = %.3f inches <<<\n", horizOffset);
  printf("\n  Plug this into TwoWheelOdometry constructor in main.cpp\n");
  printf("  Final pose: x=%.1f y=%.1f (should both be ~0 with correct offset)\n",
         to_in(odometrySystem.getPose().x), to_in(odometrySystem.getPose().y));
}