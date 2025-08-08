#include "auton/SkidSteerOdomTest.hpp"

extern odometry::SkidSteerOdometry odometrySystem;
extern viz::FieldView fieldView;
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
  printf("Started odom....\n");
  Time startTime = from_msec(pros::millis());
  Time testDuration = 15_sec;

  while (from_msec(pros::millis()) - startTime < testDuration)
  {
      // Get current pose from odometry system
      units::Pose currentPose = odometrySystem.getPose();

      // Update field visualization with current pose
      // fieldView.updateRobotPosition(currentPose, from_msec(pros::millis()) - startTime);

      printf("\tProsLeftMotors: %.2f, ProsRightMotors: %.2f\n", prosLeftMotors.get_position(), prosRightMotors.get_position());
      printf("\tLeftMotors: %.2f, RightMotors: %.2f\n", to_stDeg(leftMotors.getAngle()), to_stDeg(rightMotors.getAngle()));
      // Print current position for debugging
      printf("Current Position: x=%.1f, y=%.1f, theta=%.2f\n", 
             to_in(currentPose.x), to_in(currentPose.y), to_cDeg(currentPose.orientation));

      // Wait for 1 second before next update
      pros::delay(1000);
  }

  printf("Odometry test completed after 15 seconds\n");

}