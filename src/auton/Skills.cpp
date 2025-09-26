#include "auton/CompetitionAutons.hpp"
#include "pros/adi.hpp"
#include "AntistallColorSort.hpp" // Add this include for SnailState definition
#include "control/DifferentialDriveConfig.hpp"

void autonSkills()
{
    units::Pose initialPose = units::Pose(48_in, 12_in, from_cDeg(0));
    odometrySystem.resetPose(initialPose);
    particleFilter.resetPose(initialPose);
    odometrySystem.start();
    particleFilter.start();
    leftMotors.setBrakeMode(lemlib::BrakeMode::BRAKE);
    rightMotors.setBrakeMode(lemlib::BrakeMode::BRAKE);

    // snailState = SnailState::Index; // Only the first stage on
    firstStageIntake.setMaxJiggleCycles(33);

    pidDriveController.driveDistance(33_in, 6.0, 4_sec, true); // Drive to the match loader
    pidDriveController.turnToHeading(90_cDeg, 8.0, 2_sec, true);
 
    scraperPiston.set_value(true);
    pidDriveController.driveDistance(24_in, 5.0, 4_sec, true); // Drive to the match loader

    pros::delay(2000); // Wait to intake the balls

    pidDriveController.driveDistance(-36_in, 4.0, 4_sec, true); // Drive to the goal

    // snailState = SnailState::Long;
    pros::delay(2000); // Wait to intake the balls
    snailState = SnailState::OFF;
    scraperPiston.set_value(false);

    pidDriveController.driveDistance(12_in, 10.0, 4_sec, true); // Backaway from the goal
    pidDriveController.turnToHeading(180_cDeg, 8.0, 2_sec, true);
    pidDriveController.driveToPoint({48_in,-52_in}, 9.0, 9_sec, false, true); // Drive to the match loader

    {
        auto odomPose = odometrySystem.getPose();
        printf("curPose   x=%.2f y=%.2f hComp=%.1f\n",
            to_in(odomPose.x), to_in(odomPose.y), to_cDeg(odomPose.orientation));
        auto pfPose = particleFilter.getPose();
        printf("PF Pose   x=%.2f y=%.2f hComp=%.1f\n",
            to_in(pfPose.x), to_in(pfPose.y), to_cDeg(pfPose.orientation));
    }

    pidDriveController.turnToHeading(90_cDeg, 8.0, 2_sec, true);
    scraperPiston.set_value(true);
    // snailState = SnailState::Index; // Only the first stage on
    pidDriveController.driveDistance(12_in, 5.0, 3_sec, true); // Drive to the match loader
    pros::delay(2000); // Wait to intake the balls

    pidDriveController.turnToHeading(45_cDeg, 8.0, 2_sec, true);
    pidDriveController.driveDistance(-12_in, 5.0, 1.5_sec, true); // Back away

    pidDriveController.turnToHeading(270_cDeg, 8.0, 2_sec, true);
    pidDriveController.driveDistance(90_in, 7.0, 10_sec, true);  // Drive to blue side

    {
        auto odomPose = odometrySystem.getPose();
        printf("curPose   x=%.2f y=%.2f hComp=%.1f\n",
            to_in(odomPose.x), to_in(odomPose.y), to_cDeg(odomPose.orientation));
        auto pfPose = particleFilter.getPose();
        printf("PF Pose   x=%.2f y=%.2f hComp=%.1f\n",
            to_in(pfPose.x), to_in(pfPose.y), to_cDeg(pfPose.orientation));
    }
    snailState = SnailState::OFF;
     
}