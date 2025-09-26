#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "main.h"

void autonFourBallLongGoal()
{
    allianceColor = AllianceColor::BLUE; // Hardcoded for now, eventually set with selector
    // autonStartingPosition = LeftOrRight::LEFT; // Set the starting position for this auton
    // We flip the sign of the starting X coordinate based on Alliance Color
    float xCoordMultiplier = (allianceColor == AllianceColor::BLUE) ? -1.0 : 1.0;
    float yCoordMultiplier = 1.0; 
    if (autonStartingPosition == LeftOrRight::RIGHT && allianceColor == AllianceColor::BLUE)
    {
        yCoordMultiplier = -1.0;
    }
    else if (autonStartingPosition == LeftOrRight::LEFT && allianceColor == AllianceColor::RED)
    {
        yCoordMultiplier = -1.0;
    }
    Length x_start = 48_in * xCoordMultiplier;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 12_in * yCoordMultiplier;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    float heading_start = 90;
    float heading_multiplier = 1.0;
    if (allianceColor == AllianceColor::RED)
    {
        heading_multiplier = -1.0;
    }
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start * heading_multiplier));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Capture and print odometry starting pose after reset
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
            to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Index; // Only the first stage on
    control::BoomerangPathConfig pathConfig(0.7, false);
    pidDriveController.driveToPoseBoomerang(
        units::Pose(17_in * xCoordMultiplier, 31_in * yCoordMultiplier, from_cDeg(265 * heading_multiplier)),
        pathConfig,
        8.0,
        4_sec,
        true
    ); // Drive to the 3 balls

    {
        auto curPose = odometrySystem.getPose();
        printf("curPose   x=%.2f y=%.2f hComp=%.1f\n",
            to_in(curPose.x), to_in(curPose.y), to_cDeg(curPose.orientation));
    }

    pathConfig.lead = 0.5;
    pathConfig.closeDistance = 2_in;
    pidDriveController.driveToPoseBoomerang(
        units::Pose(50_in * xCoordMultiplier, 56_in * yCoordMultiplier, from_cDeg(90 * heading_multiplier)),
        pathConfig,
        10.0,
        5_sec,
        true
    ); // Drive to line up on the match loader

    // Lower the scraper to prepare for intake

    pidDriveController.turnToHeading(from_cDeg(90 * (allianceColor == AllianceColor::BLUE ? -1 : 1)), 8.0, 1_sec, true); // Turn to face the match loader
    pidDriveController.driveDistance(20_in, 3.0, 2_sec, true); // Drive to the match loader
    pros::delay(1000); // Wait to intake the balls
    pidDriveController.driveDistance(-17_in, 6.0, 2_sec, true); // Drive to the long goal
    snailState = SnailState::Long;
    pros::delay(3000); // Wait to score the balls

    // Ram the goal
    pidDriveController.driveDistance(8_in, 4.0, 1_sec, true); // Drive to the match loader
    pidDriveController.driveDistance(-12_in, 12.0, 5_sec, true); // Drive to the match loader

    snailState = SnailState::OFF;
}

void autonSevenBallLongGoal()
{
    // Implement the seven ball long goal autonomous routine here

    allianceColor = AllianceColor::BLUE; // Hardcoded for now, eventually set with selector
    // autonStartingPosition = LeftOrRight::LEFT; // Set the starting position for this auton
    // We flip the sign of the starting X coordinate based on Alliance Color
    float xCoordMultiplier = (allianceColor == AllianceColor::BLUE) ? -1.0 : 1.0;
    float yCoordMultiplier = 1.0; 
    if (autonStartingPosition == LeftOrRight::RIGHT && allianceColor == AllianceColor::BLUE)
    {
        yCoordMultiplier = -1.0;
    }
    else if (autonStartingPosition == LeftOrRight::LEFT && allianceColor == AllianceColor::RED)
    {
        yCoordMultiplier = -1.0;
    }
    Length x_start = 48_in * xCoordMultiplier;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 12_in * yCoordMultiplier;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    float heading_start = 90;
    float heading_multiplier = 1.0;
    if (allianceColor == AllianceColor::RED)
    {
        heading_multiplier = -1.0;
    }
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start * heading_multiplier));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Capture and print odometry starting pose after reset
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
            to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));

        printf("Driving to %.2f, %.2f, %.2f\n", 17 * xCoordMultiplier, 31 * yCoordMultiplier, 265 * heading_multiplier);
   }

    snailState = SnailState::Index; // Only the first stage on
    firstStageIntake.setMaxJiggleCycles(33);
    control::BoomerangPathConfig pathConfig(0.7, false);
    pidDriveController.driveToPoseBoomerang(
        units::Pose(17_in * xCoordMultiplier, 31_in * yCoordMultiplier, from_cDeg(265 * heading_multiplier)),
        pathConfig,
        8.0,
        4_sec,
        true
    ); // Drive to the 3 balls

    {
        auto curPose = odometrySystem.getPose();
        printf("curPose   x=%.2f y=%.2f hComp=%.1f\n",
            to_in(curPose.x), to_in(curPose.y), to_cDeg(curPose.orientation));
    }

    pathConfig.lead = 0.5;
    pathConfig.closeDistance = 2_in;
    pidDriveController.driveToPoseBoomerang(
        units::Pose(50_in * xCoordMultiplier, 58_in * yCoordMultiplier, from_cDeg(90 * heading_multiplier)),
        pathConfig,
        10.0,
        5_sec,
        true
    ); // Drive to line up on the match loader

    // Lower the scraper to prepare for intake

    

    scraperPiston.set_value(true);
    pidDriveController.turnToHeading(from_cDeg(90 * (allianceColor == AllianceColor::BLUE ? -1 : 1)), 8.0, 1_sec, true); // Turn to face the match loader
    
    pros::delay(300); 
    snailState = SnailState::Long;
    pidDriveController.driveDistance(22_in, 8.0, 4_sec, true); // Drive to the match loader

    for(int gregory = 0; gregory < 3; gregory++) {
        pidDriveController.driveDistance(-1_in, 7.0, 0.1_sec, true);
        pidDriveController.driveDistance(1_in, 7.0, 0.1_sec, true);
    }

    pros::delay(500); // Wait to intake the balls
    pidDriveController.driveDistance(-17_in, 4.0, 3_sec, true); // Drive to the long goal
    snailState = SnailState::Long;
    
    HoodClose.set_value(true);
    pros::delay(2500); // Wait to score the balls
    snailState = SnailState::OFF;
    firstStageIntake.setMaxJiggleCycles(3);

    // Ram the goal
    pidDriveController.driveDistance(8_in, 4.0, 1_sec, true); // Drive to the match loader
    pidDriveController.driveDistance(-12_in, 12.0, 5_sec, true); // Drive to the match loader
    
}

void autonNineBallLongGoal()
{
    // Implement the nine ball long goal autonomous routine here
    Length x_start = (allianceColor == AllianceColor::RED) ? -57_in : 57_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = (autonStartingPosition == LeftOrRight::RIGHT) ? -12_in : -12_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    if (autonStartingPosition == LeftOrRight::RIGHT && allianceColor == AllianceColor::BLUE)
    {
        heading_start = -180;
    }
    if (autonStartingPosition == LeftOrRight::LEFT && allianceColor == AllianceColor::RED)
    {
        heading_start = -180;
    }
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
}

void manualTurnTest()
{
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    pidDriveController.turnToHeading(90_cDeg, 8.0, 10_sec, true);
    units::Pose odomPose = odometrySystem.getPose();
    std::cout << "Odom Pose after turn: (" << to_in(odomPose.x) << ", " << to_in(odomPose.y) << ", "
    << to_cDeg(odomPose.orientation) << ")\n";
}

void manualLinearTest()
{
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    pidDriveController.driveDistance(48_in, 8.0, 10_sec, true);
    units::Pose odomPose = odometrySystem.getPose();
    std::cout << "Odom Pose after drive: (" << to_in(odomPose.x) << ", " << to_in(odomPose.y) << ", "
              << to_cDeg(odomPose.orientation) << ")\n";
}