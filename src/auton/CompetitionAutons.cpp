#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "main.h"

void autonSevenBallLongGoal()
{
    // Implement the seven ball long goal autonomous routine here

    float xCoordMultiplier = -1.0;
    float yCoordMultiplier = -1.0;
    float heading_multiplier = 1.0;         // blue right
    if (potSelector.get_value() < 4095 / 2) // potentiometer max value is 4095, so anything past halfway is pointing to the right
    {
        yCoordMultiplier = -1.0;
        printf("Starting on Right Side %d\n", potSelector.get_value());
    }
    else
    {
        printf("Starting on Left Side %d\n", potSelector.get_value());
    }

    // allianceColor = AllianceColor::BLUE;
    // auto autonStartingPosition = LeftOrRight::LEFT; // Hardcoded for now, eventually set with selector
    Length x_start = 49_in * xCoordMultiplier;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 18_in * yCoordMultiplier;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    float heading_start = 90;
    // float heading_multiplier = 1.0;
    /*if (allianceColor == AllianceColor::RED)
    {
        heading_multiplier = -1.0;
    }*/
    // heading_multiplier = -1.0;

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
        units::Pose(14_in * xCoordMultiplier, 40_in * yCoordMultiplier, from_cDeg(265 * heading_multiplier)),
        pathConfig,
        7.0,
        4_sec,
        true); // Drive to the 3 balls
    scraperPiston.set_value(true);
    pros::delay(250);

    {
        auto curPose = odometrySystem.getPose();
        printf("curPose   x=%.2f y=%.2f hComp=%.1f\n",
               to_in(curPose.x), to_in(curPose.y), to_cDeg(curPose.orientation));
    }

    pathConfig.lead = 0.7;
    pathConfig.closeDistance = 2_in;
    pidDriveController.driveToPoseBoomerang(
        units::Pose(53_in * xCoordMultiplier, 50_in * yCoordMultiplier, from_cDeg(-90 * heading_multiplier)),
        pathConfig,
        10.0,
        6_sec,
        true

    ); // Drive to line up on the match loader
    {
        auto curPose = odometrySystem.getPose();
        printf("Lining up to MatchLoader - curPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(curPose.x), to_in(curPose.y), to_stDeg(curPose.orientation), to_cDeg(curPose.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(-90 * heading_multiplier), 6.0, 1.5_sec, true); // Turn to face the match loader

    pidDriveController.driveDistance(28_in, 8.0, 1.5_sec, true); // Drive to the match loader
    pros::delay(700);

    pidDriveController.driveDistance(-36_in, 5.0, 4_sec, true); // Drive to the match loader

    {
        auto curPose = odometrySystem.getPose();
        printf("At MatchLoader - curPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(curPose.x), to_in(curPose.y), to_stDeg(curPose.orientation), to_cDeg(curPose.orientation));
    }
    snailState = SnailState::Long;

    pros::delay(4300); // Wait to score the balls
    snailState = SnailState::OFF;
    firstStageIntake.setMaxJiggleCycles(3);

    // pidDriveController.driveDistance(10_in, 12.0, 1_sec, true);
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
    pidDriveController.turnToHeading(180_cDeg, 8.0, 3_sec, true);
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

void autonPartnerSelfAWPLeftSide()
{
    autonPartnerSelfAWP(LeftOrRight::LEFT);
}

void autonPartnerSelfAWPRightSide()
{
    autonPartnerSelfAWP(LeftOrRight::RIGHT);
}

void autonPartnerSelfAWPDialSide()
{
    if (potSelector.get_value() < 4095 / 2) // potentiometer max value is 4095, so anything past halfway is pointing to the right
    {
        autonPartnerSelfAWP(LeftOrRight::RIGHT);
    }
    else
    {
        autonPartnerSelfAWP(LeftOrRight::LEFT);
    }
}

void autonPartnerSelfAWP(LeftOrRight autonStartingPosition)
{
    // We change these values to select the auton before a match
    float xCoordMultiplier = 1.0;
    float yCoordMultiplier = -1.0;
    float heading_multiplier = 1.0;

    if (autonStartingPosition == LeftOrRight::RIGHT)
    {
        yCoordMultiplier = 1.0;
    }

    Length x_start = 60_in * xCoordMultiplier;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 18_in * yCoordMultiplier;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    float heading_start = 0;
    if (autonStartingPosition == LeftOrRight::RIGHT)
    {
        heading_start = 180;
    }

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // pros::delay(9000);   // Give our partner 9 seconds to clear our side of the field
    // Capture and print odometry starting pose after reset
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));

        printf("Driving to %.2f, %.2f, %.2f\n", 36 * xCoordMultiplier, 48 * yCoordMultiplier, 90 * heading_multiplier);
    }

    pidDriveController.driveDistance(-12_in, 6.0, 1_sec, true); // Back away from park
    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnToHeading(from_cDeg(45), 8.0, 1_sec, true); // Turn to face the AWP
    }
    else
    {
        pidDriveController.turnToHeading(from_cDeg(135), 8.0, 1_sec, true); // Turn to face the AWP
    }
    pidDriveController.driveDistance(-20_in, 3.0, 3_sec, true); // Back away from park
    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1_sec, true); // Turn to face the AWP
    }
    else
    {
        pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1_sec, true); // Turn to face the AWP
    }
    pidDriveController.driveDistance(-20_in, 3.0, 3_sec, true); // Back away from park

    {
        auto curPose = odometrySystem.getPose();
        printf("curPose   x=%.2f y=%.2f hComp=%.1f\n",
               to_in(curPose.x), to_in(curPose.y), to_cDeg(curPose.orientation));
    }
}

void autonSevenBallLongGoalAltLeftSide()
{
    autonSevenBallLongGoalAlt(LeftOrRight::LEFT);
}

void autonSevenBallLongGoalAltRightSide()
{
    autonSevenBallLongGoalAlt(LeftOrRight::RIGHT);
}

void autonSevenBallLongGoalAltDialSide()
{
    if (potSelector.get_value() < 4095 / 2) // potentiometer max value is 4095, so anything past halfway is pointing to the right
    {
        autonSevenBallLongGoalAlt(LeftOrRight::RIGHT);
    }
    else
    {
        autonSevenBallLongGoalAlt(LeftOrRight::LEFT);
    }
}

void autonSevenBallLongGoalAlt(LeftOrRight autonStartingPosition)
{
    // We change these values to select the auton before a match
    float xCoordMultiplier = 1.0;
    float yCoordMultiplier = -1.0;
    float heading_multiplier = 1.0;

    if (autonStartingPosition == LeftOrRight::RIGHT)
    {
        yCoordMultiplier = 1.0;
    }

    Length x_start = 60_in * xCoordMultiplier;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 18_in * yCoordMultiplier;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    float heading_start = -90;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    snailState = SnailState::Index; // Only the first stage on
    firstStageIntake.setMaxJiggleCycles(33);

    // Capture and print odometry starting pose after reset
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));

        printf("Driving to %.2f, %.2f, %.2f\n", 36 * xCoordMultiplier, 48 * yCoordMultiplier, 90 * heading_multiplier);
    }

    pidDriveController.driveDistance(36_in, 7.0, 2_sec, true); // Approach 3 stack
    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnToHeading(from_cDeg(-135), 8.0, 1_sec, true); // Turn to face the AWP
    }
    else
    {
        pidDriveController.turnToHeading(from_cDeg(-45), 8.0, 1_sec, true); // Turn to face the AWP
    }
    pidDriveController.driveDistance(20_in, 3.0, 4_sec, true); // Pick up 3 stack
    // pros::delay(100); // Intake more balls
    // pidDriveController.driveDistance(-6_in, 6.0, 1_sec, true); // Pick up 3 stack
    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnToHeading(from_cDeg(120), 8.0, 2.5_sec, true); // Turn to face the AWP
    }
    else
    {
        pidDriveController.turnToHeading(from_cDeg(60), 8.0, 2.5_sec, true); // Turn to face the AWP
    }
    pidDriveController.driveDistance(36_in, 6.0, 4_sec, true);         // Back away from park
    pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1_sec, true); // Turn to face the AWP

    pidDriveController.driveDistance(-20_in, 4.0, 2_sec, true); // Long goal approach

    snailState = SnailState::Long; // Score in long goal
    scraperPiston.set_value(true);
    pros::delay(1000);              // Wait to score the balls
    snailState = SnailState::Index; // Get ready to intake more balls

    pidDriveController.driveDistance(36_in, 6.0, 3_sec, true);  // Long goal approach
    pros::delay(2000);                                          // Intake more balls
    pidDriveController.driveDistance(-36_in, 5.0, 3_sec, true); // Long goal approach
    snailState = SnailState::Long;                              // Score in long goal
    pros::delay(1250);                                          // Wait to score the balls
    snailState = SnailState::OFF;
}

void autonLongAndUpperGoal()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = 48.5_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 13_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(33.5_in, 8.0, 1.75_sec, true); // Drive to match loader
    scraperPiston.set_value(true);
    pidDriveController.turnToHeading(from_cDeg(-90), 9.0, 1_sec, true); // Turn to face the match loader
    // scraperPiston.set_value(true);

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("Face MatchLoad x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Index; // Get ready to match load
    // scraperPiston.set_value(true);
    pidDriveController.turnToHeading(from_cDeg(-90), 9.0, .5_sec, true); // Really face the match loader
    pidDriveController.driveDistance(15_in, 6.0, 1.0_sec, true);         // Drive to match loader
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("At MatchLoad x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    //pros::delay(2000);                                             // Match load
    pidDriveController.driveDistance(-37_in, 5.0, 2.0_sec, true); // Move to long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("At LongGoal x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    scraperPiston.set_value(false);
    snailState = SnailState::Long; // Score LG
    pros::delay(1250);
    snailState = SnailState::Index;

    pidDriveController.driveDistance(18_in, 8.0, 1.75_sec, true); // Drive away from long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("Away From LongGoal x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(133), 9.0, 1_sec, true); // Turn to face the 3 Stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("Turn to 3-stack x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(39_in, 5.0, 2_sec, true); // Drive to 3 stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("At 3-stack x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    pros::delay(250);                                                     // Intake 3 stack
    pidDriveController.turnToHeading(from_cDeg(-36), 9.0, 1.5_sec, true); // Turn to face center goal

    {
        auto startPose = odometrySystem.getPose();
        printf("At 3-stack x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(-21_in, 6.0, 1_sec, true); // Drive to center goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("End at centergoal x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    //snailState = SnailState::Out;
    //pros::delay(150);
    snailState = SnailState::Middle; // Score in upper goal
    //pros::delay(1500);
    //snailState = SnailState::OFF;
}

void autonLongAndLowerGoal()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = 50_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 12_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(35_in, 8.0, 1.75_sec, true); // Drive to match loader
    scraperPiston.set_value(true);
    pidDriveController.turnToHeading(from_cDeg(90), 9.0, 1_sec, true); // Turn to face the match loader

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("Face MatchLoad x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Index; // Get ready to match load
    // scraperPiston.set_value(true);
    // pidDriveController.turnToHeading(from_cDeg(90), 9.0, .5_sec, true); // Really face the match loader
    pidDriveController.driveDistance(19_in, 7.0, 1_sec, true); // Drive to match loader
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("At MatchLoad x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    pros::delay(150);                                             // Match load
    pidDriveController.driveDistance(-37_in, 5.0, 2.0_sec, true); // Move to long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("At LongGoal x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    scraperPiston.set_value(false);
    snailState = SnailState::Long; // Score LG
    pros::delay(950);
    snailState = SnailState::Index;

    pidDriveController.driveDistance(21_in, 8.0, 1.75_sec, true); // Drive away from long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("Away From LongGoal x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(-132), 9.0, 1_sec, true); // Turn to face the 3 Stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("Turn to 3-stack x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(-135), 9.0, .5_sec, true);
    pidDriveController.driveDistance(39_in, 5.0, 2_sec, true); // Drive to 3 stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("At 3-stack x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }
    pros::delay(250); // Intake 3 stack

    pidDriveController.turnToHeading(from_cDeg(-124), 9.0, 1_sec, true);
    pidDriveController.driveDistance(12_in, 6.0, 1_sec, true); // Drive to center goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("End at centergoal x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Out; // Score in upper goal
    pros::delay(1000);
    snailState = SnailState::OFF;
}

void autonPartnerSelfAWPDumb()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = 50_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 12_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(8_in, 8.0, 1.75_sec, true); // Drive to match loader
}

void autonRushLower()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = -56_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = -18_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
               to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Index; // Only the first stage on

    pidDriveController.turnToHeading(from_cDeg(17), 8.0, 1_sec, true);  // Turn to face the 3stack
    pidDriveController.driveDistance(32_in, 5.0, 3_sec, true);          // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(135), 8.0, 1_sec, true); // Turn to face the lower goal
    snailState = SnailState::OFF;
    pidDriveController.driveDistance(33_in, 7.0, 3_sec, true); // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(-24_in, 5.0, 1.5_sec, true); // Drive to 3 stack

    snailState = SnailState::Long;
    pros::delay(1250);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(12_in, 5.0, 1_sec, true); // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(135), 8.0, .75_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(-15_in, 5.0, 1_sec, true); // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(-24_in, 5.0, 3_sec, true); // Drive to 3 stack
	leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
	rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    pros::delay(10000);
}