#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "main.h"

void manualTurnTest()
{
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    uint32_t t0 = pros::millis();
    pidDriveController.turnToHeading(180_cDeg, 8.0, 3_sec, true);
    units::Pose odomPose = odometrySystem.getPose();
    printf("[%5lums] afterTurn x=%.2f y=%.2f hComp=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(odomPose.x), to_in(odomPose.y), to_cDeg(odomPose.orientation));
}

void manualLinearTest()
{
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    uint32_t t0 = pros::millis();
    pidDriveController.driveDistance(48_in, 8.0, 10_sec, true);
    units::Pose odomPose = odometrySystem.getPose();
    printf("[%5lums] afterDrive x=%.2f y=%.2f hComp=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(odomPose.x), to_in(odomPose.y), to_cDeg(odomPose.orientation));
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

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(8_in, 8.0, 1.75_sec, true); // Drive to match loader
}

void matchLoadToLongGoalStart(UpperOrLower goalSide, uint32_t t0)
{
    scraperPiston.set_value(true);
    snailState = SnailState::Index; // Get ready to match load

    // May need slightly different movements based on starting side
    if (goalSide == UpperOrLower::UPPER)
    {
        printf("Starting Upper Goal Side\n");
        pidDriveController.driveDistance(33_in, 8.0, 2_sec, true); // Drive to match loader
    }
    else
    {
        printf("Starting Lower Goal Side\n");
        pidDriveController.driveDistance(31_in, 8.0, 2_sec, true); // Drive to match loader
    }

    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1_sec, true); // Turn to face the match loader

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] face ml x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(14_in, 6.5, 1.0_sec, true); // Drive to match loader
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] match load x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pros::delay(200); // Match load
    snailState = SnailState::OFF;

    // Since we are setting starting heading, this should always turn the right way
    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, .5_sec, true); // Turn to face the match loader
    pidDriveController.driveDistance(-35.5_in, 6.5, 1.25_sec, true);     // Move to long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] LG x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    snailState = SnailState::Long; // Score LG
    scraperPiston.set_value(false);
    pros::delay(1250);
}

void autonLongAndUpperGoal()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = -48.5_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 13_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    matchLoadToLongGoalStart(UpperOrLower::UPPER, t0);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(20_in, 8.0, 1.25_sec, true); // Drive away from long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] away lg x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(140), 9.0, 1_sec, true); // Turn to face the 3 Stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] face 3stack x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    snailState = SnailState::Index;
    pidDriveController.driveDistance(41_in, 4.5, 2.25_sec, true); // Drive to 3 stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] at 3stack x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(-45), 9.0, 1.5_sec, true); // Turn to face center goal

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] face cg x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pidDriveController.driveDistance(-14_in, 6.0, 1_sec, true); // Drive to center goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] at cg x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    // snailState = SnailState::Out;
    snailState = SnailState::Middle; // Score in upper goal
    pros::delay(1250);
    snailState = SnailState::OFF;
    /*
    pidDriveController.driveDistance(32_in, 7.0, 1_sec, true);            // Drive to center goal
    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1_sec, true);   // Turn to push blocks
    pidDriveController.driveDistance(-16_in, 7.0, 1.25_sec, true);         // Drive blocks
    pidDriveController.driveDistance(4_in, 7.0, .5_sec, true);         // Drive blocks
    leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    */
}

void autonRushUpper()
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

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    matchLoadToLongGoalStart(UpperOrLower::UPPER, t0);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(10_in, 5.0, 1_sec, true);            // Away from LG
    pidDriveController.turnToHeading(from_cDeg(-45), 8.0, .75_sec, true); // Turn to go by LG
    pidDriveController.driveDistance(-13_in, 5.0, 1_sec, true);           // Move to be next to LG
    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1_sec, true);   // Turn to push blocks
    pidDriveController.driveDistance(-24_in, 5.0, 2.5_sec, true);         // Drive blocks
    pros::delay(500);
    pidDriveController.driveDistance(6_in, 5.0, .5_sec, true); // Drive blocks

    leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
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

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(35_in, 8.0, 1.75_sec, true); // Drive to match loader
    scraperPiston.set_value(true);
    pidDriveController.turnToHeading(from_cDeg(90), 9.0, 1_sec, true); // Turn to face the match loader

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] Face MatchLoad x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Index; // Get ready to match load
    // scraperPiston.set_value(true);
    // pidDriveController.turnToHeading(from_cDeg(90), 9.0, .5_sec, true); // Really face the match loader
    pidDriveController.driveDistance(19_in, 7.0, 1_sec, true); // Drive to match loader
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] At MatchLoad x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pros::delay(150);                                             // Match load
    pidDriveController.driveDistance(-37_in, 5.0, 2.0_sec, true); // Move to long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] At LongGoal x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    scraperPiston.set_value(false);
    snailState = SnailState::Long; // Score LG
    pros::delay(950);
    snailState = SnailState::Index;

    pidDriveController.driveDistance(21_in, 8.0, 1.75_sec, true); // Drive away from long goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] Away From LongGoal x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(-132), 9.0, 1_sec, true); // Turn to face the 3 Stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] Turn to 3-stack x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(-135), 9.0, .5_sec, true);
    pidDriveController.driveDistance(39_in, 5.0, 2_sec, true); // Drive to 3 stack
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] At 3-stack x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pros::delay(250); // Intake 3 stack

    pidDriveController.turnToHeading(from_cDeg(-124), 9.0, 1_sec, true);
    pidDriveController.driveDistance(12_in, 6.0, 1_sec, true); // Drive to center goal
    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] End at centergoal x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Out; // Score in upper goal
    pros::delay(1000);
    snailState = SnailState::OFF;
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

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    snailState = SnailState::Index; // Only the first stage on

    pidDriveController.turnToHeading(from_cDeg(17), 8.0, 1_sec, true);  // Turn to face the 3stack
    pidDriveController.driveDistance(32_in, 5.0, 3_sec, true);          // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(135), 8.0, 1_sec, true); // Turn to face the lower goal
    snailState = SnailState::OFF;
    pidDriveController.driveDistance(33_in, 7.0, 3_sec, true);          // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(-24_in, 5.0, 1.5_sec, true);       // Drive to 3 stack

    snailState = SnailState::Long;
    pros::delay(1250);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(12_in, 5.0, 1_sec, true);            // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(135), 8.0, .75_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(-15_in, 5.0, 1_sec, true);           // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1_sec, true);   // Turn to face the lower goal
    pidDriveController.driveDistance(-24_in, 5.0, 3_sec, true);           // Drive to 3 stack
    leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    pros::delay(10000);
}

void autonRushLowerMatchLoaderAlleyEnd()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = 48.5_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = -13_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 180;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    matchLoadToLongGoalStart(UpperOrLower::LOWER, t0);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(10_in, 5.0, 1_sec, true);            // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(-135), 8.0, .75_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(10_in, 5.0, 1_sec, true);           // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1_sec, true);   // Turn to face the lower goal
    pidDriveController.driveDistance(-36_in, 8.0, 3_sec, true);           // Drive to 3 stack
    leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    pros::delay(10000);
}

void movementTest()
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = -47.5_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 14.5_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    double heading_start = 0;
    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    uint32_t t0 = pros::millis();

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] startPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    // Drive to x=-48.5, y=48.5, h=0
    pidDriveController.driveToPoint(units::Pose(-47.5_in, 47.5_in, from_cDeg(0)), 10.0, 3_sec, false);

    // Print odometry pose for debugging
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1.5_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pidDriveController.driveDistance(14_in, 6.0, 1_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    // Intake
    pros::delay(200);
    // Intake off

    control::BoomerangPathConfig config;
    config.lead = 0.6; // adjust curvature as needed
    config.reversed = true;
    config.minLateralSpeed = 0.6;
    pidDriveController.driveToPoseBoomerang(
        units::Pose(-33_in, 48_in, from_cDeg(-90)),
        config, // pass the config
        10.0,   // max voltage
        3_sec,  // timeout
        true    // wait
    );
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    // Score Upper
    pros::delay(1000);
    // Index off
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveDistance(6_in, 8.0, 1_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    // Turn to face the point (-24, 24)
    pidDriveController.turnToPoint({-22_in, 18_in}, 8.0, 2_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    // Index On
    pidDriveController.driveToPoint({-22_in, 18_in}, 8.0, 1.5_sec, false);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.turnAwayFromPoint({-12_in, 8_in}, 8.0, 1_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }

    pidDriveController.driveToPoint({-12_in, 8_in}, 8.0, 1.2_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    // Score Upper
    pros::delay(600);

    // pidDriveController.driveToPoint({-40_in, 24_in}, 8.0, 1.5_sec, false);
    /*
    pidDriveController.turnToPoint({-36_in, 45_in}, 8.0, 1_sec, true);
    pidDriveController.driveDistance(33_in, 9.0, 1.25_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1_sec, true);
    pidDriveController.driveDistance(-30_in, 9.0, 1.25_sec, true);
    leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    */
}