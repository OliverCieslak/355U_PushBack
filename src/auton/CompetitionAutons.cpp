#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "main.h"
// use this for debugging odometry pose
/*{
    auto startPose = odometrySystem.getPose();
    printf("[%5lums] face ml x=%.2f y=%.2f hComp=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
}*/

// NOTE: for Turning PID
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
// Note: for Linear PID
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

    pidDriveController.driveDistance(8_in, 8.0, 1.75_sec, true); // Drive to match loader
}

void ParkZoneToMatchLoader(UpperOrLower goalSide, uint32_t t0)
{
    // Implement the long and upper goal autonomous routine here
    Length x_start = 50_in;
    // We flip the sign of the starting Y coordinate based on Starting Position (Left/Right)
    Length y_start = 12_in;
    // We flip the sign of the starting heading based on Starting Position (Left/Right) and Alliance
    scraperPiston.set_value(true);
    snailState = SnailState::Index; // Get ready to match load
    pidDriveController.driveToPoint(units::Pose(-47.5_in, 47.5_in, from_cDeg(0)), 10.0, 3_sec, false);

    // May need slightly different movements based on starting side
    if (goalSide == UpperOrLower::UPPER)
    {
        printf("Starting Upper Goal Side\n");
        pidDriveController.driveDistance(33_in, 8.0, 2_sec, true); // Drive to match loader
    }
    else
    {
        printf("Starting Lower Goal Side\n");
        pidDriveController.driveDistance(30_in, 8.0, 2_sec, true); // Drive to match loader
    }

    pidDriveController.turnToHeading(from_cDeg(-88), 15, 1_sec, true); // Turn to face the match loader

    pidDriveController.driveDistance(14_in, 20, 2.0_sec, true); // Drive to match loader
    pros::delay(100);                                           // wait at match loader for ball to be intaked
    snailState = SnailState::OFF;
}
void MatchLoaderToLongGoal(UpperOrLower goalSide, uint32_t t0)
{
    pidDriveController.driveDistance(-35.5_in, 6.5, 1.0_sec, true); // Move to long goal
    snailState = SnailState::Long;                                  // Score LG
    scraperPiston.set_value(false);
    pros::delay(1000);
    // this so that way we only have to change one nuymber to change all autons.
}
void autonLongAndUpperGoal()
{
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

    ParkZoneToMatchLoader(UpperOrLower::UPPER, t0);
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
}

void autonRushLower()
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

    ParkZoneToMatchLoader(UpperOrLower::LOWER, t0);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(10_in, 5.0, 1_sec, true);             // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(-135), 8.0, .75_sec, true); // Turn to face the lower goal
    pidDriveController.driveDistance(10_in, 5.0, 1_sec, true);             // Drive to 3 stack
    pidDriveController.turnToHeading(from_cDeg(-94), 8.0, 1_sec, true);    // Turn to face the lower goal
    pidDriveController.driveDistance(-38_in, 5.0, 1.5_sec, true);          // Drive to 3 stack
    leftMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    rightMotors.setBrakeMode(lemlib::BrakeMode::HOLD);
    pros::delay(100);
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

    scraperPiston.set_value(true);
    snailState = SnailState::Index; // Get ready to match load
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
    snailState = SnailState::OFF;

    control::BoomerangPathConfig config;
    config.lead = 0.6; // adjust curvature as needed
    config.reversed = true;
    config.minLateralSpeed = 0.55;
    pidDriveController.driveToPoseBoomerang(
        units::Pose(-32_in, 48_in, from_cDeg(-90)),
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
    snailState = SnailState::Long; // Score LG
    // Score Upper
    pros::delay(1000);
    snailState = SnailState::OFF;
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

    snailState = SnailState::Index;
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

    pidDriveController.driveToPoint({-15_in, 11_in}, 6.0, 1.2_sec, true);
    {
        auto startPose = odometrySystem.getPose();
        printf("[%5lums] nextPose x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(startPose.x), to_in(startPose.y), to_cDeg(startPose.orientation));
    }
    snailState = SnailState::Middle;
    pros::delay(600);
}

void Start_MatchLoad()
{
    /*
    
    Length x_start = 0_in;
    Length y_start = 0_in;
    double heading_start = 0;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    */

    uint32_t t0 = pros::millis();

    pidDriveController.driveToPoint(units::Pose(0_in, 31_in, from_cDeg(0)), 12.0, 1.0_sec, false);
    scraperPiston.set_value(true);
    pros::delay(10);

    pidDriveController.turnToHeading(from_cDeg(-90), 12.0, 1.0_sec, true);

    snailState = SnailState::Index; 

    pros::delay(50);

    pidDriveController.driveToPoint(units::Pose(-16_in, 31_in, from_cDeg(0)), 12.0, 1.0_sec, false);

    //pros::delay(100); 

    //auton = working
}
    
void MatchLoad_LongGoal() {

    /*

    Length x_start = -40_in;
    Length y_start = -20_in;
    double heading_start = -180;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    */

    //pidDriveController.driveToPoint(units::Pose(18_in, 33_in, from_cDeg(0)), 12.0, 1_sec, true); 

    pidDriveController.driveToPoint(units::Pose(24_in, 31_in, from_cDeg(0)), 12.0, 1.0_sec, true);

    snailState = SnailState::Long; 

    //pros::delay(200); 
}
void LongGoal_ThreeBlocks() {

    /*

    Length x_start = -40_in;
    Length y_start = -20_in;
    double heading_start = -180;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    */

    

    
    pidDriveController.driveToPoint(units::Pose(20_in, 33_in, from_cDeg(0)), 10.0, 1_sec, false); 
    pidDriveController.turnAngle(from_cDeg(210), 12.0, 2_sec, true);
    //snailState = SnailState::Long; 

    //pros::delay(200); 
}
void MatchLoad_Wing() {

    /*

    Length x_start = -40_in;
    Length y_start = -20_in;
    double heading_start = -180;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    */

    /*
    control::BoomerangPathConfig config;
    config.lead = 0.6; // adjust curvature as needed
    config.reversed = false;
    config.minLateralSpeed = 0.55;
     pidDriveController.driveToPoseBoomerang(
        units::Pose(24_in, 40_in, from_cDeg(90)),
        config, // pass the config
        10.0,   // max voltage
        3_sec,  // timeout
        true    // wait
    );

    */
    
    pidDriveController.driveToPoint(units::Pose(0_in, 31_in, from_cDeg(0)), 12.0, 1_sec, false); 

    snailState = SnailState::OFF; 

    pidDriveController.turnAngle(from_cDeg(129), 12.0, 1_sec, true);

    pros::delay(50);

    pidDriveController.driveToPoint(units::Pose(20_in, 19.5_in, from_cDeg(0)), 12.0, 1.0_sec, true);

    wingState = WingState::LEFTUP; 

    pros::delay(20);

    pidDriveController.turnToHeading(from_cDeg(-90), 12.0, 1_sec, true);

    //wingState = WingState::RIGHTUP;

    pidDriveController.driveToPoint(units::Pose(25_in, 23_in, from_cDeg(0)), 12.0, 1.0_sec, true);

    pidDriveController.driveToPoint(units::Pose(37_in, 25_in, from_cDeg(0)), 12.0, 1.0_sec, true);

    pros::delay(40); 
}

void BaconEggAndCheese() {
    
    Length x_start = 0_in;
    Length y_start = 0_in;
    double heading_start = 0;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    uint32_t t0 = pros::millis();

    Start_MatchLoad();

    pros::delay(400);

    MatchLoad_LongGoal();

    pros::delay(1900);

    MatchLoad_Wing();
}

void DoubleBaconAndEgg() {
    
    Length x_start = 0_in;
    Length y_start = 0_in;
    double heading_start = 0;

    units::Pose initialPose = units::Pose(x_start, y_start, from_cDeg(heading_start));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    uint32_t t0 = pros::millis();

    Start_MatchLoad();

    pros::delay(400);

    MatchLoad_LongGoal();
    pros::delay(1900);

    LongGoal_ThreeBlocks();
}