#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "main.h"
#include <cmath>
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

void mpStartToMatchLoader() {
    units::Pose initialPose = units::Pose(0_in, 0_in, from_cDeg(0));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    uint32_t t0 = pros::millis();

    // ── Curved path: start (0,0) heading 0°, curve to match loader at (-9,36) heading -90° ──
    motion::TrajectoryConfig trajCfg(maxVelocity, maxAccel, 80_inps2);

    // Waypoints with headings that guide the spline into a smooth curve:
    //   Start:  (0, 0)   heading 0°   → driving straight forward
    //   Mid:    (0, 16)  heading -20°  → gentler initial curve
    //   End:    (-13, 25.5) heading -90°  → facing left at the match loader
    std::vector<units::Pose> waypoints = {
        units::Pose( 0_in,  0_in, from_cDeg(0)),
        units::Pose( 0_in, 15_in, from_cDeg(-20)),
        units::Pose(-13_in, from_in(25.5), from_cDeg(-90))
    };

    auto traj = motion::TrajectoryGenerator::generateTrajectory(waypoints, trajCfg);
    printf("Traj: %zu pts, dist=%.1f in, dur=%.2f s\n",
           traj.getStates().size(),
           to_in(traj.getTotalDistance()),
           to_sec(traj.getTotalTime()));

    // ── Pure pursuit controller (global) ──
    motion::PurePursuitConfig ppCfg;
    ppCfg.lookahead = 8_in;
    ppCfg.waypointTolerance = 2_in;
    ppCfg.finalPivot = true;
    ppCfg.finalPivotTolerance = from_stDeg(5);
    ppCfg.dynamicLookahead = true;
    ppCfg.dynLookMin = 6_in;
    ppCfg.dynLookMax = 14_in;

    purePursuitController.setConfig(ppCfg);
    purePursuitController.setTrajectory(traj);
    purePursuitController.setRequireFinalHeading(true);

    // ── Follow with telemetry ──
    purePursuitController.followPath(true); // async
    while (purePursuitController.isFollowing()) {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] x=%.1f y=%.1f hC=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
        pros::delay(200);
    }

    auto endPose = odometrySystem.getPose();
    printf("[%5lums] DONE x=%.2f y=%.2f hComp=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(endPose.x), to_in(endPose.y), to_cDeg(endPose.orientation));
    purePursuitController.stop();


    //////////////////////////////////////////////////////////////////////
    // next movement: load from match loader
    pros::delay(500); // Load blocks from ML

    //////////////////////////////////////////////////////////////////////
    // next movement: drive backwards to long goal (fixed target, reversed)
    {
        control::PIDDriveController::Point longGoalPt(14_in, 32_in);
        pidDriveController.driveToPoint(longGoalPt, 6.0, 2_sec, true, true);
    }
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] afterReverse x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }

    pros::delay(750); // Score blocks

    //////////////////////////////////////////////////////////////////////
    // next movement: drive forward away from long goal (fixed target)
    {
        control::PIDDriveController::Point pullForwardPt(0_in, 32_in);
        pidDriveController.driveToPoint(pullForwardPt, 8.0, 1_sec, false, true);
    }
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] afterForward x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }

    //////////////////////////////////////////////////////////////////////
    // next movement: S-curve backwards — 8" left, 24" back (robot frame)
    {
        auto startPose = odometrySystem.getPose();
        double sx = to_in(startPose.x);
        double sy = to_in(startPose.y);
        double h  = to_stRad(startPose.orientation); // heading in standard radians

        // Backward and Left unit vectors in field frame
        double backX = -std::cos(h), backY = -std::sin(h);   // backward
        double leftX = -std::sin(h), leftY =  std::cos(h);   // robot left

        // Mid-point: 10" back, 18" left (apex of outward curve)
        double midX = sx + backX * 10.0 + leftX * 18.0;
        double midY = sy + backY * 10.0 + leftY * 18.0;
        // End-point: 24" back, 6" left (pulls back close to goal line)
        double endX = sx + backX * 24.0 + leftX * 6.0;
        double endY = sy + backY * 24.0 + leftY * 6.0;

        // For reversed trajectory, the trajectory generator uses waypoint
        // headings as spline tangent directions.  Tangents must point in the
        // direction of TRAVEL (backward), not the robot's facing direction.
        double backH = h + M_PI; // backward heading in standard radians
        Angle startSplineH = from_stRad(backH);
        Angle midSplineH   = from_stRad(backH - 0.50); // ~30° toward robot left
        Angle endSplineH   = from_stRad(backH + 0.40);   // ~23° back toward right (completing S)

        printf("S-curve wp: start(%.1f,%.1f) mid(%.1f,%.1f) end(%.1f,%.1f) backH=%.1f\n",
               sx, sy, midX, midY, endX, endY, backH * 180.0 / M_PI);

        motion::TrajectoryConfig trajCfg2(maxVelocity, maxAccel, 80_inps2);
        trajCfg2.setReversed(true);

        std::vector<units::Pose> sCurveWaypoints = {
            units::Pose(from_in(sx), from_in(sy), startSplineH),
            units::Pose(from_in(midX), from_in(midY), midSplineH),
            units::Pose(from_in(endX), from_in(endY), endSplineH)
        };

        auto sTraj = motion::TrajectoryGenerator::generateTrajectory(sCurveWaypoints, trajCfg2);
        printf("S-curve traj: %zu pts, dist=%.1f in, dur=%.2f s\n",
               sTraj.getStates().size(),
               to_in(sTraj.getTotalDistance()),
               to_sec(sTraj.getTotalTime()));

        motion::PurePursuitConfig ppCfg2;
        ppCfg2.lookahead = 8_in;
        ppCfg2.waypointTolerance = 2_in;
        ppCfg2.finalPivot = false;
        ppCfg2.dynamicLookahead = true;
        ppCfg2.dynLookMin = 6_in;
        ppCfg2.dynLookMax = 14_in;

        purePursuitController.setConfig(ppCfg2);
        purePursuitController.setTrajectory(sTraj);
        purePursuitController.setRequireFinalHeading(false);

        purePursuitController.followPath(true); // async
        while (purePursuitController.isFollowing()) {
            auto pose = odometrySystem.getPose();
            printf("[%5lums] Scurve x=%.1f y=%.1f hC=%.1f\n",
                   (unsigned long)(pros::millis() - t0),
                   to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
            pros::delay(200);
        }

        auto sPose = odometrySystem.getPose();
        printf("[%5lums] S-DONE x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(sPose.x), to_in(sPose.y), to_cDeg(sPose.orientation));
        purePursuitController.stop();
    }

    //////////////////////////////////////////////////////////////////////
    // next movement: reverse 15" to swipe blocks toward control zone
    {
        auto pose = odometrySystem.getPose();
        double h = to_stRad(pose.orientation);
        // 15" behind the robot
        double tgtX = to_in(pose.x) - std::cos(h) * 15.0;
        double tgtY = to_in(pose.y) - std::sin(h) * 15.0;
        control::PIDDriveController::Point swipePt(from_in(tgtX), from_in(tgtY));
        pidDriveController.driveToPoint(swipePt, 8.0, 2_sec, true, true);
    }
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] afterSwipe x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }

}