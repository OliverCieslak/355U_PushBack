#include "auton/CompetitionAutons.hpp"

void autonCenterGoalOnly()
{
    units::Pose initialPose = (autonStartingPosition == LeftOrRight::LEFT) ? units::Pose(-57_in, 12_in, 90_cDeg)
                                                                           : units::Pose(-57_in, -12_in, -90_cDeg);
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    // autonStartingPosition = LeftOrRight::RIGHT; // Set the starting position for this auton
    snailState = SnailState::INTAKE_TO_BASKET;
    pidDriveController.driveDistance(22_in, 5.0, 2_sec, true);

    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnAngle(-120_stDeg, 6.0, 3_sec, true); // Turn towards the center goal
    }
    else
    {
        pidDriveController.turnAngle(120_stDeg, 5.0, 3_sec, true); // Turn towards the center goal
    }

    pidDriveController.driveDistance(17_in, 4.0, 4_sec, true); // Drive forward to the center goal
    pidDriveController.driveDistance(10_in, 2.0, 4_sec, true);
    pros::delay(200); // Intake to basket

    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnAngle(20_stDeg, 5.0, 3_sec, true);
    }
    else
    {
        pidDriveController.turnAngle(-20_stDeg, 3.0, 3_sec, true);
    }

    pidDriveController.driveDistance(5_in, 2.0, 1_sec, true);
    pros::delay(200);

    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnAngle(-20_stDeg, 5.0, 3_sec, true);
    }
    else
    {
        pidDriveController.turnAngle(20_stDeg, 3.0, 3_sec, true);
    }

    pidDriveController.driveDistance(8_in, 2.0, 4_sec, true);
    pros::delay(200);

    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        // pidDriveController.turnAngle(33_stDeg, 6.0, 3_sec, true); // Turn towards the center goal
    }
    else
    {
        pidDriveController.turnAngle(-33_stDeg, 3.0, 3_sec, true); // Turn towards the center goal
    }

    // pidDriveController.driveDistance(10_in, 3.0, 3_sec, true); // Drive forward to the center goal

    if (autonStartingPosition == LeftOrRight::RIGHT)
    {
        // pidDriveController.driveDistance(3_in, 3.0, 3_sec, false);    a
    }

    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        // pidDriveController.turnAngle(-48_stDeg, 6.0, 3_sec, true); // Turn towards the center goal
        pidDriveController.turnAngle(-14_stDeg, 6.0, 3_sec, true);
    }
    else
    {
        pidDriveController.turnAngle(55_stDeg, 3.0, 3_sec, true); // Turn towards the center goal
    }
    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.driveDistance(10_in, 6.0, 4_sec, true); // Drive forward to the center goal
    }
    else
    {
        pidDriveController.driveDistance(11_in, 3.0, 4_sec, true); // Drive forward to the center goal
    }

    conveyorState = ConveyorState::REVERSED; // Start conveyor to score
    if (autonStartingPosition == LeftOrRight::RIGHT)
    {
        snailState = SnailState::SCORE_LOWER_CENTER;
        // snailState = SnailState::SCORE_LOWER_CENTER;
    }
    else
    {
        snailState = SnailState::SCORE_UPPER_CENTER;
    }
}

void autonLoadingZoneLongGoalCenterGoal()
{
    /*
    // Set initial pose based on side
    units::Pose initialPose = (autonStartingPosition == LeftOrRight::LEFT) ? units::Pose(-57_in, 12_in, 90_cDeg)
                                                          : units::Pose(-57_in, -12_in, -90_cDeg);
    odometrySystem.resetPose(initialPose);

    // pidDriveController.turnAngle(180_stDeg, 8.0, 15_sec);

    units::Pose currentPose = odometrySystem.getPose();
    units::Pose nextPose = (autonStartingPosition == LeftOrRight::LEFT) ? units::Pose(-57_in, 40_in, 0_cDeg)
                                             : units::Pose(24_in, 24_in, -90_cDeg);
     pidDriveController.driveDistance(currentPose.distanceTo(nextPose), 8.0, 15_sec, true);

    //pidDriveController.turnToHeading(currentPose.angleTo(nextPose), 8.0, 1_sec, true);
    currentPose = odometrySystem.getPose();
     nextPose = (autonStartingPosition == LeftOrRight::LEFT) ? units::Pose(-57_in, 38_in, 180_cDeg)
                                             : units::Pose(24_in, 24_in, -90_cDeg);
    //pidDriveController.driveDistance(currentPose.distanceTo(currentPose), 8.0, 15_sec, true);
    //pidDriveController.turnToHeading(currentPose.angleTo(nextPose), 8.0, 5_sec, true);
    pidDriveController.turnToHeading(180_stDeg, 8.0, 5_sec, true);
   scraperPiston.set_value(-1); // Move scraper piston down
        currentPose = odometrySystem.getPose();
    nextPose = (autonStartingPosition == LeftOrRight::LEFT) ? units::Pose(-70_in, 40_in, 0_cDeg)
                                             : units::Pose(24_in, 24_in, -90_cDeg);
    pidDriveController.driveDistance(currentPose.distanceTo(nextPose), 16.0, 15_sec, true);
    snailState = SnailState::INTAKE_TO_BASKET;
    */

    // skeltonfor awp
    // run this after simple autoin
    pidDriveController.driveDistance(25_in, 5.0, 4_sec, false); // drive backward to matchload

    if (autonStartingPosition == LeftOrRight::LEFT)
    {
        pidDriveController.turnAngle(-135_stDeg, 5.0, 3_sec, true); // Turn towards the loading zone
    }
    else
    {
        pidDriveController.turnAngle(135_stDeg, 5.0, 3_sec, true); // Turn towards the loading zone
    }

    pidDriveController.driveDistance(15_in, 5.0, 4_sec, true); // drive to matchload

    // whatever code to pick up the three balls

    // drive to high goal

    // outtake into high goal
}

void autonSevenBallLongGoal()
{
    // Implement the seven ball long goal autonomous routine here
    // We flip the sign of the starting X coordinate based on Alliance Color
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
    // TODO - Figure out if we should start the ParticleFilter
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
    // TODO - Figure out if we should start the ParticleFilter
}

void manualTurnTest()
{
    // Implement manual turn test here
    odometrySystem.resetPose(units::Pose(0_in, 0_in, 0_cDeg));
    odometrySystem.start();
    // pidDriveController.turnAngle(180_stDeg, 8.0, 10_sec, true);
    pidDriveController.turnAngle(10_stDeg, 8.0, 10_sec, true);
    units::Pose odomPose = odometrySystem.getPose();
    std::cout << "Odom Pose after turn: (" << odomPose.x.internal() << ", " << odomPose.y.internal() << ", "
              << to_cDeg(odomPose.orientation) << ")\n";
}

void manualLinearTest()
{
    // Implement manual linear test here
    odometrySystem.start();
}