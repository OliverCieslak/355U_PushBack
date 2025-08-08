#include "auton/CompetitionAutons.hpp"
#include "pros/adi.hpp"
#include "AntistallColorSort.hpp" // Add this include for SnailState definition

void autonCenterGoalOnly() {
    units::Pose initialPose = (autonStartingPosition == LeftOrRight::LEFT) ? units::Pose(-57_in, 12_in, 90_cDeg)
                                                          : units::Pose(-57_in, -12_in, -90_cDeg);
    odometrySystem.resetPose(initialPose);
    //autonStartingPosition = LeftOrRight::RIGHT; // Set the starting position for this auton
    snailState = SnailState::INTAKE_TO_BASKET;
    pidDriveController.driveDistance(22_in, 5.0, 2_sec, true); 

    if(autonStartingPosition == LeftOrRight::LEFT) {
        pidDriveController.turnAngle(-120_stDeg, 6.0, 3_sec, true); // Turn towards the center goal
    } else {
        pidDriveController.turnAngle(120_stDeg, 5.0, 3_sec, true); // Turn towards the center goal
    }

    pidDriveController.driveDistance(17_in, 4.0, 4_sec, true); // Drive forward to the center goal
    pidDriveController.driveDistance(10_in, 2.0, 4_sec, true); 
    pros::delay(200);    // Intake to basket

    if(autonStartingPosition == LeftOrRight::LEFT) {
        pidDriveController.turnAngle(20_stDeg, 5.0, 3_sec, true);
    } else {
        pidDriveController.turnAngle(-20_stDeg, 3.0, 3_sec, true);
    }

    pidDriveController.driveDistance(5_in, 2.0, 1_sec, true); 
    pros::delay(200);
    
    if(autonStartingPosition == LeftOrRight::LEFT) {
        pidDriveController.turnAngle(-20_stDeg, 5.0, 3_sec, true);
    } else {
        pidDriveController.turnAngle(20_stDeg, 3.0, 3_sec, true);
    }

    pidDriveController.driveDistance(8_in, 2.0, 4_sec, true);
    pros::delay(200);

    if(autonStartingPosition == LeftOrRight::LEFT) {
       // pidDriveController.turnAngle(33_stDeg, 6.0, 3_sec, true); // Turn towards the center goal
    } else {
        pidDriveController.turnAngle(-33_stDeg, 3.0, 3_sec, true); // Turn towards the center goal
    }
    
    //pidDriveController.driveDistance(10_in, 3.0, 3_sec, true); // Drive forward to the center goal

    if(autonStartingPosition == LeftOrRight::RIGHT) {
        //pidDriveController.driveDistance(3_in, 3.0, 3_sec, false);    a  
    } 

    if(autonStartingPosition == LeftOrRight::LEFT) {
       // pidDriveController.turnAngle(-48_stDeg, 6.0, 3_sec, true); // Turn towards the center goal
       pidDriveController.turnAngle(-14_stDeg, 6.0, 3_sec, true);
    } else {
        pidDriveController.turnAngle(55_stDeg, 3.0, 3_sec, true); // Turn towards the center goal
    }
    if(autonStartingPosition == LeftOrRight::LEFT) {
        pidDriveController.driveDistance(10_in, 6.0, 4_sec, true); // Drive forward to the center goal
    } else {
        pidDriveController.driveDistance(11_in, 3.0, 4_sec, true); // Drive forward to the center goal
    }

    
    conveyorState = ConveyorState::REVERSED; // Start conveyor to score
    if(autonStartingPosition == LeftOrRight::RIGHT) {
        snailState = SnailState::SCORE_LOWER_CENTER;
        //snailState = SnailState::SCORE_LOWER_CENTER;
    } else {
        snailState = SnailState::SCORE_UPPER_CENTER;
    }

}

void autonLoadingZoneLongGoalCenterGoal() {
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

    //skeltonfor awp
    //run this after simple autoin
    pidDriveController.driveDistance(25_in, 5.0, 4_sec, false); //drive backward to matchload

    if(autonStartingPosition == LeftOrRight::LEFT) {
        pidDriveController.turnAngle(-135_stDeg, 5.0, 3_sec, true); // Turn towards the loading zone
    } else {
        pidDriveController.turnAngle(135_stDeg, 5.0, 3_sec, true); // Turn towards the loading zone
    }

    pidDriveController.driveDistance(15_in, 5.0, 4_sec, true); //drive to matchload

    //whatever code to pick up the three balls

    //drive to high goal

    //outtake into high goal
    
}
/**/