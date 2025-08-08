// Includes
#include <vector>
#include <cmath>
#include "auton/ParticleFilterTest.hpp"
#include "main.h"

#include "units/Pose.hpp"
#include "utils/DistanceUtils.hpp"
#include "odometry/SkidSteerOdometry.hpp"
#include "localization/ParticleFilter.hpp"
#include "viz/FieldView.hpp"
#include "viz/DiagnosticsView.hpp"
#include "tuning/CharacterizationView.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"

// External references declared in main.cpp
extern odometry::SkidSteerOdometry odometrySystem;
extern localization::ParticleFilter particleFilter;
extern viz::FieldView fieldView;
extern viz::DiagnosticsView diagnosticsView;
extern tuning::CharacterizationView characterizationView;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;
extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::v5::Motor prosLeft1;
extern pros::v5::Motor prosLeft2;
extern pros::v5::Motor prosLeft3;
extern pros::v5::Motor prosRight1;
extern pros::v5::Motor prosRight2;
extern pros::v5::Motor prosRight3;
extern pros::v5::Distance frontSensor;
extern pros::v5::Distance rightSensor;
extern pros::v5::Distance backSensor;
extern pros::v5::Distance leftSensor;
extern lemlib::V5InertialSensor imu;
extern units::Pose backSensorPos;
extern units::Pose leftSensorPos;
extern units::Pose rightSensorPos;
extern units::Pose frontSensorPos;



void calibrateParticleFilterDistanceSensorPoses() {
    // Helper for calibration: collect odometry pose and sensor readings on Enter
    std::cout << "\n=== Distance Sensor Calibration Helper ===\n";
    std::cout << "Start robot at a known pose (e.g., (48, 48, 0)). Move to several positions and press Enter at each.\n";
    std::cout << "At each step, the system will print odometry pose, expected sensor readings, actual readings, and error.\n";
    std::cout << "Capture at least 3-4 poses for best results.\n\n";

    struct Sample {
        units::Pose robotPose;
        double actualFront, actualRight, actualBack, actualLeft;
        double expectedFront, expectedRight;
    };
    std::vector<Sample> samples;

    // Hardcode initial pose to (48, 48, 0)
    // units::Pose initialPose(48_in, 48_in, 0_cDeg);
    units::Pose initialPose(48_in, 48_in, 180_cDeg);
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    std::cout << "Robot pose reset to (48, 48, 0).\n";

    bool useMockData = false; // Set to false to use real sensor readings and odometry pose
    for (int i = 0; i < 2; ++i) {
        std::cout << "Move robot to a new pose and press Enter...";
        std::cin.ignore();

        units::Pose robotPose;
        double actualFront, actualRight;
        if (useMockData) {
            // Pivot in place at 0/90 headings
            int headings[2] = {0, 90};
            robotPose = units::Pose(initialPose.x, initialPose.y, Angle(headings[i] * 100)); // 100 = centidegrees
            // Mock values for each pass (in inches)
            switch (i) {
                case 0:
                    actualFront = 48.0; actualRight = 36.0; break;
                case 1:
                    actualFront = 47.5; actualRight = 35.5; break;
                default:
                    actualFront = 48.0; actualRight = 36.0; break;
            }
        } else {
            robotPose = odometrySystem.getPose();
            actualFront = to_in(from_mm(frontSensor.get_distance()));
            actualRight = to_in(from_mm(rightSensor.get_distance()));
        }

        double expectedFront = to_in(utils::calculateExpectedDistance(robotPose, frontSensorPos));
        double expectedRight = to_in(utils::calculateExpectedDistance(robotPose, rightSensorPos));

        std::cout << "\n--- Sample " << (i+1) << " ---\n";
        std::cout << "Robot Pose: (" << to_in(robotPose.x) << ", " << to_in(robotPose.y) << ", " << to_cDeg(robotPose.orientation) << ")\n";
        // Print back and left sensor actual and expected distances
        double actualBack = to_in(from_mm(backSensor.get_distance()));
        double actualLeft = to_in(from_mm(leftSensor.get_distance()));
        double expectedBack = to_in(utils::calculateExpectedDistance(robotPose, backSensorPos));
        double expectedLeft = to_in(utils::calculateExpectedDistance(robotPose, leftSensorPos));
        std::cout << "Front Sensor: Actual=" << actualFront << " in, Expected=" << expectedFront << " in, Error=" << (actualFront-expectedFront) << " in\n";
        std::cout << "Right Sensor: Actual=" << actualRight << " in, Expected=" << expectedRight << " in, Error=" << (actualRight-expectedRight) << " in\n";
        std::cout << "Back Sensor: Actual=" << actualBack << " in, Expected=" << expectedBack << " in, Error=" << (actualBack-expectedBack) << " in\n";
        std::cout << "Left Sensor: Actual=" << actualLeft << " in, Expected=" << expectedLeft << " in, Error=" << (actualLeft-expectedLeft) << " in\n";

        // Print all sensor readings for debugging
        int backConfidence = backSensor.get_confidence();
        int leftConfidence = leftSensor.get_confidence();
        int frontConfidence = frontSensor.get_confidence();
        int rightConfidence = rightSensor.get_confidence();
        int backSize = backSensor.get_object_size();
        int leftSize = leftSensor.get_object_size();
        int frontSize = frontSensor.get_object_size();
        int rightSize = rightSensor.get_object_size();

         // If any sensor confidence is zero, skip this sample and ask user to reposition
         if (leftConfidence == 0 || backConfidence == 0) {
             std::cout << "\n[SENSOR WARNING] One or more sensors have zero confidence. Please reposition the robot and try again.\n";
             std::cout << "  Front:  Confidence=" << frontConfidence << ", Value=" << actualFront << "\n";
             std::cout << "  Right:  Confidence=" << rightConfidence << ", Value=" << actualRight << "\n";
             std::cout << "  Back:   Confidence=" << backConfidence << ", Value=" << actualBack << "\n";
             std::cout << "  Left:   Confidence=" << leftConfidence << ", Value=" << actualLeft << "\n";
             --i; // Decrement i to repeat this sample
             continue;
         }
        std::cout << "All Sensor Readings:\n";
        std::cout << "  Front:  " << actualFront << " in, Confidence=" << frontConfidence << ", Size=" << frontSize << "\n";
        std::cout << "  Right:  " << actualRight << " in, Confidence=" << rightConfidence << ", Size=" << rightSize << "\n";
        std::cout << "  Back:   " << actualBack << " in, Confidence=" << backConfidence << ", Size=" << backSize << "\n";
        std::cout << "  Left:   " << actualLeft << " in, Confidence=" << leftConfidence << ", Size=" << leftSize << "\n";

        samples.push_back({robotPose, actualFront, actualRight, actualBack, actualLeft, expectedFront, expectedRight});
    }

    std::cout << "\nCalibration samples collected.\n";

    // === Direct Sensor Pose Estimation (front and right only) ===
    Length fieldNorthWall = utils::halfHeight; // y = +halfHeight
    Length fieldEastWall = utils::halfWidth; // x = +halfWidth
    const double headingTolerance = 3.0; // degrees



    // Store perpendicular and non-perpendicular samples for each sensor
    struct SensorSample {
        double perp; // perpendicular offset (y for front/back, x for right/left)
        double dist; // measured distance
        bool valid = false;
        double angle = 0.0;
    };
    SensorSample frontPerp, frontNonPerp, rightPerp, rightNonPerp, backPerp, backNonPerp, leftPerp, leftNonPerp;

    for (const auto& s : samples) {
        std::cout << "\nSample direct geometric estimates:" << std::endl;
        double robotHeading = to_cDeg(s.robotPose.orientation);

        // --- Front sensor: facing north wall ---
        double frontRelAngle = to_cDeg(frontSensorPos.orientation);
        double frontGlobalAngle = std::fmod(robotHeading + frontRelAngle, 360.0);
        if (frontGlobalAngle < 0) frontGlobalAngle += 360.0;
        double frontAngleDiff = std::fmod(std::abs(frontGlobalAngle - 0.0), 360.0);
        if (frontAngleDiff > 180.0) frontAngleDiff = 360.0 - frontAngleDiff;
        if (!frontPerp.valid && frontAngleDiff < headingTolerance) {
            frontPerp.perp = to_in(fieldNorthWall - s.robotPose.y - from_in(s.actualFront));
            frontPerp.dist = to_in(from_mm(frontSensor.get_distance()));
            frontPerp.valid = true;
            frontPerp.angle = frontGlobalAngle;
            std::cout << "  [Front] Robot aligned with north wall. Sensor offsets (robot-relative):" << std::endl;
            std::cout << "    x = 0 in" << std::endl;
            std::cout << "    y = " << frontPerp.perp << " in" << std::endl;
        } else if (!frontNonPerp.valid && frontAngleDiff >= headingTolerance) {
            frontNonPerp.perp = to_in(fieldNorthWall - s.robotPose.y - from_in(s.actualFront));
            frontNonPerp.dist = to_in(from_mm(frontSensor.get_distance()));
            frontNonPerp.valid = true;
            frontNonPerp.angle = frontGlobalAngle;
            double sensorX = 0.0;
            if (frontNonPerp.dist > std::abs(frontNonPerp.perp)) {
                sensorX = sqrt(frontNonPerp.dist * frontNonPerp.dist - frontNonPerp.perp * frontNonPerp.perp);
            }
            std::cout << "  [Front] Non-perpendicular sample. Trig solution:" << std::endl;
            std::cout << "    x = " << sensorX << " in" << std::endl;
            std::cout << "    y = " << frontNonPerp.perp << " in" << std::endl;
        }

        // --- Right sensor: facing east wall ---
        double rightRelAngle = to_cDeg(rightSensorPos.orientation);
        double rightGlobalAngle = std::fmod(robotHeading + rightRelAngle, 360.0);
        if (rightGlobalAngle < 0) rightGlobalAngle += 360.0;
        double rightAngleDiff = std::fmod(std::abs(rightGlobalAngle - 90.0), 360.0);
        if (rightAngleDiff > 180.0) rightAngleDiff = 360.0 - rightAngleDiff;
        if (!rightPerp.valid && rightAngleDiff < headingTolerance) {
            rightPerp.perp = to_in(fieldEastWall - s.robotPose.x - from_in(s.actualRight));
            rightPerp.dist = to_in(from_mm(rightSensor.get_distance()));
            rightPerp.valid = true;
            rightPerp.angle = rightGlobalAngle;
            std::cout << "  [Right] Robot aligned with east wall. Sensor offsets (robot-relative):" << std::endl;
            std::cout << "    x = " << rightPerp.perp << " in" << std::endl;
            std::cout << "    y = 0 in" << std::endl;
        } else if (!rightNonPerp.valid && rightAngleDiff >= headingTolerance) {
            rightNonPerp.perp = to_in(fieldEastWall - s.robotPose.x - from_in(s.actualRight));
            rightNonPerp.dist = to_in(from_mm(rightSensor.get_distance()));
            rightNonPerp.valid = true;
            rightNonPerp.angle = rightGlobalAngle;
            double sensorY = 0.0;
            if (rightNonPerp.dist > std::abs(rightNonPerp.perp)) {
                sensorY = sqrt(rightNonPerp.dist * rightNonPerp.dist - rightNonPerp.perp * rightNonPerp.perp);
            }
            std::cout << "  [Right] Non-perpendicular sample. Trig solution:" << std::endl;
            std::cout << "    x = " << rightNonPerp.perp << " in" << std::endl;
            std::cout << "    y = " << sensorY << " in" << std::endl;
        }

        // --- Back sensor: facing south wall ---
        double backRelAngle = to_cDeg(backSensorPos.orientation);
        double backGlobalAngle = std::fmod(robotHeading + backRelAngle, 360.0);
        if (backGlobalAngle < 0) backGlobalAngle += 360.0;
        double backAngleDiff = std::fmod(std::abs(backGlobalAngle - 180.0), 360.0);
        if (backAngleDiff > 180.0) backAngleDiff = 360.0 - backAngleDiff;
        if (!backPerp.valid && backAngleDiff < headingTolerance) {
            backPerp.perp = to_in(-fieldNorthWall - s.robotPose.y + from_in(s.actualBack)); // South wall is -halfHeight
            backPerp.dist = to_in(from_mm(backSensor.get_distance()));
            backPerp.valid = true;
            backPerp.angle = backGlobalAngle;
            if (std::abs(backPerp.perp) > 24.0) {
                std::cout << "  [Back] WARNING: Impossible sensor offset (y = " << backPerp.perp << "). Check alignment and measurement." << std::endl;
            }
            std::cout << "  [Back] Robot aligned with south wall. Sensor offsets (robot-relative):" << std::endl;
            std::cout << "    x = 0 in" << std::endl;
            std::cout << "    y = " << backPerp.perp << " in" << std::endl;
        } else if (!backNonPerp.valid && backAngleDiff >= headingTolerance) {
            backNonPerp.perp = to_in(-fieldNorthWall - s.robotPose.y + from_in(s.actualBack));
            backNonPerp.dist = to_in(from_mm(backSensor.get_distance()));
            backNonPerp.valid = true;
            backNonPerp.angle = backGlobalAngle;
            double sensorX = 0.0;
            if (backNonPerp.dist > std::abs(backNonPerp.perp)) {
                sensorX = sqrt(backNonPerp.dist * backNonPerp.dist - backNonPerp.perp * backNonPerp.perp);
            }
            if (std::abs(backNonPerp.perp) > 24.0) {
                std::cout << "  [Back] WARNING: Impossible sensor offset (y = " << backNonPerp.perp << "). Check alignment and measurement." << std::endl;
            }
            std::cout << "  [Back] Non-perpendicular sample. Trig solution:" << std::endl;
            std::cout << "    x = " << sensorX << " in" << std::endl;
            std::cout << "    y = " << backNonPerp.perp << " in" << std::endl;
        }

        // --- Left sensor: facing west wall ---
        double leftRelAngle = to_cDeg(leftSensorPos.orientation);
        double leftGlobalAngle = std::fmod(robotHeading + leftRelAngle, 360.0);
        if (leftGlobalAngle < 0) leftGlobalAngle += 360.0;
        double leftAngleDiff = std::fmod(std::abs(leftGlobalAngle - 270.0), 360.0);
        if (leftAngleDiff > 180.0) leftAngleDiff = 360.0 - leftAngleDiff;
        if (!leftPerp.valid && leftAngleDiff < headingTolerance) {
            leftPerp.perp = to_in(-fieldEastWall - s.robotPose.x + from_in(s.actualLeft)); // West wall is -halfWidth
            leftPerp.dist = to_in(from_mm(leftSensor.get_distance()));
            leftPerp.valid = true;
            leftPerp.angle = leftGlobalAngle;
            if (std::abs(leftPerp.perp) > 24.0) {
                std::cout << "  [Left] WARNING: Impossible sensor offset (x = " << leftPerp.perp << "). Check alignment and measurement." << std::endl;
            }
            std::cout << "  [Left] Robot aligned with west wall. Sensor offsets (robot-relative):" << std::endl;
            std::cout << "    x = " << leftPerp.perp << " in" << std::endl;
            std::cout << "    y = 0 in" << std::endl;
        } else if (!leftNonPerp.valid && leftAngleDiff >= headingTolerance) {
            leftNonPerp.perp = to_in(-fieldEastWall - s.robotPose.x + from_in(s.actualLeft));
            leftNonPerp.dist = to_in(from_mm(leftSensor.get_distance()));
            leftNonPerp.valid = true;
            leftNonPerp.angle = leftGlobalAngle;
            double sensorY = 0.0;
            if (leftNonPerp.dist > std::abs(leftNonPerp.perp)) {
                sensorY = sqrt(leftNonPerp.dist * leftNonPerp.dist - leftNonPerp.perp * leftNonPerp.perp);
            }
            if (std::abs(leftNonPerp.perp) > 24.0) {
                std::cout << "  [Left] WARNING: Impossible sensor offset (x = " << leftNonPerp.perp << "). Check alignment and measurement." << std::endl;
            }
            std::cout << "  [Left] Non-perpendicular sample. Trig solution:" << std::endl;
            std::cout << "    x = " << leftNonPerp.perp << " in" << std::endl;
            std::cout << "    y = " << sensorY << " in" << std::endl;
        }
    }

    std::cout << "\nIf you align the robot and sensor to face a wall, the above values give you the sensor's position directly.\n";
    std::cout << "Otherwise, use the average error method for more general cases.\n";

    // === Least squares fit for sensor pose (robot-relative, inches) ===
    auto clampInches = [](double value) {
        return std::max(-9.0, std::min(9.0, value));
    };
    // For each sensor, fit y (front) and x (right) using all samples
    double sumFrontY = 0.0, sumRightX = 0.0;
    for (const auto& s : samples) {
        Length frontY = fieldNorthWall - s.robotPose.y - from_in(s.actualFront);
        Length rightX = fieldEastWall - s.robotPose.x - from_in(s.actualRight);
        sumFrontY += to_in(frontY);
        sumRightX += to_in(rightX);
    }
    // Output direct geometric/trig solution for all sensors, clamped to [-9, 9] inches
    std::cout << "\n--- Suggested Sensor Relative Poses (robot-relative, inches, direct geometric solution) ---\n";
auto clamp = [](double v) { return std::max(-10.0, std::min(10.0, v)); };
    // Front
    std::cout << "  Front sensor pose: (x = ";
    if (frontPerp.valid && frontNonPerp.valid) {
        double sensorX = 0.0;
        if (frontNonPerp.dist > std::abs(frontNonPerp.perp)) {
            sensorX = sqrt(frontNonPerp.dist * frontNonPerp.dist - frontNonPerp.perp * frontNonPerp.perp);
        }
        std::cout << clamp(sensorX);
    } else {
        std::cout << "?";
    }
    std::cout << ", y = ";
    if (frontPerp.valid) std::cout << clamp(frontPerp.perp); else std::cout << "?";
    std::cout << ", orientation = " << to_cDeg(frontSensorPos.orientation) << ")\n";

    // Right
    std::cout << "  Right sensor pose: (x = ";
    if (rightPerp.valid) std::cout << clamp(rightPerp.perp); else std::cout << "?";
    std::cout << ", y = ";
    if (rightPerp.valid && rightNonPerp.valid) {
        double sensorY = 0.0;
        if (rightNonPerp.dist > std::abs(rightNonPerp.perp)) {
            sensorY = sqrt(rightNonPerp.dist * rightNonPerp.dist - rightNonPerp.perp * rightNonPerp.perp);
        }
        std::cout << clamp(sensorY);
    } else {
        std::cout << "?";
    }
    std::cout << ", orientation = " << to_cDeg(rightSensorPos.orientation) << ")\n";

    // Back
    std::cout << "  Back sensor pose: (x = ";
    if (backPerp.valid && backNonPerp.valid) {
        double sensorX = 0.0;
        if (backNonPerp.dist > std::abs(backNonPerp.perp)) {
            sensorX = sqrt(backNonPerp.dist * backNonPerp.dist - backNonPerp.perp * backNonPerp.perp);
        }
        std::cout << clamp(sensorX);
    } else {
        std::cout << "?";
    }
    std::cout << ", y = ";
    if (backPerp.valid) std::cout << clamp(backPerp.perp); else std::cout << "?";
    std::cout << ", orientation = " << to_cDeg(backSensorPos.orientation) << ")\n";

    // Left
    std::cout << "  Left sensor pose: (x = ";
    if (leftPerp.valid) std::cout << clamp(leftPerp.perp); else std::cout << "?";
    std::cout << ", y = ";
    if (leftPerp.valid && leftNonPerp.valid) {
        double sensorY = 0.0;
        if (leftNonPerp.dist > std::abs(leftNonPerp.perp)) {
            sensorY = sqrt(leftNonPerp.dist * leftNonPerp.dist - leftNonPerp.perp * leftNonPerp.perp);
        }
        std::cout << clamp(sensorY);
    } else {
        std::cout << "?";
    }
    std::cout << ", orientation = " << to_cDeg(leftSensorPos.orientation) << ")\n";
}

void runParticleFilterTest() {    
    double prosLeft1Pos = prosLeft1.get_position();
    double prosLeft2Pos = prosLeft2.get_position();
    double prosLeft3Pos = prosLeft3.get_position();
    double prosRight1Pos = prosRight1.get_position();
    double prosRight2Pos = prosRight2.get_position();
    double prosRight3Pos = prosRight3.get_position();
    double prosLeftMotorsPos = prosLeftMotors.get_position();
    double prosRightMotorsPos = prosRightMotors.get_position();
    Angle leftMotorsAngle = leftMotors.getAngle();
    Angle rightMotorsAngle = rightMotors.getAngle();

    // Initialize test - reset odometry and particle filter to a known pose
    units::Pose initialPose(48_in, 48_in, 0_cDeg);        // Blue Right Side facing North
    //units::Pose initialPose(-48_in, 48_in, 60_cDeg);    // Red Left Side 30 degrees
    //units::Pose initialPose(48_in, 48_in, -180_cDeg);   // Blue Right Side
    odometrySystem.resetPose(initialPose);
    particleFilter.resetPose(initialPose);
    odometrySystem.start();
    particleFilter.start();

    // Enable performance logging with 1-second intervals
    particleFilter.enablePerformanceLogging(true, 1000);
    
    // Enable visualization
    particleFilter.enableVisualization(true, &fieldView, 15);
    
    // Setup diagnostics view with sensors and motors
    pros::v5::Motor* motors[] = {&prosLeft1, &prosLeft2, &prosLeft3, &prosRight1, &prosRight2, &prosRight3};
    const char* motorNames[] = {"Left 1", "Left 2", "Left 3", "Right 1", "Right 2", "Right 3"};
    
    pros::v5::Distance* distanceSensors[] = {&frontSensor, &rightSensor, &backSensor, &leftSensor};
    const char* sensorNames[] = {"Front", "Right", "Back", "Left"};
    
    Length initialBackDistance = from_mm(backSensor.get_distance());
    Length initialLeftDistance = from_mm(leftSensor.get_distance());
    int initialBackConfidence = backSensor.get_confidence();
    int initialLeftConfidence = leftSensor.get_confidence();
    
    std::cout << "\n==== STARTING PARTICLE FILTER TEST ====\n";
    std::cout << "Initial pose: (" << to_in(initialPose.x) << " in, " 
              << to_in(initialPose.y) << " in, " << to_cDeg(initialPose.orientation) << " deg)\n";
    
    Time startTime = from_msec(pros::millis());
    Time testDuration = 15_sec;
    //Time testDuration = 5_sec;
    
    while (from_msec(pros::millis()) - startTime < testDuration)
    {
        // Define motor arrays outside the loop to avoid potential stack issues
        static pros::v5::Motor* leftMotors[3] = {&prosLeft1, &prosLeft2, &prosLeft3};
        static pros::v5::Motor* rightMotors[3] = {&prosRight1, &prosRight2, &prosRight3};
        
        // Update diagnostic view with motor groups
        // diagnosticsView.updateMotors(leftMotors, rightMotors, 3);
        diagnosticsView.updateIMU(imu.getRotation());
        
        // Print current positions
        units::Pose odomPose = odometrySystem.getPose();
        units::Pose pfPose = particleFilter.getPose();
        std::cout << "Time: " << (from_msec(pros::millis()) - startTime) << " s\n";
        std::cout << "  Odometry: (" << to_in(odomPose.x) << " in, " 
                  << to_in(odomPose.y) << " in, " << to_cDeg(odomPose.orientation) << " deg)\n";
        std::cout << "  Particle Filter: (" << to_in(pfPose.x) << " in, " 
                 << to_in(pfPose.y) << " in, " << to_cDeg(pfPose.orientation) << " deg)\n";
        Angle odomToPfDiff = pfPose.orientation - odomPose.orientation;
        odomToPfDiff = units::constrainAngle180(odomToPfDiff);
        std::cout << "  Difference: (" 
                  << to_in(pfPose.x - odomPose.x) << " in, " 
                  << to_in(pfPose.y - odomPose.y) << " in, "
                  << to_stDeg(odomToPfDiff) << " deg)\n\n";
        
        pros::delay(1000);
    }
    // Print final positions
    units::Pose finalOdomPose = odometrySystem.getPose();
    units::Pose finalPfPose = particleFilter.getPose();

    particleFilter.stop();
    odometrySystem.stop();

    Angle odomToPfDiff = finalPfPose.orientation - finalOdomPose.orientation;
    odomToPfDiff = units::constrainAngle180(odomToPfDiff);
    std::cout << "\n==== PARTICLE FILTER TEST COMPLETED ====\n";
    std::cout << "Final Difference: (" 
              << to_in(finalPfPose.x - finalOdomPose.x) << " in, " 
              << to_in(finalPfPose.y - finalOdomPose.y) << " in, "
              << to_stDeg(odomToPfDiff) << " deg)\n";
    std::cout << "Return-to-start error (Odometry): " 
              << to_in(units::sqrt(utils::poseDistanceSquared(initialPose, finalOdomPose))) << " in\n";
    std::cout << "Return-to-start error (Particle Filter): " 
              << to_in(units::sqrt(utils::poseDistanceSquared(initialPose, finalPfPose))) << " in\n";
    std::cout << "=======================================\n\n";

    /*
    std::cout << "Start ProsLeft1: " << prosLeft1Pos << "\n";
    std::cout << "Start ProsLeft2: " << prosLeft2Pos << "\n";
    std::cout << "Start ProsLeft3: " << prosLeft3Pos << "\n";
    std::cout << "Start ProsRight1: " << prosRight1Pos << "\n";
    std::cout << "Start ProsRight2: " << prosRight2Pos << "\n";
    std::cout << "Start ProsRight3: " << prosRight3Pos << "\n";
    std::cout << "Start ProsLeftMotors: " << prosLeftMotorsPos << "\n";
    std::cout << "Start ProsRightMotors: " << prosRightMotorsPos << "\n";
    std::cout << "Start LeftMotors: " << to_stDeg(leftMotorsAngle) << "\n";
    std::cout << "Start RightMotors: " << to_stDeg(rightMotorsAngle) << "\n";
    */
    prosLeft1Pos = prosLeft1.get_position();
    prosLeft2Pos = prosLeft2.get_position();
    prosLeft3Pos = prosLeft3.get_position();
    prosRight1Pos = prosRight1.get_position();
    prosRight2Pos = prosRight2.get_position();
    prosRight3Pos = prosRight3.get_position();
    prosLeftMotorsPos = prosLeftMotors.get_position();
    prosRightMotorsPos = prosRightMotors.get_position();
    leftMotorsAngle = leftMotors.getAngle();
    rightMotorsAngle = rightMotors.getAngle();
    /*
    std::cout << "End ProsLeft1: " << prosLeft1Pos << "\n";
    std::cout << "End ProsLeft2: " << prosLeft2Pos << "\n";
    std::cout << "End ProsLeft3: " << prosLeft3Pos << "\n";
    std::cout << "End ProsRight1: " << prosRight1Pos << "\n";
    std::cout << "End ProsRight2: " << prosRight2Pos << "\n";
    std::cout << "End ProsRight3: " << prosRight3Pos << "\n";
    std::cout << "End ProsLeftMotors: " << prosLeftMotorsPos << "\n";
    std::cout << "End ProsRightMotors: " << prosRightMotorsPos << "\n";
    std::cout << "End LeftMotors: " << to_stDeg(leftMotorsAngle) << "\n";
    std::cout << "End RightMotors: " << to_stDeg(rightMotorsAngle) << "\n";
    */
    Length endBackDistance = from_mm(backSensor.get_distance());
    Length endLeftDistance = from_mm(leftSensor.get_distance());
    int endBackConfidence = backSensor.get_confidence();
    int endLeftConfidence = leftSensor.get_confidence();

    /*
    std::cout << "Initial Back Sensor: " << to_in(initialBackDistance) << " in\n";
    std::cout << "Initial Left Sensor: " << to_in(initialLeftDistance) << " in\n";
    std::cout << "End Back Sensor: " << to_in(endBackDistance) << " in\n";
    std::cout << "End Left Sensor: " << to_in(endLeftDistance) << " in\n";
    std::cout << "Back Sensor Delta: " << to_in(endBackDistance - initialBackDistance) << " in\n";
    std::cout << "Left Sensor Delta: " << to_in(endLeftDistance - initialLeftDistance) << " in\n";
    std::cout << "Start Back Sensor Confidence: " << initialBackConfidence << "\n";
    std::cout << "End Back Sensor Confidence: " << endBackConfidence << "\n";
    std::cout << "Start Left Sensor Confidence: " << initialLeftConfidence << "\n";
    std::cout << "End Left Sensor Confidence: " << endLeftConfidence << "\n";
    int backSize = backSensor.get_object_size();
    int frontSize = frontSensor.get_object_size();
    int frontConfidence = frontSensor.get_confidence();
    int frontDist = frontSensor.get_distance();
    std::cout << "Front Sensor Size: " << frontSize << "\n";
    std::cout << "Back Sensor Size: " << backSize << "\n";
    std::cout << "Front Sensor Confidence: " << frontConfidence << "\n";
    std::cout << "Front Sensor Distance: " << frontDist << "\n";
    */
}

void testPoseTransformAndExpectedDistance() {
    // Test sensorPoseToGlobal with robot at (48, 48, 0)
    // Use the global sensor pose instances from main.cpp


    // Test case 1: robot at (48, 48, 0)
    units::Pose robotPose1(48_in, 48_in, 0_cDeg);
    units::Pose globalBack1 = utils::sensorPoseToGlobal(robotPose1, backSensorPos);
    units::Pose globalLeft1 = utils::sensorPoseToGlobal(robotPose1, leftSensorPos);
    units::Pose globalRight1 = utils::sensorPoseToGlobal(robotPose1, rightSensorPos);
    units::Pose globalFront1 = utils::sensorPoseToGlobal(robotPose1, frontSensorPos);

    std::cout << "\n--- Robot Pose: (" << to_in(robotPose1.x) << ", " << to_in(robotPose1.y) << ", " << to_cDeg(robotPose1.orientation) << ") ---\n";
    std::cout << "Back Sensor Pose (robot-relative): (" << to_in(backSensorPos.x) << ", " << to_in(backSensorPos.y) << ", " << to_cDeg(backSensorPos.orientation) << ")\n";
    std::cout << "Left Sensor Pose (robot-relative): (" << to_in(leftSensorPos.x) << ", " << to_in(leftSensorPos.y) << ", " << to_cDeg(leftSensorPos.orientation) << ")\n";
    std::cout << "Right Sensor Pose (robot-relative): (" << to_in(rightSensorPos.x) << ", " << to_in(rightSensorPos.y) << ", " << to_cDeg(rightSensorPos.orientation) << ")\n";
    std::cout << "Front Sensor Pose (robot-relative): (" << to_in(frontSensorPos.x) << ", " << to_in(frontSensorPos.y) << ", " << to_cDeg(frontSensorPos.orientation) << ")\n";
    std::cout << "Back Sensor Global: (" << to_in(globalBack1.x) << ", " << to_in(globalBack1.y) << ", " << to_cDeg(globalBack1.orientation) << ")\n";
    std::cout << "Left Sensor Global: (" << to_in(globalLeft1.x) << ", " << to_in(globalLeft1.y) << ", " << to_cDeg(globalLeft1.orientation) << ")\n";
    std::cout << "Right Sensor Global: (" << to_in(globalRight1.x) << ", " << to_in(globalRight1.y) << ", " << to_cDeg(globalRight1.orientation) << ")\n";
    std::cout << "Front Sensor Global: (" << to_in(globalFront1.x) << ", " << to_in(globalFront1.y) << ", " << to_cDeg(globalFront1.orientation) << ")\n";

    // Print expected distance readings for test case 1
    std::cout << "Expected Back Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose1, backSensorPos)) << " in\n";
    std::cout << "Expected Left Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose1, leftSensorPos)) << " in\n";
    std::cout << "Expected Right Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose1, rightSensorPos)) << " in\n";
    std::cout << "Expected Front Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose1, frontSensorPos)) << " in\n";

    // Test case 2: robot at (48, 48, 270)
    units::Pose robotPose2(48_in, 48_in, 270_cDeg);
    units::Pose globalBack2 = utils::sensorPoseToGlobal(robotPose2, backSensorPos);
    units::Pose globalLeft2 = utils::sensorPoseToGlobal(robotPose2, leftSensorPos);
    units::Pose globalRight2 = utils::sensorPoseToGlobal(robotPose2, rightSensorPos);
    units::Pose globalFront2 = utils::sensorPoseToGlobal(robotPose2, frontSensorPos);

    std::cout << "\n--- Robot Pose: (" << to_in(robotPose2.x) << ", " << to_in(robotPose2.y) << ", " << to_cDeg(robotPose2.orientation) << ") ---\n";
    std::cout << "Back Sensor Pose (robot-relative): (" << to_in(backSensorPos.x) << ", " << to_in(backSensorPos.y) << ", " << to_cDeg(backSensorPos.orientation) << ")\n";
    std::cout << "Left Sensor Pose (robot-relative): (" << to_in(leftSensorPos.x) << ", " << to_in(leftSensorPos.y) << ", " << to_cDeg(leftSensorPos.orientation) << ")\n";
    std::cout << "Right Sensor Pose (robot-relative): (" << to_in(rightSensorPos.x) << ", " << to_in(rightSensorPos.y) << ", " << to_cDeg(rightSensorPos.orientation) << ")\n";
    std::cout << "Front Sensor Pose (robot-relative): (" << to_in(frontSensorPos.x) << ", " << to_in(frontSensorPos.y) << ", " << to_cDeg(frontSensorPos.orientation) << ")\n";
    std::cout << "Back Sensor Global: (" << to_in(globalBack2.x) << ", " << to_in(globalBack2.y) << ", " << to_cDeg(globalBack2.orientation) << ")\n";
    std::cout << "Left Sensor Global: (" << to_in(globalLeft2.x) << ", " << to_in(globalLeft2.y) << ", " << to_cDeg(globalLeft2.orientation) << ")\n";
    std::cout << "Right Sensor Global: (" << to_in(globalRight2.x) << ", " << to_in(globalRight2.y) << ", " << to_cDeg(globalRight2.orientation) << ")\n";
    std::cout << "Front Sensor Global: (" << to_in(globalFront2.x) << ", " << to_in(globalFront2.y) << ", " << to_cDeg(globalFront2.orientation) << ")\n";

    // Print expected distance readings for test case 2
    std::cout << "Expected Back Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose2, backSensorPos)) << " in\n";
    std::cout << "Expected Left Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose2, leftSensorPos)) << " in\n";
    std::cout << "Expected Right Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose2, rightSensorPos)) << " in\n";
    std::cout << "Expected Front Sensor Distance: " << to_in(utils::calculateExpectedDistance(robotPose2, frontSensorPos)) << " in\n";
}

// Helper function to solve for missing sensor coordinate (e.g., front.x, right.y, etc.)
double solveSensorOffset(
    double robotX,
    double robotY,
    double robotOrientationDeg,
    double knownSensorY,
    double sensorOrientationDeg,
    double measuredDistance,
    double wallY
) {
    // Convert degrees to radians
    double theta = robotOrientationDeg * M_PI / 180.0;
    // For most cases, sensor orientation matches robot orientation
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    // The global Y position of the sensor
    // globalY = robotY + sensorX * sin(theta) + knownSensorY * cos(theta)
    // measuredDistance = wallY - globalY
    // Rearranged:
    // globalY = wallY - measuredDistance
    // robotY + sensorX * sin(theta) + knownSensorY * cos(theta) = wallY - measuredDistance
    // sensorX = (wallY - measuredDistance - robotY - knownSensorY * cosTheta) / sinTheta
    if (fabs(sinTheta) < 1e-6) {
        std::cout << "Cannot solve for sensorX: sin(theta) is zero (robot facing wall directly)" << std::endl;
        return 0.0;
    }
    return (wallY - measuredDistance - robotY - knownSensorY * cosTheta) / sinTheta;
}

// Example usage inside calibrateParticleFilterDistanceSensorPoses:
// After collecting a sample, you can call:
// double solvedFrontX = solveSensorOffset(
//     to_in(robotPose.x),
//     to_in(robotPose.y),
//     to_cDeg(robotPose.orientation),
//     to_in(frontSensorPos.y),
//     to_cDeg(frontSensorPos.orientation),
//     actualFront,
//     to_in(utils::halfHeight)
// );
// std::cout << "Solved front.x: " << solvedFrontX << " in" << std::endl;

// Simple sensor verification function
void calibrateLeftYAndBackXAtRotatedHeading() {
    std::cout << "\n=== Simple Sensor Position Verification ===\n";
    std::cout << "Current sensor positions in main.cpp:\n";
    std::cout << "  Back:  (0, -7.64, 180°) - sensor 7.64\" behind robot center\n";
    std::cout << "  Left:  (-6.0, 0, 270°) - sensor 6.0\" left of robot center\n";
    std::cout << "  Right: (7.15, 0, 90°) - sensor 7.15\" right of robot center\n";
    std::cout << "  Front: (0, 9.16, 0°) - sensor 9.16\" in front of robot center\n\n";
    
    std::cout << "This function will test if your current sensor positions are accurate.\n";
    std::cout << "Place robot at (48, 48) and press Enter to start verification...\n";
    
    std::string line;
    std::getline(std::cin, line);

    // Reset pose and start odometry
    units::Pose testPose(48_in, 48_in, 0_cDeg);
    odometrySystem.resetPose(testPose);
    odometrySystem.start();
    
    std::cout << "Robot reset to (48, 48, 0°)\n";
    std::cout << "Take 3 samples by rotating to different headings.\n\n";

    for (int i = 0; i < 3; ++i) {
        std::cout << "=== Sample " << (i+1) << " ===\n";
        std::cout << "Rotate robot and press Enter...";
        std::getline(std::cin, line);

        // Get current readings
        units::Pose robotPose = odometrySystem.getPose();
        double actualLeft = to_in(from_mm(leftSensor.get_distance()));
        double actualBack = to_in(from_mm(backSensor.get_distance()));
        int leftConf = leftSensor.get_confidence();
        int backConf = backSensor.get_confidence();

        // Calculate expected distances with current sensor positions
        double expectedLeft = to_in(utils::calculateExpectedDistance(robotPose, leftSensorPos));
        double expectedBack = to_in(utils::calculateExpectedDistance(robotPose, backSensorPos));
        
        std::cout << "Robot: (" << to_in(robotPose.x) << ", " << to_in(robotPose.y) 
                  << ", " << to_cDeg(robotPose.orientation) << "°)\n";
        std::cout << "Left:  actual=" << actualLeft << "\" expected=" << expectedLeft 
                  << "\" error=" << std::abs(actualLeft - expectedLeft) << "\" conf=" << leftConf << "\n";
        std::cout << "Back:  actual=" << actualBack << "\" expected=" << expectedBack 
                  << "\" error=" << std::abs(actualBack - expectedBack) << "\" conf=" << backConf << "\n";
                  
        // Simple assessment
        double leftError = std::abs(actualLeft - expectedLeft);
        double backError = std::abs(actualBack - expectedBack);
        if (leftError < 2.0) std::cout << "Left sensor position looks good!\n";
        else std::cout << "Left sensor position might need adjustment.\n";
        if (backError < 2.0) std::cout << "Back sensor position looks good!\n";
        else std::cout << "Back sensor position might need adjustment.\n";
        std::cout << std::endl;
    }

    odometrySystem.stop();
    std::cout << "Verification complete. If errors are consistently > 2\", consider measuring\n";
    std::cout << "your sensor positions more precisely and updating main.cpp.\n";
}

