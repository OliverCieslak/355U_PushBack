// Includes
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include "auton/ParticleFilterTest.hpp"
#include "main.h"

#include "units/Pose.hpp"
#include "utils/DistanceUtils.hpp"
#include "odometry/SkidSteerOdometry.hpp"
#include "localization/ParticleFilter.hpp"
#include "viz/FieldView.hpp"
#include "viz/DiagnosticsView.hpp"
#include "tuning/CharacterizationView.hpp"
#include "control/PIDDriveController.hpp"
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
extern control::PIDDriveController pidDriveController; // Use the PID controller to turn 20° each step

void calibrateParticleFilterDistanceSensorPoses() {
    // ========================= OVERVIEW =========================
    // Goal: Find the true x/y location of each distance sensor on the robot.
    // Plan:
    // 1) Start at a known field pose: (48 in, 48 in, 0°).
    // 2) For 36 steps (36 × 10° = full 360°):
    //    - Every second, record the odometry pose and 3 quick readings from each sensor
    //      (one immediately, then +100 ms, then +200 ms). Also record confidence.
    //    - Turn the robot by +10° using the PID drive controller.
    // 3) After collecting data, keep each sensor's facing direction (orientation) fixed,
    //    but adjust its x/y within ±1 inch to best fit all the measurements.
    // ============================================================

    // Helper structs that keep the code easy to understand
    struct Reading {
        double inches;   // distance in inches (NaN if invalid)
        int confidence;  // 0..63
    };
    struct SensorTriple {
        // Three readings from the same pose for one sensor (reduced from 5)
        Reading r[3];
        double weightedAvgInches; // confidence-weighted average of valid readings
        int totalConfidence;      // sum of confidences for the 3 readings (valid ones)
    };
    struct PoseSample {
        units::Pose odomPose;   // robot pose from odometry
        SensorTriple front;
        SensorTriple right;
        SensorTriple back;
        SensorTriple left;
    };

    // 1) Calculate true robot position using initial sensor readings
    std::cout << "\n=== Distance Sensor Pose Calibration ===\n";
    std::cout << "Determining true robot position from sensor readings...\n";
    
    // Take initial sensor readings to determine actual robot position
    // Assume robot is facing north (0°) and roughly centered
    int frontMm = frontSensor.get_distance();
    int rightMm = rightSensor.get_distance();
    int frontConf = frontSensor.get_confidence();
    int rightConf = rightSensor.get_confidence();
    
    std::cout << "Initial sensor readings:\n";
    std::cout << "  Front: " << frontMm << "mm (conf=" << frontConf << "/63)\n";
    std::cout << "  Right: " << rightMm << "mm (conf=" << rightConf << "/63)\n";
    
    // Calculate robot position assuming:
    // - Robot is facing north (0°)
    // - Field boundaries are at ±72" (144" total width/height)
    // - Front sensor faces north, right sensor faces east
    Length frontDist = from_mm(frontMm);
    Length rightDist = from_mm(rightMm);
    
    // Robot center position calculation:
    // If robot faces north (0°) and front sensor reads distance to north wall:
    // - Robot Y = (distance to north wall) + (robot center to north wall)
    // - Robot Y = frontDistance + frontSensorOffset.y  [sensor is ahead of center]
    // If robot faces north (0°) and right sensor reads distance to east wall:
    // - Robot X = (distance to east wall) + (robot center to east wall) 
    // - Robot X = rightDistance + rightSensorOffset.x  [sensor is right of center]
    
    // For a 144" x 144" field with origin at center (-72" to +72" in both directions):
    // - North wall is at Y = +72"
    // - East wall is at X = +72"
    // So: Robot Y = +72" - (frontDistance + frontSensorOffset.y)
    //     Robot X = +72" - (rightDistance + rightSensorOffset.x)
    
    Length robotY = 72_in - (frontDist + frontSensorPos.y);  
    Length robotX = 72_in - (rightDist + rightSensorPos.x);
    
    units::Pose startPose(robotX, robotY, from_cDeg(0.0));
    
    std::cout << "Calculated true robot position: (" << to_in(robotX) << " in, " << to_in(robotY) << " in, 0 deg)\n";
    std::cout << "  Front sensor: " << to_in(frontDist) << "\" + sensor pos.y=" << to_in(frontSensorPos.y) << "\" = " << to_in(frontDist + frontSensorPos.y) << "\" from north wall\n";
    std::cout << "  Right sensor: " << to_in(rightDist) << "\" + sensor pos.x=" << to_in(rightSensorPos.x) << "\" = " << to_in(rightDist + rightSensorPos.x) << "\" from east wall\n";
    std::cout << "  Robot center: (" << to_in(robotX) << ", " << to_in(robotY) << ") = (72 - " << to_in(rightDist + rightSensorPos.x) << ", 72 - " << to_in(frontDist + frontSensorPos.y) << ")\n";
    
    // Verify readings make sense
    if (frontConf < 10 || rightConf < 10) {
        std::cout << "WARNING: Low sensor confidence. Position may be inaccurate.\n";
    }
    if (to_in(robotX) < 10 || to_in(robotX) > 134 || to_in(robotY) < 10 || to_in(robotY) > 134) {
        std::cout << "WARNING: Calculated position seems outside reasonable field bounds.\n";
    }
    
    odometrySystem.resetPose(startPose);
    odometrySystem.start();

    std::cout << "Collecting data every second, then turning +20 deg each step...\n";

    // Helper functions for taking sensor readings
    auto getDistanceReading = [](pros::v5::Distance& ds, const std::string& sensorName, const units::Pose& robotPose, const units::Pose& sensorPose) -> Reading {
        int mm = ds.get_distance();
        int conf = ds.get_confidence();
        Length d = from_mm(mm);

        // Calculate expected distance for comparison
        Length expectedDist = utils::calculateExpectedDistance(robotPose, sensorPose);
        double expectedInches = to_in(expectedDist);

        // Require confidence at least 40/63 (~63%) to accept a reading as 'good'
        if (conf < 40 || !utils::isValidDistanceSensorReading(d)) {
            std::cout << sensorName << ": REJECTED (conf=" << conf << "/63, dist=" << to_in(d) << " in) vs expected " << expectedInches << " in" << std::endl;
            return {std::numeric_limits<double>::quiet_NaN(), 0};
        }

        // Additional field-specific validation - readings beyond field diagonal are impossible
        double inches = to_in(d);
        if (inches > 170.0) {  // Field diagonal is ~170 inches, anything beyond is clearly wrong
            std::cout << sensorName << ": REJECTED (too far: " << inches << " in) vs expected " << expectedInches << " in" << std::endl;
            return {std::numeric_limits<double>::quiet_NaN(), 0};
        }

        // Reject readings that are suspiciously close to the 393.66" error value
        if (std::abs(inches - 393.66) < 1.0) {
            std::cout << sensorName << ": REJECTED (error value: " << inches << " in) vs expected " << expectedInches << " in" << std::endl;
            return {std::numeric_limits<double>::quiet_NaN(), 0};
        }

        // Print detailed comparison between actual and expected
        double error = std::abs(inches - expectedInches);
        std::cout << sensorName << ": " << inches << " in (conf=" << conf << "/63) vs expected " << expectedInches << " in (error=" << error << " in)" << std::endl;

        return {inches, conf};
    };

    auto takeSensorTriple = [&](pros::v5::Distance& ds, const std::string& sensorName, const units::Pose& robotPose, const units::Pose& sensorPose) -> SensorTriple {
        SensorTriple t{};
        std::cout << "  " << sensorName << " sensor readings:" << std::endl;
        // Take 3 readings with 50ms spacing for better noise filtering
        for (int i = 0; i < 3; ++i) {
            t.r[i] = getDistanceReading(ds, sensorName, robotPose, sensorPose);
            if (i < 2) pros::delay(50); // 50ms between readings (100ms total)
        }

        // Compute confidence-weighted average of the valid readings
        double num = 0.0;
        double den = 0.0;
        int confSum = 0;
        for (int i = 0; i < 3; ++i) {
            if (!std::isnan(t.r[i].inches) && t.r[i].confidence > 0) {
                num += t.r[i].inches * t.r[i].confidence;
                den += t.r[i].confidence;
                confSum += t.r[i].confidence;
            }
        }
        if (den > 0.0) {
            t.weightedAvgInches = num / den;
        } else {
            t.weightedAvgInches = std::numeric_limits<double>::quiet_NaN();
        }
        t.totalConfidence = confSum;
        return t;
    };

    auto takePoseSample = [&]() -> PoseSample {
        PoseSample s{};
        s.odomPose = odometrySystem.getPose();
        std::cout << "Robot pose: (" << to_in(s.odomPose.x) << ", " << to_in(s.odomPose.y) << ", " << to_cDeg(s.odomPose.orientation) << "°)" << std::endl;
        
        // Reordered: Start with LEFT sensor first to test timing theory
        s.left  = takeSensorTriple(leftSensor,  "LEFT",  s.odomPose, leftSensorPos);
        s.back  = takeSensorTriple(backSensor,  "BACK",  s.odomPose, backSensorPos);
        s.right = takeSensorTriple(rightSensor, "RIGHT", s.odomPose, rightSensorPos);
        s.front = takeSensorTriple(frontSensor, "FRONT", s.odomPose, frontSensorPos);
        return s;
    };

    std::vector<PoseSample> samples;
    samples.reserve(37);

    // Take initial sample at 0°
    std::cout << "\n[Step 1/37] Sampling sensors..." << std::endl;
    PoseSample s = takePoseSample();
    samples.push_back(s);

    // 2) Loop 36 times to cover full 360° (after 36 × 10° turns)
    for (int i = 0; i < 36; ++i) {
        // Brief delay before turning (no need for full second anymore)
        pros::delay(100);

        // Turn +10° using the PID controller and wait until the turn settles
        std::cout << "Turning +10 degrees..." << std::endl;
        pidDriveController.turnAngle(10_stDeg, 6.0, 3_sec, true);
        pros::delay(500);

        // Additional settling time after turn to ensure robot is completely stable
        // This helps prevent vibration/oscillation from affecting sensor readings
        std::cout << "Allowing robot to settle..." << std::endl;
        pros::delay(500);

        std::cout << "\n[Step " << (i + 2) << "/37] Sampling sensors..." << std::endl;
        PoseSample s = takePoseSample();
        samples.push_back(s);
    }

    std::cout << "\nData collection complete. Samples collected: " << samples.size() << "\n";

    // 3) Tuning step: keep each sensor's orientation the same, but search for the
    //    x/y (within ±1 inch of the current guess) that best matches all readings.
    struct BestFitResult { double bestX; double bestY; double bestError; int used; };

    auto fitSensor = [&](const std::string& name,
                         const units::Pose& currentPose,
                         auto selector) -> BestFitResult {
        // selector gets the appropriate SensorTriple from a PoseSample
        // Grid search over x and y in ±2.0 in around current guess, step 0.1 in
        double x0 = to_in(currentPose.x);
        double y0 = to_in(currentPose.y);
        double bestErr = std::numeric_limits<double>::infinity();
        double bestX = x0, bestY = y0;
        int usedCount = 0; // how many samples actually contributed

        auto evalError = [&](double xGuessIn, double yGuessIn) -> double {
            double sse = 0.0; // sum of squared errors (weighted)
            int used = 0;
            units::Pose guess(from_in(xGuessIn), from_in(yGuessIn), currentPose.orientation);
            for (const auto& s : samples) {
                const SensorTriple& T = selector(s);
                // Use weighted average distance from this pose
                if (std::isnan(T.weightedAvgInches) || T.totalConfidence <= 0) continue;
                Length expected = utils::calculateExpectedDistance(s.odomPose, guess);
                double expectedIn = to_in(expected);
                double error = T.weightedAvgInches - expectedIn;
                // Weight by confidence (normalize to ~0..1, adjusted for 7 readings)
                // Weight by confidence (normalize to ~0..1, adjusted for 3 readings)
                double w = std::min(1.0, std::max(0.0, T.totalConfidence / 63.0 / 3.0));
                sse += w * error * error;
                ++used;
            }
            if (used == 0) return std::numeric_limits<double>::infinity();
            return sse / used; // mean squared error
        };

        for (double dx = -2.0; dx <= 2.0001; dx += 0.1) {
            for (double dy = -2.0; dy <= 2.0001; dy += 0.1) {
                double err = evalError(x0 + dx, y0 + dy);
                if (err < bestErr) {
                    bestErr = err;
                    bestX = x0 + dx;
                    bestY = y0 + dy;
                }
            }
        }

        // Count how many samples would be used at the best pose (for reporting)
        {
            units::Pose bestPose(from_in(bestX), from_in(bestY), currentPose.orientation);
            int used = 0;
            for (const auto& s : samples) {
                const SensorTriple& T = selector(s);
                if (!std::isnan(T.weightedAvgInches) && T.totalConfidence > 0) ++used;
            }
            usedCount = used;
        }

        return {bestX, bestY, bestErr, usedCount};
    };

    // Run fitting for each sensor
    auto frontFit = fitSensor("Front", frontSensorPos, [](const PoseSample& s) -> const SensorTriple& { return s.front; });
    auto rightFit = fitSensor("Right", rightSensorPos, [](const PoseSample& s) -> const SensorTriple& { return s.right; });
    auto backFit  = fitSensor("Back",  backSensorPos,  [](const PoseSample& s) -> const SensorTriple& { return s.back;  });
    auto leftFit  = fitSensor("Left",  leftSensorPos,  [](const PoseSample& s) -> const SensorTriple& { return s.left;  });

    // Print suggested updates and then apply them
    auto reportAndApply = [&](const std::string& name, units::Pose& pose, const BestFitResult& r) {
        std::cout << "\n=== " << name << " Sensor Fit ===\n";
        std::cout << "Used samples: " << r.used << ", mean squared error: " << r.bestError << " (inches^2)\n";
        std::cout << "Old pose: (x = " << to_in(pose.x) << ", y = " << to_in(pose.y)
                  << ", orientation = " << to_cDeg(pose.orientation) << ")\n";
        std::cout << "New pose: (x = " << r.bestX << ", y = " << r.bestY
                  << ", orientation = " << to_cDeg(pose.orientation) << ")\n";
        // Apply the tuned x/y while keeping the original orientation
        pose.x = from_in(r.bestX);
        pose.y = from_in(r.bestY);
    };

    reportAndApply("Front", frontSensorPos, frontFit);
    reportAndApply("Right", rightSensorPos, rightFit);
    reportAndApply("Back",  backSensorPos,  backFit);
    reportAndApply("Left",  leftSensorPos,  leftFit);

    std::cout << "\nTuning complete. The sensor positions above are now updated in memory.\n";
    std::cout << "You can copy these numbers into main.cpp to make them permanent.\n";
}

void runParticleFilterTest() {    
    // First, diagnose sensor hardware
    printf("\n=== SENSOR HARDWARE DIAGNOSTIC ===\n");
    printf("Raw sensor readings and confidence values:\n");
    printf("Front sensor (port 7): distance=%dmm, confidence=%d/63\n", 
           frontSensor.get_distance(), frontSensor.get_confidence());
    printf("Right sensor (port 8): distance=%dmm, confidence=%d/63\n", 
           rightSensor.get_distance(), rightSensor.get_confidence());
    printf("Back sensor (port 10): distance=%dmm, confidence=%d/63\n", 
           backSensor.get_distance(), backSensor.get_confidence());
    printf("Left sensor (port 6): distance=%dmm, confidence=%d/63\n", 
           leftSensor.get_distance(), leftSensor.get_confidence());
    printf("Note: 9999mm (393.66\") indicates sensor error/no reading\n");
    printf("======================================\n\n");
    
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
