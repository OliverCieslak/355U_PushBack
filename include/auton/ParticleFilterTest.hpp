#pragma once

#include "tuning/CharacterizationView.hpp"
#include "viz/DiagnosticsView.hpp"

/**
 * @brief Tests the particle filter localization system with predetermined movements
 * 
 * This test drives the robot in a square pattern, allowing us to verify the accuracy
 * of the particle filter localization compared to odometry.
 */
void runParticleFilterTest();
void testPoseTransformAndExpectedDistance();
void calibrateParticleFilterDistanceSensorPoses();
void calibrateLeftYAndBackXAtRotatedHeading();
void debugBackSensorTransformation();
void debugLeftSensorConnectivity();