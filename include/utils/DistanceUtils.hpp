#pragma once

#include "units/Pose.hpp"
#include "units/units.hpp"
#include "units/Vector2D.hpp"
#include "utils/FastMath.hpp"
#include "utils/Utils.hpp" // Include Utils.hpp to access the conversion functions
#include <vector>

namespace utils {

/**
 * @brief Transforms a sensor's pose (relative to robot center) to global field coordinates
 *
 * @param robotPose The robot's global pose (x, y, orientation)
 * @param sensorPose The sensor's pose relative to the robot center (x, y, orientation)
 * @return units::Pose The sensor's global pose on the field
 */
inline units::Pose sensorPoseToGlobal(const units::Pose& robotPose, const units::Pose& sensorPose) {
    // Convert compass heading (0=N, 90=E, 180=S, 270=W) to standard trig angle (0=E, 90=N, 180=W, 270=S)
    auto compassToTrig = [](Angle compass) {
        return units::constrainAngle180(from_cDeg(90.0 - to_cDeg(compass)));
    };

    Angle robotThetaTrig = compassToTrig(robotPose.orientation);
    double cosTheta = fastCos(to_stRad(robotThetaTrig));
    double sinTheta = fastSin(to_stRad(robotThetaTrig));

    Length globalSensorX = robotPose.x + sensorPose.x * cosTheta - sensorPose.y * sinTheta;
    Length globalSensorY = robotPose.y + sensorPose.x * sinTheta + sensorPose.y * cosTheta;

    // For orientation, add sensor orientation to robot orientation, but convert sensor orientation from compass to trig before adding, then convert back to compass
    auto trigToCompass = [](Angle trig) {
        return units::constrainAngle180(from_cDeg(90.0 - to_cDeg(trig)));
    };

    Angle sensorOrientationTrig = compassToTrig(sensorPose.orientation);
    Angle globalOrientationTrig = robotThetaTrig + sensorOrientationTrig;
    Angle globalSensorOrientation = trigToCompass(globalOrientationTrig);

    return units::Pose(globalSensorX, globalSensorY, globalSensorOrientation);
}

// Field dimensions: how big the field is
const Length fieldWidth = 12_ft;  // Width of the field (12 feet)
const Length fieldHeight = 12_ft; // Height of the field (12 feet)

// Calculate field boundaries: where the edges of the field are
const Length halfWidth = fieldWidth / 2.0;   // Half of the field width
const Length halfHeight = fieldHeight / 2.0;  // Half of the field height

/**
 * @brief Calculates the expected distance from a sensor to the nearest wall
 *
 * The robot has a sensor that can measure how far away the nearest wall is.
 * This function helps predict what that sensor should read, based on where the robot is,
 * where the sensor is on the robot, and which direction the sensor is pointing.
 *
 * @param pose Current robot pose (where the robot is and which direction it's facing)
 * @param sensorPose Position and orientation of the sensor relative to the robot's center
 * @return Length The expected distance to the nearest wall
 */
Length calculateExpectedDistance(
    const units::Pose& pose,
    const units::Pose& sensorPose);

/**
 * @brief Calculates the squared distance between two poses (position only, ignores orientation)
 *
 * This function figures out how far apart two locations are. It only cares about the (x, y)
 * coordinates of the locations and ignores which direction they're facing.
 *
 * It calculates the *squared* distance (distance * distance), which is faster than calculating
 * the actual distance because it avoids using a square root. This is useful when you only need
 * to compare distances and don't care about the exact number.
 *
 * @param pose1 First pose (location)
 * @param pose2 Second pose (location)
 * @return Area The squared distance between the positions (as an Area type)
 */
Area poseDistanceSquared(const units::Pose& pose1, const units::Pose& pose2);

/**
 * @brief Calculates the expected noise/accuracy of the V5 Distance Sensor based on the measured distance
 *
 * The V5 Distance Sensor isn't perfect; it has some error in its readings. This function estimates
 * how much error to expect, based on how far away the object is.
 *
 * According to V5 Distance Sensor specifications:
 * - Below 200mm: accuracy is approximately ±15mm
 * - Above 200mm: accuracy is approximately 5% of the measured distance
 *
 * @param measuredDistance The distance measured by the sensor
 * @return Length The expected standard deviation of measurement error
 */
Length calculateV5DistanceSensorNoise(Length measuredDistance);

/**
 * @brief Checks if a distance sensor reading is valid
 *
 * The V5 Distance Sensor can only measure distances within a certain range. This function checks
 * if a sensor reading is within that valid range.
 *
 * The V5 Distance Sensor has a range of 20mm to 2400mm.
 *
 * @param distance The measured distance
 * @return bool True if the distance is within the valid range
 */
bool isValidDistanceSensorReading(Length distance);

/**
 * @brief Check if a position is within the field boundaries
 *
 * This function checks if a given (x, y) coordinate is inside the playing field.
 *
 * @param x X position to check
 * @param y Y position to check
 * @return bool True if within boundaries
 */
bool isWithinFieldBoundaries(
    const Length& x,
    const Length& y);

/**
 * @brief Constrain a pose to remain within field boundaries
 *
 * If the robot tries to go outside the field, this function will move it back to the edge of the field.
 * It keeps the robot's location inside the field.
 *
 * @param pose The pose to constrain
 * @return units::Pose The constrained pose
 */
units::Pose constrainToField(
    const units::Pose& pose);

/**
 * @brief Structure to hold a sensor ID and its corresponding weight/quality score
 */
struct SensorScore {
    int sensorID;      // ID of the sensor
    double score;      // How good/reliable the sensor reading is
    Length measurement; // The distance measured by the sensor

    // Default constructor - required for vector operations
    SensorScore() : sensorID(0), score(0.0), measurement(-1_in) {}

    // Constructor
    SensorScore(int id, double s, Length d) : sensorID(id), score(s), measurement(d) {}

    // Comparison operator for sorting
    bool operator<(const SensorScore& other) const {
        if(score == other.score) {
            return measurement < other.measurement; // Sort by measurement (lower distance first) if scores are equal
        }
        return score > other.score; // Sort in descending order (highest score first)
    }
};

/**
 * @brief Calculate the signed curvature of the arc tangent to two poses
 * 
 * @param pose Current pose
 * @param target Target position
 * @return Number Signed curvature (1/radius). Positive = counter-clockwise, negative = clockwise
 */
Number getSignedTangentArcCurvature(const units::Pose& pose, const units::Pose& target);

/**
 * @brief Calculate the curvature at a point in a path
 * 
 * @param prevPose Previous pose
 * @param currentPose Current pose
 * @param nextPose Next pose
 * @return Curvature Calculated curvature
 */
Curvature calculateCurvature(
    const units::Pose& prevPose,
    const units::Pose& currentPose,
    const units::Pose& nextPose);

// Remove these function declarations as they're defined in Utils.hpp
// LinearVelocity toLinear(AngularVelocity angular, Length diameter);
// LinearAcceleration toLinear(AngularAcceleration angular, Length diameter);
// AngularVelocity toAngular(LinearVelocity linear, Length diameter);
// AngularAcceleration toAngular(LinearAcceleration linear, Length diameter);

} // namespace utils
