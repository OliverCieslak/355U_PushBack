#include <iostream>
#include "utils/DistanceUtils.hpp"
#include <algorithm>
#include <cmath>

namespace utils {

// Usage:
//   Length expected = calculateExpectedDistance(robotPose, sensorPose);
// Where:
//   - robotPose: The robot's current field pose (x, y, orientation in compass degrees)
//   - sensorPose: The sensor's offset and orientation RELATIVE to the robot center (robot coordinates)
//     - For front/back sensors: offset is (x, 0)
//     - For left/right sensors: offset is (0, y)
//       - left: (0, +5.62, 270)
//       - right: (0, -5.62, 90)
// Do NOT pre-transform the sensor pose; this function will handle the transformation.
// All angles must use compass heading (0° = North, 90° = East, 180° = South, 270° = West).
// Returns the expected distance from the sensor to the nearest field wall.
Length calculateExpectedDistance(const units::Pose& robotPose, const units::Pose& sensorOffset) {
    // Debug: Print input values for transformation
    // std::cout << "[TransformDebug] Robot pose: (" << to_in(robotPose.x) << ", " << to_in(robotPose.y) << ", " << to_cDeg(robotPose.orientation) << ")" << std::endl;
    // std::cout << "[TransformDebug] Sensor offset: (" << to_in(sensorOffset.x) << ", " << to_in(sensorOffset.y) << ", " << to_cDeg(sensorOffset.orientation) << ")" << std::endl;
    // Use sensorPoseToGlobal for transformation
    units::Pose globalSensorPose = sensorPoseToGlobal(robotPose, sensorOffset);

    // Pre-calculate trig values for sensor orientation (trig angle)
    double cosAngle = fastCos(to_stRad(globalSensorPose.orientation));
    double sinAngle = fastSin(to_stRad(globalSensorPose.orientation));

    // Print only one line per sensor
    // std::cout << "[ExpectedDistance] Sensor global pose: (" << to_in(globalSensorPose.x) << ", " << to_in(globalSensorPose.y)
    //          << ") heading " << to_cDeg(globalSensorPose.orientation) << " deg" << std::endl;

    // Define epsilon to avoid division by very small values
    const double EPSILON = 0.000001; // A tiny number to prevent dividing by zero
    
    // Calculate distances to each wall efficiently
    Length minDist = 999_ft;
    const char* chosenWall = "None";
    double chosenValue = 0.0;
    
    // North wall (y = halfHeight)
    if (sinAngle > EPSILON) {
        Length distToNorth = (halfHeight - globalSensorPose.y) / sinAngle;
        if (distToNorth > EPSILON * 1_in && distToNorth < minDist) {
            minDist = distToNorth;
            chosenWall = "North";
            chosenValue = distToNorth.convert(in);
        }
    }
    // South wall (y = -halfHeight)
    if (-sinAngle > EPSILON) {
        Length distToSouth = (globalSensorPose.y - (-halfHeight)) / (-sinAngle);
        if (distToSouth > EPSILON * 1_in && distToSouth < minDist) {
            minDist = distToSouth;
            chosenWall = "South";
            chosenValue = distToSouth.convert(in);
        }
    }
    // East wall (x = halfWidth)
    if (cosAngle > EPSILON) {
        Length distToEast = (halfWidth - globalSensorPose.x) / cosAngle;
        if (distToEast > EPSILON * 1_in && distToEast < minDist) {
            minDist = distToEast;
            chosenWall = "East";
            chosenValue = distToEast.convert(in);
        }
    }
    // West wall (x = -halfWidth)
    if (-cosAngle > EPSILON) {
        Length distToWest = (globalSensorPose.x - (-halfWidth)) / (-cosAngle);
        if (distToWest > EPSILON * 1_in && distToWest < minDist) {
            minDist = distToWest;
            chosenWall = "West";
            chosenValue = distToWest.convert(in);
        }
    }
    // Debug output for which wall and value was chosen
    // std::cout << "[ExpectedDistance] Sensor global pose: (" << to_in(globalSensorPose.x) << ", " << to_in(globalSensorPose.y)
    //          << ") heading " << to_cDeg(globalSensorPose.orientation) << " deg: Closest wall is " << chosenWall
    //          << " at " << chosenValue << " in" << std::endl;
    return minDist;
}
    // Utility functions below are now inside the utils namespace
    Area poseDistanceSquared(const units::Pose& pose1, const units::Pose& pose2) {
        Length dx = pose1.x - pose2.x;
        Length dy = pose1.y - pose2.y;
        return dx * dx + dy * dy;
    }

    Length calculateV5DistanceSensorNoise(Length measuredDistance) {
        const Length THRESHOLD = 200_mm;
        const Length FIXED_ACCURACY = 15_mm;
        const double PERCENTAGE_ACCURACY = 0.05;
        if (measuredDistance < THRESHOLD) {
            return FIXED_ACCURACY;
        } else {
            return measuredDistance * PERCENTAGE_ACCURACY;
        }
    }

    bool isValidDistanceSensorReading(Length distance) {
        const Length MIN_RANGE = 20_mm;
        const Length MAX_RANGE = 2400_mm;
        return (distance >= MIN_RANGE && distance <= MAX_RANGE);
    }

    bool isWithinFieldBoundaries(const Length& x, const Length& y) {
        return (x >= -utils::halfWidth && x <= utils::halfWidth && y >= -utils::halfHeight && y <= utils::halfHeight);
    }

    units::Pose constrainToField(const units::Pose& pose) {
        Length constrainedX = pose.x;
        if (constrainedX < -utils::halfWidth) constrainedX = -utils::halfWidth;
        if (constrainedX > utils::halfWidth) constrainedX = utils::halfWidth;
        Length constrainedY = pose.y;
        if (constrainedY < -utils::halfHeight) constrainedY = -utils::halfHeight;
        if (constrainedY > utils::halfHeight) constrainedY = utils::halfHeight;
        return units::Pose(constrainedX, constrainedY, pose.orientation);
    }

    Number getSignedTangentArcCurvature(const units::Pose& pose, const units::Pose& target) {
        double dx = to_in(target.x - pose.x);
        double dy = to_in(target.y - pose.y);
        double hx = utils::fastCos(to_stRad(pose.orientation));
        double hy = utils::fastSin(to_stRad(pose.orientation));
        double distance = utils::fastSqrt(dx*dx + dy*dy);
        if (distance < 0.001) return 0.0;
        dx /= distance;
        dy /= distance;
        double sinTheta = dx*hy - dy*hx;
        double cosTheta = dx*hx + dy*hy;
        if (std::abs(sinTheta) < 0.001) return 0.0;
        return 2.0 * sinTheta / distance;
    }

    Curvature calculateCurvature(
        const units::Pose& prevPose,
        const units::Pose& currentPose,
        const units::Pose& nextPose
    ) {
        Length a = units::hypot(nextPose.x - currentPose.x, nextPose.y - currentPose.y);
        Length b = units::hypot(prevPose.x - nextPose.x, prevPose.y - nextPose.y);
        Length c = units::hypot(prevPose.x - currentPose.x, prevPose.y - currentPose.y);
        Length s = (a + b + c) / 2.0;
        Area triangleArea = units::sqrt(s * (s - a) * (s - b) * (s - c));
        if (a < 0.001_in || b < 0.001_in || c < 0.001_in) {
            return 0_radpm;
        }
        return Curvature(4.0 * triangleArea / (a * b * c));
    }

} // namespace utils
