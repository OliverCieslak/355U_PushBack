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
    // Convert orientations from compass to standard degrees for all math
    units::Pose robotPoseStd(robotPose.x, robotPose.y, from_stDeg(to_stDeg(robotPose.orientation)));
    units::Pose sensorOffsetStd(sensorOffset.x, sensorOffset.y, from_stDeg(to_stDeg(sensorOffset.orientation)));
    // Use sensorPoseToGlobal for transformation (now in standard degrees)
    units::Pose globalSensorPose = sensorPoseToGlobal(robotPoseStd, sensorOffsetStd);

    // Pre-calculate trig values for sensor orientation (trig angle)
    double cosAngle = fastCos(to_stRad(globalSensorPose.orientation));
    double sinAngle = fastSin(to_stRad(globalSensorPose.orientation));

    // Define epsilon to avoid division by very small values
    const double EPSILON = 0.000001; // A tiny number to prevent dividing by zero
    
    // Calculate distances to each wall, but only if sensor is pointing toward that wall
    Length minDist = 999_ft;
    const char* chosenWall = "None";
    double chosenValue = 0.0;
    
    // North wall (y = halfHeight) - only if pointing North (sinAngle > 0)
    if (sinAngle > EPSILON) {
        Length distToNorth = (halfHeight - globalSensorPose.y) / sinAngle;
        if (distToNorth > EPSILON * 1_in && distToNorth < minDist) {
            minDist = distToNorth;
            chosenWall = "North";
            chosenValue = distToNorth.convert(in);
        }
    }
    // South wall (y = -halfHeight) - only if pointing South (sinAngle < 0)
    if (sinAngle < -EPSILON) {
        Length distToSouth = (globalSensorPose.y - (-halfHeight)) / (-sinAngle);
        if (distToSouth > EPSILON * 1_in && distToSouth < minDist) {
            minDist = distToSouth;
            chosenWall = "South";
            chosenValue = distToSouth.convert(in);
        }
    }
    // East wall (x = halfWidth) - only if pointing East (cosAngle > 0)
    if (cosAngle > EPSILON) {
        Length distToEast = (halfWidth - globalSensorPose.x) / cosAngle;
        if (distToEast > EPSILON * 1_in && distToEast < minDist) {
            minDist = distToEast;
            chosenWall = "East";
            chosenValue = distToEast.convert(in);
        }
    }
    // West wall (x = -halfWidth) - only if pointing West (cosAngle < 0)
    if (cosAngle < -EPSILON) {
        Length distToWest = (globalSensorPose.x - (-halfWidth)) / (-cosAngle);
        if (distToWest > EPSILON * 1_in && distToWest < minDist) {
            minDist = distToWest;
            chosenWall = "West";
            chosenValue = distToWest.convert(in);
        }
    }
    // Debug output for which wall and value was chosen
    // std::cout << "[ExpectedDistance Debug] Sensor at global (" << to_in(globalSensorPose.x) << ", " << to_in(globalSensorPose.y) << ") facing " 
    //           << to_stDeg(globalSensorPose.orientation) << "° => Closest wall is " << chosenWall
    //           << " at " << chosenValue << " in (field bounds: ±" << to_in(halfWidth) << " x, ±" << to_in(halfHeight) << " y)" << std::endl;
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
        const Length FIXED_ACCURACY = 30_mm;  // Increased from 15mm
        const double PERCENTAGE_ACCURACY = 0.15; // Increased from 0.05
        if (measuredDistance < THRESHOLD) {
            return FIXED_ACCURACY;
        } else {
            return measuredDistance * PERCENTAGE_ACCURACY;
        }
    }

    bool isValidDistanceSensorReading(Length distance) {
        const Length MIN_RANGE = 20_mm;
        const Length MAX_RANGE = 2400_mm;  // V5 sensor max range
        const Length FIELD_MAX = 110_in;   // Stricter field-based limit
        
        // First check sensor hardware limits
        if (distance < MIN_RANGE || distance > MAX_RANGE) {
            return false;
        }
        
        // Then check field geometry limits
        if (distance > FIELD_MAX) {
            return false;
        }
        
        // Reject known error values
        if (std::abs(to_in(distance) - 393.66) < 0.5) {  // Common V5 error value
            return false;
        }
        
        return true;
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
        // Use circumcircle formula for magnitude and cross product sign for direction (CCW positive)
        Length a = units::hypot(nextPose.x - currentPose.x, nextPose.y - currentPose.y); // |current->next|
        Length b = units::hypot(prevPose.x - nextPose.x, prevPose.y - nextPose.y);       // |next->prev|
        Length c = units::hypot(prevPose.x - currentPose.x, prevPose.y - currentPose.y); // |prev->current|
        if (a < 0.001_in || b < 0.001_in || c < 0.001_in) {
            return 0_radpm;
        }
        Length s = (a + b + c) / 2.0;
        Area triangleArea = units::sqrt(s * (s - a) * (s - b) * (s - c));
        if (triangleArea < 1e-6_in2) return 0_radpm;
    // Raw magnitude using lengths in inches -> store directly as 1/in (treat internal units consistently with other code paths)
    Curvature mag = Curvature(4.0 * triangleArea / (a * b * c));
        // Determine sign via z-component of cross((current-prev),(next-current))
        double x1 = to_in(currentPose.x - prevPose.x);
        double y1 = to_in(currentPose.y - prevPose.y);
        double x2 = to_in(nextPose.x - currentPose.x);
        double y2 = to_in(nextPose.y - currentPose.y);
        double cross = x1 * y2 - y1 * x2; // >0 => CCW turn
        if (cross < 0) mag = Curvature(-mag.internal());
        // Clamp unrealistically large curvature (degenerate tiny triangles) to prevent numerical blow-up
        const double maxCurv = 1.5; // 1/in -> min turn radius ~0.67 in (already very tight)
        if (std::fabs(mag.internal()) > maxCurv) {
            mag = Curvature((mag.internal() > 0 ? 1 : -1) * maxCurv);
        }
        return mag;
    }

} // namespace utils
