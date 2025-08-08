#include <iostream>
#include <cmath>

// Simplified test to check coordinate transformation
int main() {
    // Robot at (48, 48, 0°compass) = (48, 48, 90°standard)  
    double robotX = 48.0;
    double robotY = 48.0;
    double robotHeadingStandard = 90.0; // 0° compass = 90° standard
    
    // Front sensor: (-5.5, 9.16, 0°compass) = (-5.5, 9.16, 90°standard)
    double sensorX = -5.5;
    double sensorY = 9.16;
    double sensorHeadingStandard = 90.0; // 0° compass = 90° standard
    
    // Transform sensor to global coordinates
    double theta = robotHeadingStandard * M_PI / 180.0;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    
    // Global sensor position
    double globalX = robotX + sensorX * cos_theta - sensorY * sin_theta;
    double globalY = robotY + sensorX * sin_theta + sensorY * cos_theta;
    
    // Global sensor heading
    double globalHeading = robotHeadingStandard + sensorHeadingStandard;
    if (globalHeading >= 360.0) globalHeading -= 360.0;
    
    std::cout << "Robot: (" << robotX << ", " << robotY << ", " << robotHeadingStandard << "° std)\n";
    std::cout << "Sensor relative: (" << sensorX << ", " << sensorY << ", " << sensorHeadingStandard << "° std)\n";
    std::cout << "Sensor global: (" << globalX << ", " << globalY << ", " << globalHeading << "° std)\n";
    
    // Calculate expected distance to walls
    // Standard angles: 0°=East, 90°=North, 180°=West, 270°=South
    // Field bounds: x=±72", y=±72"
    
    double sensorHeadingRad = globalHeading * M_PI / 180.0;
    double sinAngle = sin(sensorHeadingRad);
    double cosAngle = cos(sensorHeadingRad);
    
    std::cout << "Sensor direction: cos=" << cosAngle << ", sin=" << sinAngle << "\n";
    
    // Check which walls sensor could hit
    bool canHitNorth = (sinAngle > 0);
    bool canHitEast = (cosAngle > 0);  
    bool canHitSouth = (sinAngle < 0);
    bool canHitWest = (cosAngle < 0);
    
    std::cout << "Can hit: North=" << canHitNorth << ", East=" << canHitEast 
              << ", South=" << canHitSouth << ", West=" << canHitWest << "\n";
    
    // Calculate distances to walls sensor is pointing toward
    if (canHitNorth) {
        double distToNorth = (72.0 - globalY) / sinAngle;
        std::cout << "Distance to North wall: " << distToNorth << " in\n";
    }
    
    if (canHitEast) {
        double distToEast = (72.0 - globalX) / cosAngle;  
        std::cout << "Distance to East wall: " << distToEast << " in\n";
    }
    
    return 0;
}
