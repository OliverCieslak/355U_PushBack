#include "auton/CompetitionAutons.hpp"
#include "pros/adi.hpp"
#include "AntistallColorSort.hpp" // Add this include for SnailState definition
#include "control/DifferentialDriveConfig.hpp"

void autonSkills()
{
    // Implement the skills autonomous routine here
    motion::TrajectoryConfig config(maxVelocity, maxAccel, maxCentripetalAccel);
    control::DifferentialDriveConfig driveConfig(
        std::max(1.0_in, trackWidth),    // Ensure non-zero track width
        std::max(1.0_in, wheelDiameter), // Ensure non-zero wheel diameter
        kV,
        kA,
        kS);


        // Will make it START TO THE LEFT OF THE RED PARKING ZONE,
    units::Pose initialPose = units::Pose(-60_in, -14_in, from_cDeg(90));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    
    /* We are doing generic points, will flesh out the tehcnicals later...
        (-60,-47,270) Stops at match loader
        (-24,-47,90) Gets to the close side of the long goal
        (63,-47,90) Gets other side match loader
        (24,-47,270) Gets to the other side of the long goal
        (63,47,90) Gets to the other match load tube on the 'far' side
        ()
        
    
    
    
    
    
    
    
    */


    





        /*
    // Set up first movement...
    std::vector<units::Pose> path = {
        initialPose,
        units::Pose(-36_in, -58_in, from_cDeg(90)),
        /*
        units::Pose(29_in, -58_in, from_cDeg(90)),
        units::Pose(40_in, 37_in, from_cDeg(0)),
        units::Pose(-31_in, -59_in, from_cDeg(270)),
        units::Pose(-47_in, -58_in, from_cDeg(270)),
        units::Pose(-47_in, 12_in, from_cDeg(180)),
        
    };
    */
    // Generate and follow trajectory
    printf("Generating traj");

    // Measure trajectory generation time
    uint32_t start_time = pros::micros();

    // motion::Trajectory trajectory = motion::TrajectoryGenerator::generateTrajectory(path, config);

    uint32_t end_time = pros::micros();
    uint32_t generation_time_us = end_time - start_time;
    double generation_time_ms = generation_time_us / 1000.0;
}