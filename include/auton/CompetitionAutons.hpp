#include "auton/AntiStallColorSort.hpp"
#include "control/PIDDriveController.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "localization/ParticleFilter.hpp"
#include "main.h"
#include "motion/TrajectoryGenerator.hpp"
#include "odometry/SkidSteerOdometry.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "units/units.hpp"
#include "units/Pose.hpp"


#pragma once

enum class LeftOrRight {
    LEFT = -1,
    RIGHT = 1
};

extern Length trackWidth;
extern Length wheelDiameter;
extern Number kS;
extern Number kV;
extern Number kA;
extern Mass robotMass;
extern Torque driveTrainTorque;
extern LinearVelocity maxVelocity;
extern LinearAcceleration maxAccel;
extern LinearAcceleration maxCentripetalAccel;
extern control::PIDDriveController pidDriveController;
extern control::PIDDriveController pidPfDriveController;
extern odometry::SkidSteerOdometry odometrySystem;
extern localization::ParticleFilter particleFilter;
extern LeftOrRight autonStartingPosition;
extern pros::ADIDigitalOut scraperPiston;
extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::ADIDigitalOut HoodClose;
void autonFourBallLongGoal();
void autonSevenBallLongGoal();
void autonNineBallLongGoal();
void autonSkills();
void purePursuitTest();
void purePursuitStraightTest();
void purePursuitSTest();

void manualTurnTest();
void manualLinearTest();