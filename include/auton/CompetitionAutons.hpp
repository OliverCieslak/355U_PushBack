#include "auton/IntakeAndPistonState.hpp"
#include "control/PIDDriveController.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "localization/ParticleFilter.hpp"
#include "main.h"
#include "motion/TrajectoryGenerator.hpp"
#include "odometry/OneWheelOdometry.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "units/units.hpp"
#include "units/Pose.hpp"


#pragma once

enum class LeftOrRight {
    LEFT = -1,
    RIGHT = 1
};

enum class UpperOrLower {
    UPPER,
    LOWER
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
extern odometry::OneWheelOdometry odometrySystem;
extern localization::ParticleFilter particleFilter;
extern LeftOrRight autonStartingPosition;
extern pros::adi::DigitalOut scraperPiston;
extern pros::adi::DigitalOut WingLeft;
extern pros::adi::AnalogIn potSelector;

extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::Distance leftSensor;
extern pros::Distance rightSensor;

void autonPartnerSelfAWPDumb();
void autonLongAndUpperGoal();
void autonRushLower();
void autonRushUpper();
void autonLongAndLowerGoal();
void autonSkills();
void autonSkillsRedSideOnly();
void autonTwentyBallSkills();
void purePursuitTest();
void purePursuitStraightTest();
void purePursuitSTest();
void movementTest();
void manualTurnTest();
void manualLinearTest();
void matchLoadToLongGoalStart (uint32_t t0);