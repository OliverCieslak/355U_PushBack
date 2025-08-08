#include "auton/AntiStallColorSort.hpp"
#include "control/PIDDriveController.hpp"
#include "odometry/SkidSteerOdometry.hpp"
#include "main.h"
#include "units/units.hpp"
#include "units/Pose.hpp"

#pragma once

enum class LeftOrRight {
    LEFT = -1,
    RIGHT = 1
};

extern control::PIDDriveController pidDriveController;
extern odometry::SkidSteerOdometry odometrySystem;
extern LeftOrRight autonStartingPosition;
extern pros::ADIDigitalOut scraperPiston;

void autonCenterGoalOnly();
void autonLoadingZoneLongGoalCenterGoal();
