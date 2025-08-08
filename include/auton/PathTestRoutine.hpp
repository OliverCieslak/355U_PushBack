#pragma once

#include "hardware/Motor/MotorGroup.hpp"
#include "motion/MotionProfilerRamseteController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "odometry/SkidSteerOdometry.hpp"
#include "pros/motors.hpp"
#include "robodash/api.h"
#include "units/units.hpp"
#include "viz/FieldView.hpp"

// Function to run a path test as an autonomous routine
void runPathTest();
void genPathTest();
