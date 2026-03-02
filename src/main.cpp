#include "main.h"
#include "antistall/AntistallMotor.hpp"
#include "auton/IntakeAndPistonState.hpp"
#include "auton/CompetitionAutons.hpp"
#include "auton/ParticleFilterTest.hpp"
#include "auton/PathTestRoutine.hpp"
#include "auton/SkidSteerOdomTest.hpp"

#include "control/DriverControl.hpp"
#include "control/PID.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "liblvgl/lvgl.h"
#include "localization/ParticleFilter.hpp"
#include "motion/TrajectoryGenerator.hpp"
 #include "odometry/SkidSteerOdometry.hpp"
// #include "odometry/OneWheelOdometry.hpp"
#include "odometry/TwoWheelOdometry.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "pros/distance.hpp"
#include "robodash/api.h"
#include "tuning/FeedforwardTuner.hpp"
#include "tuning/MotionProfileTuner.hpp"
#include "tuning/PIDAutoTuner.hpp"
#include "tuning/PIDDriveControllerTuner.hpp"
#include "tuning/PIDTuningRoutines.hpp"
#include "utils/DistanceUtils.hpp"
#include "utils/FastMath.hpp"

#include <vector>
#include <fstream>
#include <iostream>

#define LEFT_MOTOR_1 -11
#define LEFT_MOTOR_2 -12
#define LEFT_MOTOR_3 -13
#define RIGHT_MOTOR_1 16
#define RIGHT_MOTOR_2 17
#define RIGHT_MOTOR_3 18

pros::Motor prosLeft1(LEFT_MOTOR_1, pros::MotorGearset::blue);
pros::Motor prosLeft2(LEFT_MOTOR_2, pros::MotorGearset::blue);
pros::Motor prosLeft3(LEFT_MOTOR_3, pros::MotorGearset::blue);
pros::Motor prosRight1(RIGHT_MOTOR_1, pros::MotorGearset::blue);
pros::Motor prosRight2(RIGHT_MOTOR_2, pros::MotorGearset::blue);
pros::Motor prosRight3(RIGHT_MOTOR_3, pros::MotorGearset::blue);
pros::MotorGroup prosLeftMotors({LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_MOTOR_3}, pros::MotorGearset::blue);
pros::MotorGroup prosRightMotors({RIGHT_MOTOR_1, RIGHT_MOTOR_2, RIGHT_MOTOR_3}, pros::MotorGearset::blue);
lemlib::MotorGroup leftMotors({LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_MOTOR_3}, 600_rpm);
lemlib::MotorGroup rightMotors({RIGHT_MOTOR_1, RIGHT_MOTOR_2, RIGHT_MOTOR_3}, 600_rpm);

lemlib::V5InertialSensor imu(14);

// Vertical tracking wheel - about 0.5" right of center
lemlib::V5RotationSensor verticalTrackingWheel(-9);
// Horizontal tracking wheel - centered
lemlib::V5RotationSensor horizontalTrackingWheel(20);
Length trackingWheelDiameter = 2.75_in; // measure and calibrate this like we did for drive wheels

// Snail motors for intake and scoring

// antistall::AntistallMotor secondStageIntake(-10, 200_rpm, 0.0_amp, 10.0_rpm, 0.5, 50, 3);
//                                               port  rpm    stallI   stallV  revPow revDur retries debounce chkInt pause cooldown
antistall::AntistallMotor secondStageIntake(-10, 600_rpm, 2.0_amp, 10.0_rpm, 1.0, 300, 5, 5, 20, 50, 300);

// antistall::AntistallMotor firstStageIntake(1, 600_rpm, 4.0_amp, 0.0_rpm, .8, 150, 101);
antistall::AntistallMotor firstStageIntake(-7, 600_rpm, 1.0_amp, 1.0_rpm, .2, 100, 1, 1, 20, 50, 300);

pros::adi::DigitalOut scraperPiston('H');
pros::adi::AnalogIn potSelector('D');
pros::adi::DigitalOut WingLeft('G');

// Set up distance sensors for particle filter
pros::Distance frontSensor(5);
pros::Distance rightSensor(5);
pros::Distance backSensor(5);
pros::Distance leftSensor(5);

bool MiddleState = false; // Flag to track PTO state
bool scraperDown = false;
AllianceColor allianceColor = AllianceColor::RED;
SnailState snailState = SnailState::OFF;
WingState wingState = WingState::LEFTUP;
ColorSortState colorSortState = ColorSortState::OFF;
LeftOrRight autonStartingPosition = LeftOrRight::LEFT; // Default starting position

// Flag to track if the conveyor is spinning
// Create a controller instance for user input
pros::Controller controller(pros::E_CONTROLLER_MASTER);

control::DriveMode currentDriveMode = control::DriveMode::ARCADE; // Default to arcade mode

// Create the driver control instance
control::DriverControl driverControl(leftMotors, rightMotors, controller);

units::Pose backSensorPos(-4.0_in, -5.0_in, from_cDeg(180.0));
units::Pose leftSensorPos(-4.5_in, 0_in, from_cDeg(90.0));
units::Pose rightSensorPos(4.5_in, 0_in, from_cDeg(-90.0));
units::Pose frontSensorPos(-4.0_in, 7.0_in, from_cDeg(0.0));

// Setup configuration values
// Track width = center-to-center of wheel contact patches
// Robot is 27 holes wide, drivepods are 5 holes wide
// Track = (27 - 5) holes × 0.5"/hole = 11.0"
Length trackWidth = 11.0_in;
Length wheelDiameter = 2.817_in; // Calibrated wheel diameter (nominal 2.75 × 49/47.84)
// Number kS = 0.0;						// Static friction (volts)
/*
Number kS = 0.4217;
Number kV = 0.0936319;					// Velocity feedforward (volts per velocity)
Number kA = 0.035960933;				// Acceleration feedforward (volts per acceleration)
*/

/*
code = working
bugs = none
linde = fat
*/
/*
Number kS = 0.68316;
Number kV = 0.14606;  // Velocity feedforward (volts per velocity)
Number kA = 0.026048; // Acceleration feedforward (volts per acceleration)
*/
Number kS = 0.6317;
Number kV = 0.0958006;  // Velocity feedforward (volts per velocity)
Number kA = 0.035782; // Acceleration feedforward (volts per acceleration)

double linearKp = .25;
double linearKi = 0.0;
double linearKd = 12.0;
double angularKp = .5;
double angularKi = 0.0;
double angularKd = 38.0;
double headingKp = 1.0;	// Heading correction during straight-line drives
double headingKi = 0.0; // (separate from turn-in-place angular gains)
double headingKd = 8.0;
Mass robotMass = 13.0_lb;
Torque driveTrainTorque = 2.1_Nm; // 6 motors at 0.35 Nm each

// Maximum velocity of the robot - we could calculate this from drive RPM and wheelspeed, but hardcode for now based on 600RPM on 2.75
LinearVelocity maxVelocity = 80_inps;
// Calculate a theoretical max acceleration based on torque and mass with a safety factor
double accelerationSafetyFactor = 0.4;
LinearAcceleration maxAccel = accelerationSafetyFactor * ((driveTrainTorque / (wheelDiameter / 2.0) / robotMass));

// S-curve jerk limit: controls the "smoothness" of acceleration ramps.
// Higher value = snappier but jerkier; lower = smoother but slower transitions.
// A good starting point is 2-4× maxAccel.
LinearJerk maxJerk = 2.5 * maxAccel / 1_sec;

// Maximum centripetal acceleration calculation
// 1. Calculate friction-limited centripetal acceleration (slip constraint)
double frictionCoefficient = 0.6;								   // Typical rubber wheels on competition surface
LinearAcceleration maxAccelSlip = frictionCoefficient * 9.81_mps2; // μ × g

// 2. Calculate tipping-limited centripetal acceleration
Length halfTrackWidthMeters = trackWidth / 2.0;
Length centerOfMassHeightMeters = 4_in; // Rough estimate of center of mass height
LinearAcceleration maxAccelTip = ((9.81_mps2 * halfTrackWidthMeters) / centerOfMassHeightMeters);

// 3. Take the minimum of these constraints and apply safety factor
double centripetalSafetyFactor = 0.5; // Conservative safety factor
LinearAcceleration maxCentripetalAccel = centripetalSafetyFactor * std::min(maxAccelSlip, maxAccelTip);

units::Pose initialPose(0_in, 0_in, 0_cDeg); // Initial pose of the robot
// Two tracking wheels + IMU
odometry::TwoWheelOdometry odometrySystem(
	verticalTrackingWheel,
	horizontalTrackingWheel,
	imu,
	trackingWheelDiameter,  // vertical wheel diameter
	trackingWheelDiameter,  // horizontal wheel diameter
	0.5_in,   // vertical wheel lateral offset (positive = right of center)
	-4.075_in,   // horizontal wheel longitudinal offset (centered)
	initialPose);
localization::ParticleFilter particleFilter(
	[]() { return odometrySystem.getPose(); },
	initialPose, 1250);

control::PIDDriveController pidDriveController(
	leftMotors,
	rightMotors,
	{trackWidth, wheelDiameter, linearKp, linearKi, linearKd, angularKp, angularKi, angularKd, kV, kS, .5_in, 1_stDeg, headingKp, headingKi, headingKd, kA, maxVelocity, maxAccel, maxJerk},
	[]() { return odometrySystem.getPose(); },
	[]() { return odometrySystem.getVelocity(); });

control::PIDDriveController pidPfDriveController(
	leftMotors,
	rightMotors,
	{trackWidth, wheelDiameter, linearKp, linearKi, linearKd, angularKp, angularKi, angularKd, kV, kS, .5_in, 1_stDeg, headingKp, headingKi, headingKd, kA, maxVelocity, maxAccel, maxJerk},
	[]() { return particleFilter.getPose(); },
	[]() { return particleFilter.getVelocity(); });
rd::Selector selector({
	// {"Red Skills", autonSkillsRedSideOnly, "", 120},
	// {"Skills", autonSkills, "", 240},
	{"Upper Side", autonLongAndUpperGoal, "", 120},
	{"Rush Lower", autonRushLower, "", 90},
	{"Rush Upper", autonRushUpper, "", 90},
	//{"Lower Side", autonLongAndLowerGoal, "", 120},
	// {"PP Full Path", purePursuitTest, "", 240},
	// {"PP Straight", purePursuitStraightTest, "", 120},
	// {"PP S Curve", purePursuitSTest, "", 180},
	// {"CG Only", autonCenterGoalOnly, "", 240},
	// {"LZ LG CG", autonLoadingZoneLongGoalCenterGoal, "", 240},
	// {"Gen Path Test", genPathTest, "", 55},
	// {"Odom Test", runOdomTest, "", 55},
	{"Spin Calibration", runSpinCalibration, "", 30},
	// {"PF DS Calib", calibrateParticleFilterDistanceSensorPoses, "", 55},
	//{"PF Test", runParticleFilterTest, "", 55},
	/*{"Tune kS", tuneKs, "", 55},
	{"Tune kV", tuneKv, "", 55},
	{"Tune kA", tuneKa, "", 55},
	{"Tune Angular PID", tunePID_Angular, "", 30},
	{"Tune Linear PID", tunePID_Linear, "", 30},
	{"Tune Heading", tunePID_Heading, "", 30},*/
	//{"Autotune PID", tuning::autonAutoTunePID, "", 120},
	//{"Manual Turn", manualTurnTest, "", 55},
	// {"Manual Linear", manualLinearTest, "", 55},
	// {"Path Test", runPathTest, "", 55},
	{"Movement Test", movementTest, "", 55},
	{"Start to Match Load", Start_MatchLoad, "", 55},
	{"Match Load to Long Goal", MatchLoad_LongGoal, "", 55},
	{"Match Load to Wing", MatchLoad_Wing, "", 55},
	{"Bacon Egg and Cheese", BaconEggAndCheese, "", 55},
	{"Double Bacon Egg and Cheese", DoubleBaconAndEgg, "", 120}
});



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	prosLeftMotors.tare_position();
	prosRightMotors.tare_position();
	prosLeftMotors.set_zero_position(0);
	prosRightMotors.set_zero_position(0);
	leftMotors.setAngle(0_stDeg);
	rightMotors.setAngle(0_stDeg);
	secondStageIntake.setBrakeMode(lemlib::BrakeMode::COAST);
	firstStageIntake.setBrakeMode(lemlib::BrakeMode::COAST);

	// Selector callback example, prints selected auton to the console
	selector.sd_load();
	/*
	selector.on_select([](std::optional<rd::Selector::routine_t> routine)
										 {
		driverControl.setDriveMode(control::DriveMode::ARCADE);
		if (routine == std::nullopt) {
			std::cout << "No routine selected" << std::endl;
		} else {
			std::cout << "Selected Routine: " << routine.value().name << std::endl;
			if(routine.value().name == "Gen Path Test") {
				std::cout << "Running Gen Path Test" << std::endl;
				genPathTest();  // This could pre-compute the first part of an auton's path if speed is important
				std::cout << "Completed Gen Path Test" << std::endl;
			} else if (routine.value().name == "Tune kS") {
				characterizationView.showKsTest();
			} else if (routine.value().name == "Tune kV") {
				characterizationView.showKvTest();
			} else if (routine.value().name == "Tune kA") {
				characterizationView.showKaTest();
			} else if (routine.value().name == "Odom Test") {
				initialPose = {0_in, 0_in, 0_cDeg};
			} else if (routine.value().name == "Skills") {
				driverControl.setDriveMode(control::DriveMode::TANK);
			}
		} });
	*/
	if (!imu.isConnected())
	{
		controller.rumble("---");
	}
	else
	{
		imu.calibrate();
		while (imu.isCalibrating())
		{
			pros::delay(20);
		}
		controller.rumble(".");
	}

	std::cout << "Initializing robot...Done!" << std::endl;

	/*
	// Add distance sensors
	particleFilter.addDistanceSensor(
			0,							// ID
			frontSensorPos, // Now passing the complete pose with position and orientation
			[]()
			{ return from_mm(frontSensor.get_distance()); },
			[]()
			{ return frontSensor.get_confidence(); },
			[](const units::Pose &pose)
			{
				return utils::calculateExpectedDistance(pose, frontSensorPos);
			});

	particleFilter.addDistanceSensor(
			1,						 // ID
			backSensorPos, // Now passing the complete pose with position and orientation
			[]()
			{ return from_mm(backSensor.get_distance()); },
			[]()
			{ return backSensor.get_confidence(); },
			[](const units::Pose &pose)
			{
				return utils::calculateExpectedDistance(pose, backSensorPos);
			});

	particleFilter.addDistanceSensor(
			2,						 // ID
			leftSensorPos, // Now passing the complete pose with position and orientation
			[]()
			{ return from_mm(leftSensor.get_distance()); },
			[]()
			{ return leftSensor.get_confidence(); },
			[](const units::Pose &pose)
			{
				return utils::calculateExpectedDistance(pose, leftSensorPos);
			});

	particleFilter.addDistanceSensor(
			3,							// ID
			rightSensorPos, // Now passing the complete pose with position and orientation
			[]()
			{ return from_mm(rightSensor.get_distance()); },
			[]()
			{ return rightSensor.get_confidence(); },
			[](const units::Pose &pose)
			{
				return utils::calculateExpectedDistance(pose, rightSensorPos);
			});
*/
	pros::Task myAsyncControlTask([]
								  {
	  uint32_t lastTimeRun = pros::millis();
	  while (true)
	  {
		 intakeAntiStallColorSort();
		 // Only run antistall during autonomous, not driver control
		 if (pros::competition::is_autonomous()) {
			 firstStageIntake.doAntistall();
			 secondStageIntake.doAntistall();
		 }
		 //tophood.doAntistall();
		 //basketChain.doAntistall();
		 // pros::c::task_delay_until(&lastTimeRun, 10);
		 pros::delay(5);
	  } });
}

// Helper function to transform a sensor pose from robot-relative to field coordinates

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	wingState = WingState::LEFTUP;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	WingLeft.set_value(MiddleState);
}

void getAutonColorState()
{
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	// Read the file /usd/rd_auton.txt and print the contents to the console
	std::cout << "Autonomous mode started" << std::endl;
	// Capture start time for autonomous routine execution
	uint32_t autonStartMs = pros::millis();
	selector.run_auton();

	// Compute and report total autonomous execution time
	uint32_t autonEndMs = pros::millis();
	uint32_t elapsedMs = autonEndMs - autonStartMs;
	double elapsedSec = elapsedMs / 1000.0;
	std::cout << "Autonomous routine completed in " << elapsedMs << " ms (" << elapsedSec << " s)" << std::endl;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	// Set motor brake modes
	leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
	rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);
	driverControl.setDriveMode(control::DriveMode::ARCADE);

	while (true)
	{
		// Update driver control with the current drive mode
		driverControl.update();

// #define USE_NEW_CONTROLS 
#if USE_NEW_CONTROLS 
		// === NEW CONTROL SCHEME ===
		// L1 alone  = Long goal scoring
		// L2 alone  = Intake (Index)
		// R2 + L1   = Middle upper goal scoring  (R2 is "Shift")
		// R2 + L2   = Outtake
		// R1        = Wing deadman switch (hold = deploy, release = retract)
		// The thread running intakeAntistallColorSort handles the motor control
		// based on snailState.
		{
			bool shift = controller.get_digital(DIGITAL_R2);

			if (shift && controller.get_digital(DIGITAL_L2))
			{
				snailState = SnailState::Out;
			}
			else if (shift && controller.get_digital(DIGITAL_L1))
			{
				snailState = SnailState::Middle;
			}
			else if (controller.get_digital(DIGITAL_DOWN))
			{
				snailState = SnailState::Out;
			}
			else if (controller.get_digital(DIGITAL_RIGHT))
			{
				snailState = SnailState::Middle;
			}
			else if (controller.get_digital(DIGITAL_L2))
			{
				snailState = SnailState::Long;
			}
			else if (controller.get_digital(DIGITAL_L1))
			{
				snailState = SnailState::Index;
			}
			else
			{
				snailState = SnailState::OFF;
			}
			if (controller.get_digital(DIGITAL_R1))
			{
			wingState = WingState::DOWN;
			}
			else
			{
			wingState = WingState::LEFTUP;
			}
			if (shift && controller.get_digital(DIGITAL_R1))
			{
				scraperDown = !scraperDown; // Toggle scraper state
				scraperPiston.set_value(scraperDown);
			}
			

		}

		// R1 = Wing deadman switch: hold to deploy (DOWN), release to retract (UP)
		
		
#else 
		// === OLD CONTROL SCHEME ===
		// L1 - Intake to the basket
		// L2 - Out take to the Lower Center Goal / Field
		// R1 - Score in the Long Goal
		// R2 - Score in the Upper Center Goal
		// The thread running intakeAntistallColorSort handles the motor control
		// based on snailState.
		if (controller.get_digital(DIGITAL_L1))
		{
			snailState = SnailState::Index;
		}
		else if (controller.get_digital(DIGITAL_L2))
		{
			snailState = SnailState::Out;
		}
		else if (controller.get_digital(DIGITAL_R1))
		{
			snailState = SnailState::Long;
		}
		else if (controller.get_digital(DIGITAL_R2))
		{
			snailState = SnailState::Middle;
		}
		else
		{
			snailState = SnailState::OFF; // No buttons pressed, stop the intake and scoring motors
		}
		// Wing: hold to deploy (DOWN), release to retract (RIGHT)
		if (controller.get_digital_new_press(DIGITAL_DOWN) || controller.get_digital_new_press(DIGITAL_RIGHT))
		{
			if (wingState == WingState::LEFTUP)
			{
				wingState = WingState::DOWN;
			}
			else if (wingState == WingState::DOWN)
			{
				wingState = WingState::LEFTUP;
			}
	
			
		}
#endif


		if (controller.get_digital_new_press(DIGITAL_B))
		{
			scraperDown = !scraperDown; // Toggle scraper state
			scraperPiston.set_value(scraperDown);
		}
		if (controller.get_digital_new_press(DIGITAL_Y))
		{
			scraperDown = !scraperDown; // Toggle scraper state
			scraperPiston.set_value(scraperDown);
		}

		pros::delay(20);
	}
}