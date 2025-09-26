#include "main.h"
#include "antistall/AntistallMotor.hpp"
#include "auton/AntiStallColorSort.hpp"
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
#include "pros/distance.hpp"
#include "robodash/api.h"
#include "tuning/CharacterizationView.hpp"
#include "tuning/FeedforwardTuner.hpp"
#include "tuning/MotionProfileTuner.hpp"
#include "tuning/PIDDriveControllerTuner.hpp"
#include "utils/DistanceUtils.hpp"
#include "utils/FastMath.hpp"
#include "viz/FieldView.hpp"
#include "viz/DiagnosticsView.hpp"
#include <vector>
#include <fstream>
#include <iostream>

#define LEFT_MOTOR_1 -13
#define LEFT_MOTOR_2 -14
#define LEFT_MOTOR_3 -15
#define RIGHT_MOTOR_1 17
#define RIGHT_MOTOR_2 18
#define RIGHT_MOTOR_3 19

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

lemlib::V5InertialSensor imu(10);

// Snail motors for intake and scoring

antistall::AntistallMotor secondStageIntake(-2, 200_rpm, 0.0_amp, 10.0_rpm, 0.5, 50, 3);
antistall::AntistallMotor tophood(16, 200_rpm, 0.0_amp, 0.0_rpm, 0.5, 100, 1);
antistall::AntistallMotor firstStageIntake(-3, 600_rpm, 0.0_amp, 0.0_rpm, 1, 80, 5);

pros::ADIDigitalOut scraperPiston('G');
pros::ADIDigitalOut HoodClose('B');
pros::Optical topColorSortingSensor(5);
pros::Optical lowColorSortingSensor(5);
pros::Optical autonColorSensor(4); // For auton color detection

// Set up distance sensors for particle filter
pros::Distance frontSensor(6);
pros::Distance rightSensor(7);
pros::Distance backSensor(20);
pros::Distance leftSensor(8);

bool HoodState = false; // Flag to track PTO state
bool scraperDown = false; // Flag to track scraper state
AllianceColor allianceColor = AllianceColor::RED;
SnailState snailState = SnailState::OFF;
ConveyorState conveyorState = ConveyorState::OFF;
ColorSortState colorSortState = ColorSortState::OFF;
LeftOrRight autonStartingPosition = LeftOrRight::LEFT; // Default starting position

 // Initial state of the conveyor
// Anti-stall variables for firstStageISecond
uint32_t lastAntiStallTime = 0;
bool isJiggling = false;
bool jiggleDirection = true; // true = forward, false = backward
uint32_t jiggleStartTime = 0;	 
int jiggleCount = 0;

// Anti-stall variables for basketChain
bool isJiggling2 = false;
bool jiggleDirection2 = true; // true = forward, false = backward
uint32_t jiggleStartTime2 = 0;
int jiggleCount2 = 0;
// Flag to track if the conveyor is spinning
// Create a controller instance for user input
pros::Controller controller(pros::E_CONTROLLER_MASTER);

control::DriveMode currentDriveMode = control::DriveMode::ARCADE; // Default to arcade mode

// Create the driver control instance
control::DriverControl driverControl(leftMotors, rightMotors, controller);

units::Pose backSensorPos(0.5_in, -4.3_in, from_cDeg(180.0));
units::Pose leftSensorPos(-6.5_in, 0_in, from_cDeg(90.0));
units::Pose rightSensorPos(5.5_in, 0.5_in, from_cDeg(-90.0));
units::Pose frontSensorPos(-3.75_in, 7.5_in, from_cDeg(0.0));

// Setup configuration values - initial estimates that will be refined
Length trackWidth = 13.5_in;		// Initial estimate for track width
Length wheelDiameter = 2.75_in;     // Diameter of wheels
// Number kS = 0.0;								// Static friction (volts)
Number kS = 0.6171;
Number kV = 0.09796043;								// Velocity feedforward (volts per velocity)
Number kA = 0.0328166;								// Acceleration feedforward (volts per acceleration)

double linearKp = .15;
double linearKi = 0.0;
double linearKd = 0.0;
double angularKp = .5;
double angularKi = 0.0;
double angularKd = 33.0;
Mass robotMass = 13.6_lb;
Torque driveTrainTorque = 2.1_Nm; // 6 motors at 0.35 Nm each

// Maximum velocity of the robot - we could calculate this from drive RPM and wheelspeed, but hardcode for now based on 600RPM on 2.75
LinearVelocity maxVelocity = 80_inps;
// Calculate a theoretical max acceleration based on torque and mass with a safety factor
double accelerationSafetyFactor = 0.4;
LinearAcceleration maxAccel = accelerationSafetyFactor * ((driveTrainTorque / (wheelDiameter / 2.0) / robotMass));

// Maximum centripetal acceleration calculation
// 1. Calculate friction-limited centripetal acceleration (slip constraint)
double frictionCoefficient = 0.6;																	 // Typical rubber wheels on competition surface
LinearAcceleration maxAccelSlip = frictionCoefficient * 9.81_mps2; // μ × g

// 2. Calculate tipping-limited centripetal acceleration
Length halfTrackWidthMeters = trackWidth / 2.0;
Length centerOfMassHeightMeters = 4_in; // Rough estimate of center of mass height
LinearAcceleration maxAccelTip = ((9.81_mps2 * halfTrackWidthMeters) / centerOfMassHeightMeters);

// 3. Take the minimum of these constraints and apply safety factor
double centripetalSafetyFactor = 0.5; // Conservative safety factor
LinearAcceleration maxCentripetalAccel = centripetalSafetyFactor * std::min(maxAccelSlip, maxAccelTip);

units::Pose initialPose(0_in, 0_in, 0_cDeg); // Initial pose of the robot
// Create the odometry instance for tracking robot position
odometry::SkidSteerOdometry odometrySystem(
		leftMotors,
		rightMotors,
		imu,
		trackWidth,
		wheelDiameter,
		initialPose);

localization::ParticleFilter particleFilter(odometrySystem, initialPose, 1250);

control::PIDDriveController pidDriveController(
		leftMotors,
		rightMotors,
		{trackWidth, wheelDiameter, linearKp, linearKi, linearKd, angularKp, angularKi, angularKd, kV, kS},
		[]()
		{ return odometrySystem.getPose(); });

control::PIDDriveController pidPfDriveController(
		leftMotors,
		rightMotors,
		{trackWidth, wheelDiameter, linearKp, linearKi, linearKd, angularKp, angularKi, angularKd, kV, kS},
		[]()
		{ return particleFilter.getPose(); });

rd::Selector selector({
		 {"7 Ball", autonSevenBallLongGoal, "", 240},
		 {"4 Ball", autonFourBallLongGoal, "", 240},
		 {"Skills", autonSkills, "", 240},
		 // {"9 Ball", autonNineBallLongGoal, "", 240},
		 // {"PP Full Path", purePursuitTest, "", 240},
		 // {"PP Straight", purePursuitStraightTest, "", 120},
		 // {"PP S Curve", purePursuitSTest, "", 180},
		 // {"CG Only", autonCenterGoalOnly, "", 240},
		 // {"LZ LG CG", autonLoadingZoneLongGoalCenterGoal, "", 240},
		// {"Gen Path Test", genPathTest, "", 55},
		// {"Odom Test", runOdomTest, "", 55},
		// {"PF DS Calib", calibrateParticleFilterDistanceSensorPoses, "", 55},
		{"PF Test", runParticleFilterTest, "", 55},
		// {"Tune kS", tuneKs, "", 55},
		// {"Tune kV", tuneKv, "", 55},
		// {"Tune kA", tuneKa, "", 55},
		{"Manual Turn", manualTurnTest, "", 55},
		{"Manual Linear", manualLinearTest, "", 55},
		// {"Path Test", runPathTest, "", 55},
});

viz::FieldView fieldView;
tuning::CharacterizationView characterizationView;
viz::DiagnosticsView diagnosticsView;

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
	tophood.setBrakeMode(lemlib::BrakeMode::COAST);
	firstStageIntake.setBrakeMode(lemlib::BrakeMode::COAST);

	// Selector callback example, prints selected auton to the console
	selector.sd_load();
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

	pros::Task myAsyncControlTask([]{
	  uint32_t lastTimeRun = pros::millis();
	  while (true)
	  {
		 intakeAntiStallColorSort();
		 firstStageIntake.doAntistall();
		 //tophood.doAntistall();
		 //basketChain.doAntistall();
		 //secondStageIntake.doAntistall();
		 pros::c::task_delay_until(&lastTimeRun, 10);
	  }
	});
}

// Helper function to transform a sensor pose from robot-relative to field coordinates

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	int rightSensorDistance = rightSensor.get_distance();
	int leftSensorDistance = leftSensor.get_distance();

	if(rightSensorDistance > 0 && leftSensorDistance > 0) {
		if(rightSensorDistance < leftSensorDistance) {
			autonStartingPosition = LeftOrRight::RIGHT;
		} else {
			autonStartingPosition = LeftOrRight::LEFT;
		}
	}

	getAutonColorState();

	printf("Alliance: %s - Auton Starting Position: %s\n", 
		(allianceColor == AllianceColor::BLUE) ? "BLUE" : "RED",
		(autonStartingPosition == LeftOrRight::LEFT) ? "LEFT" : "RIGHT");
	controller.print(0, 0, "%s - %s", 
		(allianceColor == AllianceColor::BLUE) ? "BLUE" : "RED",
		(autonStartingPosition == LeftOrRight::LEFT) ? "LEFT" : "RIGHT");
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
	topColorSortingSensor.set_led_pwm(100); // Ensure the sensor is active
	getAutonColorState();

	// probably did not connect in the right order
	int rightSensorDistance = rightSensor.get_distance();
	int leftSensorDistance = leftSensor.get_distance();

	std::cout << "Right Sensor Distance: " << rightSensorDistance << " mm, Left Sensor Distance: " << leftSensorDistance << " mm" << std::endl;
	if(rightSensorDistance > 0 && leftSensorDistance > 0) {
		if(rightSensorDistance > 1300 && rightSensorDistance < 1450) {
			autonStartingPosition = LeftOrRight::RIGHT;
		} else if(leftSensorDistance > 1300 && leftSensorDistance < 1450) {
			autonStartingPosition = LeftOrRight::LEFT;
		} else {
			std::cout << "Could not determine auton starting position from distance sensors, hardcoding to RIGHT" << std::endl;
			autonStartingPosition = LeftOrRight::RIGHT;
		}
	}
	std::cout << "Alliance: " << ((allianceColor == AllianceColor::BLUE) ? "BLUE" : "RED") 
		<< " - Auton Starting Position: " << ((autonStartingPosition == LeftOrRight::LEFT) ? "LEFT" : "RIGHT") << std::endl;
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
	topColorSortingSensor.set_led_pwm(100); // Ensure the sensor is active
	leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
	rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);
	if(diagnosticsView.getDriveMode() == control::DriveMode::ARCADE) {
		currentDriveMode = control::DriveMode::ARCADE;
	} else {
		currentDriveMode = control::DriveMode::ARCADE;
	}
	driverControl.setDriveMode(currentDriveMode);

	while (true)
	{
		// Update driver control with the current drive mode
		driverControl.update();

		// L1 - Intake to the basket
		// L2 - Out take to the Lower Center Goal / Field
		// R1 - Score in the Long Goal
		// R2 - Score in the Upper Center Goal
		// The thread running intakeAntistallColorSort handles the motor control
		// based on snailState.	
		if (controller.get_digital(DIGITAL_L1)) {
			snailState = SnailState::Index;
		} else if (controller.get_digital(DIGITAL_L2)) {
			snailState = SnailState::Out;
		} else if (controller.get_digital(DIGITAL_R1)) {
			snailState = SnailState::Long;
		} else if (controller.get_digital(DIGITAL_R2)) {	
			snailState = SnailState::Middle;
		} else {
			snailState = SnailState::OFF; // No buttons pressed, stop the intake and scoring motors
		}
		if(controller.get_digital_new_press(DIGITAL_A)) {
			HoodState = !HoodState;
			scraperPiston.set_value(HoodState);
		 } // Close the hood
		if(controller.get_digital_new_press(DIGITAL_DOWN)) {
			if(conveyorState == ConveyorState::REVERSED) {
				conveyorState = ConveyorState::OFF;      // Stop the conveyor
			} else {
				conveyorState = ConveyorState::REVERSED; // Start the conveyor
			}
		}
		if(controller.get_digital_new_press(DIGITAL_B)) {
			scraperDown = !scraperDown;             // Toggle scraper state
			HoodClose.set_value(scraperDown);  // Move scraper piston down or up()
		}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)) {
			if (colorSortState == ColorSortState::OFF) {
				colorSortState = ColorSortState::RED; // Start color sorting in red mode
				printf("Color sorting started in red mode\n");
			} else if(colorSortState == ColorSortState::RED) {
				colorSortState = ColorSortState::BLUE; // Switch to blue mode
				printf("Color sorting switched to blue mode\n");
			} else if(colorSortState == ColorSortState::BLUE) {
				colorSortState = ColorSortState::OFF; // Turn off color sorting
				printf("Color sorting turned off\n");
			}
		}

		pros::delay(20);
	}
}