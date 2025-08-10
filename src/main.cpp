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
#include "motion/MotionProfilerRamseteController.hpp"
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

// Create motor groups for left and right sides of the drivetrain
// Don't use the prosMotors for control, only for odometry
/*
pros::Motor prosLeft1(8, pros::MotorGearset::blue);
pros::Motor prosLeft2(9, pros::MotorGearset::blue);
pros::Motor prosLeft3(-10, pros::MotorGearset::blue);
pros::Motor prosRight1(-1, pros::MotorGearset::blue);
pros::Motor prosRight2(-2, pros::MotorGearset::blue);
pros::Motor prosRight3(3, pros::MotorGearset::blue);
pros::MotorGroup prosLeftMotors({8, 9, -10}, pros::MotorGearset::blue);
pros::MotorGroup prosRightMotors({-1, -2, 3}, pros::MotorGearset::blue);
lemlib::MotorGroup leftMotors({-8, -9, 10}, 600_rpm);
lemlib::MotorGroup rightMotors({1, 2, -3}, 600_rpm);
*/
#define LEFT_MOTOR_1 -16
#define LEFT_MOTOR_2 -17
#define LEFT_MOTOR_3 18
#define RIGHT_MOTOR_1 13
#define RIGHT_MOTOR_2 14
#define RIGHT_MOTOR_3 -15
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

lemlib::V5InertialSensor imu(12);

// Snail motors for intake and scoring

antistall::AntistallMotor secondStageIntake(-2, 200_rpm, 0.0_amp, 10.0_rpm, 0.5, 50, 3);
antistall::AntistallMotor basketMotor(3, 200_rpm, 0.0_amp, 0.0_rpm, 0.5, 100, 1);
/*
lemlib::Motor basketChain(-1, 200_rpm);
lemlib::Motor basketMotor(3, 200_rpm);
lemlib::Motor firstStageIntake(-11, 200_rpm);
*/
//antistall::AntistallMotor secondStageIntake(-2, 200_rpm);
antistall::AntistallMotor basketChain(-1, 200_rpm, 0.5_amp, 1.0_rpm, 1.0, 300, 1);
antistall::AntistallMotor firstStageIntake(-11, 200_rpm, 0.5_amp, 1.0_rpm, 0.5, 10, 3);

pros::ADIDigitalOut scraperPiston('A');
pros::ADIDigitalOut ptoPiston('B');
pros::Optical topColorSortingSensor(5);
pros::Optical lowColorSortingSensor(5);
pros::Optical autonColorSensor(4); // For auton color detection

// Set up distance sensors for particle filter
pros::Distance frontSensor(7);
pros::Distance rightSensor(8);
pros::Distance backSensor(10);
pros::Distance leftSensor(6);

bool ptoEngaged = false; // Flag to track PTO state
bool scaperDown = false; // Flag to track scraper state
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

// Tune these by putting the robot at known locations and measuring the distance
// Changed from units::Vector2D<Length> to units::Pose with orientation
// Sensor poses: (x, y, orientation)
// x: right of robot center (+), y: forward from robot center (+)
// orientation: compass heading sensor faces
// Set the robot in a known position perpendicular to the wall
// and use the sensors to measure the distance to the wall
// These are the sensor poses relative to the robot center for that coordinate
// E.g. robot facing north in the northeast corner of the field should give
// the front.y and right.x positions

// Sensor orientations are relative to robot frame - use standard angles directly
/* Inital guesses for sensor positions based on physical measurements
units::Pose estBackSensorPos(6_in, -7.64_in, from_cDeg(-180.0));
units::Pose estLeftSensorPos(-6.0_in, 6.5_in, from_cDeg(90.0));
units::Pose estRightSensorPos(7.15_in, 5_in, from_cDeg(-90.0));
units::Pose estFrontSensorPos(-5.5_in, 9.16_in, from_cDeg(0.0));
*/
// Using original physical measurements for the most accurate dimensions
// front.y, back.y, left.x, right.x are based on physical measurements and should be accurate
units::Pose backSensorPos(6_in, -7.64_in, from_cDeg(180.0));
units::Pose leftSensorPos(-6.0_in, 6.5_in, from_cDeg(90.0));
units::Pose rightSensorPos(7.15_in, 5_in, from_cDeg(-90.0));
units::Pose frontSensorPos(-5.5_in, 9.16_in, from_cDeg(0.0));

// Setup configuration values - initial estimates that will be refined
Length trackWidth = 11.0_in;		// Initial estimate for track width
Length wheelDiameter = 2.75_in;     // Diameter of wheels
// Number kS = 0.0;								// Static friction (volts)
Number kS = 3.42;
// Results of 3 physical tests.  Will eventually use kS = 3.42;
// Number kS = 3.21;								// Static friction (volts)
// Number kS = 3.42;								// Static friction (volts)
// Number kS = 3.42;								// Static friction (volts)
// Number kV = 0.0;								// Velocity feedforward (volts per velocity)
Number kV = 0.0926;								// Velocity feedforward (volts per velocity)
// Results of 3 physical tests with kS=3.42.  Will eventually use kV = 0.0926 V/(in/s);
// Number kV = 0.0922;								// Velocity feedforward (volts per velocity)
// Number kV = 0.0897;								// Velocity feedforward (volts per velocity)
// Number kV = 0.0960;								// Velocity feedforward (volts per velocity)
// Number kA = 0.0;								// Acceleration feedforward (volts per acceleration)
Number kA = 0.0332;								// Acceleration feedforward (volts per acceleration)
// Number kA = 0.0332;								// Acceleration feedforward (volts per acceleration)
// Number kA = 0.0333;								// Acceleration feedforward (volts per acceleration)
// Number kA = 0.0327;								// Acceleration feedforward (volts per acceleration)

double linearKp = 0.45;
double linearKi = 0.0;
double linearKd = 0.0;
double angularKp = 1.0;
double angularKi = 0.0;
double angularKd = 165.0;
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

control::PIDDriveController pidDriveController(
		leftMotors,
		rightMotors,
		{trackWidth, wheelDiameter, linearKp, linearKi, linearKd, angularKp, angularKi, angularKd, kV, kS},
		[]()
		{ return odometrySystem.getPose(); });

localization::ParticleFilter particleFilter(odometrySystem, initialPose, 1250);

rd::Selector selector({
		// {"Auton 1", auton1, "", 1},
		 {"CG Only", autonCenterGoalOnly, "", 240},
		 {"LZ LG CG", autonLoadingZoneLongGoalCenterGoal, "", 240},
		// {"Gen Path Test", genPathTest, "", 55},
		// {"Odom Test", runOdomTest, "", 55},
		// {"PF DS Calib", calibrateParticleFilterDistanceSensorPoses, "", 55},
		// {"PF Test", runParticleFilterTest, "", 55},
		// {"Tune kS", tuneKs, "", 55},
		// {"Tune kV", tuneKv, "", 55},
		// {"Tune kA", tuneKa, "", 55},
		// {"Tune Turn PID", tuning::tuneAngularPID, "", 55},
		// {"Tune Linear PID", tuning::tuneLinearPID, "", 55},
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
	basketMotor.setBrakeMode(lemlib::BrakeMode::COAST);
	firstStageIntake.setBrakeMode(lemlib::BrakeMode::COAST);

	// Selector callback example, prints selected auton to the console
	selector.sd_load();
	selector.on_select([](std::optional<rd::Selector::routine_t> routine)
										 {
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
		 //basketMotor.doAntistall();
		 basketChain.doAntistall();
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
	topColorSortingSensor.set_led_pwm(100); // Ensure the sensor is active

	int rightSensorDistance = rightSensor.get_distance();
	int leftSensorDistance = leftSensor.get_distance();

	if(rightSensorDistance > 0 && leftSensorDistance > 0) {
		if(rightSensorDistance < leftSensorDistance) {
			autonStartingPosition = LeftOrRight::RIGHT;
		} else {
			autonStartingPosition = LeftOrRight::LEFT;
		}
	}
		/*
	} else if(rightSensorDistance > 0 && leftSensorDistance <= 0) {
		autonStartingPosition = LeftOrRight::RIGHT;
	} else if(rightSensorDistance <= 0 && leftSensorDistance > 0) {
		autonStartingPosition = LeftOrRight::LEFT;
	}
		*/

	printf("Auton Starting Position: %s\n", (autonStartingPosition == LeftOrRight::LEFT) ? "LEFT" : "RIGHT");
	getAutonColorState();
	// odometrySystem.start();
	selector.run_auton();
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
			snailState = SnailState::INTAKE_TO_BASKET;
		} else if (controller.get_digital(DIGITAL_L2)) {
			snailState = SnailState::SCORE_LOWER_CENTER;
		} else if (controller.get_digital(DIGITAL_R1)) {
			snailState = SnailState::SCORE_LONG_GOAL;
		} else if (controller.get_digital(DIGITAL_R2)) {
			snailState = SnailState::SCORE_UPPER_CENTER;
		} else {
			snailState = SnailState::OFF; // No buttons pressed, stop the intake and scoring motors
		}
		if(controller.get_digital_new_press(DIGITAL_A)) {
			if(conveyorState == ConveyorState::SPINNING) {
				conveyorState = ConveyorState::OFF;      // Stop the conveyor
			} else {
				conveyorState = ConveyorState::SPINNING; // Start the conveyor
			}
		}
		if(controller.get_digital_new_press(DIGITAL_DOWN)) {
			if(conveyorState == ConveyorState::REVERSED) {
				conveyorState = ConveyorState::OFF;      // Stop the conveyor
			} else {
				conveyorState = ConveyorState::REVERSED; // Start the conveyor
			}
		}
		if(controller.get_digital_new_press(DIGITAL_B)) {
			scaperDown = !scaperDown;             // Toggle scraper state
			scraperPiston.set_value(scaperDown);  // Move scraper piston down or up()
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