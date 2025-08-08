#pragma once

#include "hardware/Motor/MotorGroup.hpp"
#include "pros/misc.hpp"

namespace control {
enum class DriveMode {
  ARCADE,
  TANK
};

/**
 * Driver control class for a skid-steer robot.
 * Supports both tank drive and arcade drive modes.
 */
class DriverControl {
public:
  /**
   * Constructor for the DriverControl class
   * 
   * @param leftMotors Motor group for the left side of the drivetrain
   * @param rightMotors Motor group for the right side of the drivetrain 
   * @param controller PROS controller for input
   * @param mode Initial drive mode (default: ARCADE)
   */
  DriverControl(lemlib::MotorGroup& leftMotors, 
                lemlib::MotorGroup& rightMotors,
                pros::Controller& controller,
                DriveMode mode = DriveMode::ARCADE);

  /**
   * Set the drive mode
   * 
   * @param mode The drive mode to use
   */
  void setDriveMode(DriveMode mode);

  /**
   * Get the current drive mode
   * 
   * @return The current drive mode
   */
  DriveMode getDriveMode() const;

  /**
   * Update the motor outputs based on controller input
   * Call this method in a loop to continuously update the motor outputs
   */
  void update();

  /**
   * Set a deadband for the joysticks
   * Inputs with absolute value less than the deadband will be treated as zero
   * 
   * @param deadband The deadband value (0-127)
   */
  void setDeadband(int deadband);

  /**
   * Set an exponential curve for the joystick inputs
   * Higher values make the joystick less sensitive near the center
   * 
   * @param curve The curve value (1.0 = linear, >1.0 = exponential)
   */
  void setCurve(double curve);

private:
  lemlib::MotorGroup& m_leftMotors;
  lemlib::MotorGroup& m_rightMotors;
  pros::Controller& m_controller;
  DriveMode m_driveMode;
  int m_deadband = 5;  // Default deadband
  double m_curve = 1.0;  // Default curve (linear)

  /**
   * Apply deadband and curve to joystick input
   * 
   * @param input Raw joystick input (-127 to 127)
   * @return Processed input value (-1.0 to 1.0)
   */
  double processJoystickInput(int input) const;

  /**
   * Tank drive implementation
   */
  void tankDrive();

  /**
   * Arcade drive implementation
   */
  void arcadeDrive();
};

} // namespace control
