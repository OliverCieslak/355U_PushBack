#include "control/DriverControl.hpp"
#include <cmath>

namespace control {

DriverControl::DriverControl(lemlib::MotorGroup& leftMotors, 
                             lemlib::MotorGroup& rightMotors,
                             pros::Controller& controller,
                             DriveMode mode) 
    : m_leftMotors(leftMotors), 
      m_rightMotors(rightMotors),
      m_controller(controller),
      m_driveMode(mode) {
}

void DriverControl::setDriveMode(DriveMode mode) {
  m_driveMode = mode;
}

DriveMode DriverControl::getDriveMode() const {
  return m_driveMode;
}

void DriverControl::update() {
  switch (m_driveMode) {
    case DriveMode::TANK:
      tankDrive();
      break;
    case DriveMode::ARCADE:
      arcadeDrive();
      break;
  }
}

void DriverControl::setDeadband(int deadband) {
  m_deadband = deadband;
}

void DriverControl::setCurve(double curve) {
  m_curve = curve;
}

double DriverControl::processJoystickInput(int input) const {
  // Apply deadband
  if (std::abs(input) < m_deadband) {
    return 0.0;
  }
  
  // Normalize to -1.0 to 1.0
  double normalized = static_cast<double>(input) / 127.0;
  
  // Apply curve (sign-preserving)
  if (m_curve != 1.0) {
    return std::copysign(std::pow(std::abs(normalized), m_curve), normalized);
  }
  
  return normalized;
}

void DriverControl::tankDrive() {
  // Get raw joystick values for both sticks
  int leftY = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightY = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  
  // Process inputs
  double leftPower = processJoystickInput(leftY);
  double rightPower = processJoystickInput(rightY);
  
  // Apply to motors
  m_leftMotors.move(leftPower);
  m_rightMotors.move(rightPower);
}

void DriverControl::arcadeDrive() {  // Get raw joystick values
  int throttle = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // Process inputs
  double throttlePower = processJoystickInput(throttle);
  double turnPower = processJoystickInput(turn);
  
  // Arcade algorithm
  double leftPower = throttlePower + turnPower;
  double rightPower = throttlePower - turnPower;

  // Normalize to -1.0 to 1.0
  double maxPower = std::max(std::abs(leftPower), std::abs(rightPower));
  if (maxPower > 1.0) {
    leftPower /= maxPower;
    rightPower /= maxPower;
  }
  
  // Apply to motors
  m_leftMotors.move(leftPower);
  m_rightMotors.move(rightPower);
}

} // namespace control
