#pragma once

#include "control/DriverControl.hpp"
#include "liblvgl/lvgl.h"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "robodash/api.h"
#include "units/Angle.hpp"
#include "units/Temperature.hpp"
#include "units/units.hpp"
#include <vector>
#include <array>
#include <string>
#include <functional>

namespace viz {

  struct MotorData {
    Temperature temperature = 20_celsius;  // in degrees Celsius
    Angle position = 0_cDeg;               // in degrees
    int port = -9999;                      // index within side (0-2)
  };
/**
 * @brief A diagnostic view that shows detailed information about drivetrain motors
 */
class DiagnosticsView {
public:
    // Constructor and destructor
    DiagnosticsView();
    ~DiagnosticsView();
    
    // Initialize the view
    void init();
    
    /**
     * @brief Update motor information in the view
     * 
     * @param leftMotorReadings left side motor data
     * @param rightMotorReadings right side motor data
     */
    void updateDriveTrainMotors(
        std::vector<MotorData> leftMotorReadings, 
        std::vector<MotorData> rightMotorReadings
    );
    
    /**
     * @brief Update IMU information in the view
     * 
     * @param a The current IMU reading, passed in as compass heading
     */
    void updateIMU(Angle a);
    
    /**
     * @brief Set the current drive mode to update the button text
     * This does NOT change the actual drive mode
     * 
     * @param mode The current drive mode
     */
    void setDriveMode(control::DriveMode mode);
    
    /**
     * @brief Get the current drive mode
     * 
     * @return control::DriveMode The current drive mode
     */
    control::DriveMode getDriveMode() const {
        return m_drive_mode;
    }

    /**
     * @brief Get the screen object for display
     */
    lv_obj_t* getScreen() const { return rd_view_obj(m_view); }

private:
    // UI components
    rd_view_t* m_view = nullptr;         // RoboDash view
    lv_obj_t* m_title_label = nullptr;   // Title label
    lv_obj_t* m_imu_panel = nullptr;     // IMU panel
    lv_obj_t* m_drivetrain_panel = nullptr;  // Drivetrain motors panel
    lv_obj_t* m_drive_mode_button = nullptr; // Drive mode toggle button
    
    // Current drive mode displayed on the button
    control::DriveMode m_drive_mode = control::DriveMode::ARCADE;
    
    // Motor indicators - 2 sides, 3 motors per side
    static constexpr int MOTORS_PER_SIDE = 3;  // Changed to constexpr
    std::array<std::array<lv_obj_t*, MOTORS_PER_SIDE>, 2> m_motor_indicators{};
    std::array<std::array<lv_obj_t*, MOTORS_PER_SIDE>, 2> m_motor_labels{};
    
    // IMU indicators
    lv_obj_t* m_imu_orientation_label = nullptr;
    
    // Data storage
    std::array<std::array<MotorData, MOTORS_PER_SIDE>, 2> m_motor_data{};
    
    // Helper methods
    void createMotorIndicator(int side, int index);
    void updateMotorIndicator(int side, int index);
    
    // Button callback
    static void driveModeButtonCallback(lv_event_t* event);
    
    // Theme colors
    static const lv_color_t COLOR_NORMAL;
    static const lv_color_t COLOR_WARNING;
    static const lv_color_t COLOR_ERROR;
    static const lv_color_t COLOR_PANEL_BG;
    static const lv_color_t COLOR_LEFT_MOTOR;
    static const lv_color_t COLOR_RIGHT_MOTOR;
    static const lv_color_t COLOR_BUTTON_BG;
    static const lv_color_t COLOR_BUTTON_TEXT;
};

} // namespace viz
