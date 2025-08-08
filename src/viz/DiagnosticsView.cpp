#include "viz/DiagnosticsView.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// Add a forward declaration of the global variable
extern control::DriveMode currentDriveMode;

namespace viz
{

  // Initialize static color constants
  const lv_color_t DiagnosticsView::COLOR_NORMAL = lv_color_make(50, 255, 140);
  const lv_color_t DiagnosticsView::COLOR_WARNING = lv_color_make(255, 165, 0);
  const lv_color_t DiagnosticsView::COLOR_ERROR = lv_color_make(255, 50, 50);
  const lv_color_t DiagnosticsView::COLOR_PANEL_BG = lv_color_make(10, 10, 25);
  const lv_color_t DiagnosticsView::COLOR_LEFT_MOTOR = lv_color_make(0, 150, 255);  // Blue
  const lv_color_t DiagnosticsView::COLOR_RIGHT_MOTOR = lv_color_make(255, 70, 70); // Red
  const lv_color_t DiagnosticsView::COLOR_BUTTON_BG = lv_color_make(40, 40, 100);   // Dark blue for button
  const lv_color_t DiagnosticsView::COLOR_BUTTON_TEXT = lv_color_make(255, 255, 255); // White text
  // Remove the static const member definition for MOTORS_PER_SIDE since it's now constexpr

  DiagnosticsView::DiagnosticsView()
  {
    // Create a RoboDash view for diagnostics visualization
    m_view = rd_view_create("Drivetrain");
    init();
  }

  DiagnosticsView::~DiagnosticsView()
  {
    // RoboDash manages the view's lifecycle
    m_view = nullptr;
  }

  void DiagnosticsView::init()
  {
    lv_obj_t *parent = rd_view_obj(m_view);

    // Create a panel for motor indicators.
    m_drivetrain_panel = lv_obj_create(parent);
    lv_obj_set_size(m_drivetrain_panel, 280, 130); // changed panel width from 300 to 280
    lv_obj_align(m_drivetrain_panel, LV_ALIGN_CENTER, 0, 0);

    // Create title label.
    m_title_label = lv_label_create(parent);
    lv_obj_set_style_text_font(m_title_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(m_title_label, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text(m_title_label, "Diagnostics View");
    lv_obj_align(m_title_label, LV_ALIGN_TOP_MID, 0, 5);

    // Create IMU orientation label as a child of the panel instead of parent.
    m_imu_orientation_label = lv_label_create(m_drivetrain_panel);
    lv_obj_set_style_text_font(m_imu_orientation_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(m_imu_orientation_label, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text(m_imu_orientation_label, "Heading: 0.0°");
    lv_obj_align(m_imu_orientation_label, LV_ALIGN_TOP_MID, 0, -7);

    // Create drive mode toggle button
    m_drive_mode_button = lv_btn_create(parent);
    lv_obj_set_size(m_drive_mode_button, 150, 40); // Increased size for better visibility
    // Position the button higher so it's clearly visible
    lv_obj_align(m_drive_mode_button, LV_ALIGN_BOTTOM_MID, 0, -15);
    
    // Set button colors with higher contrast
    lv_obj_set_style_bg_color(m_drive_mode_button, lv_color_make(60, 80, 200), LV_PART_MAIN); // Brighter blue
    lv_obj_set_style_text_color(m_drive_mode_button, COLOR_BUTTON_TEXT, LV_PART_MAIN);
    
    // Add border to make button stand out
    lv_obj_set_style_border_width(m_drive_mode_button, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(m_drive_mode_button, lv_color_make(255, 255, 255), LV_PART_MAIN);
    
    // Add label to the button
    lv_obj_t* btn_label = lv_label_create(m_drive_mode_button);
    lv_label_set_text(btn_label, "Arcade Drive");
    // Explicitly set the label's text color
    lv_obj_set_style_text_color(btn_label, COLOR_BUTTON_TEXT, LV_PART_MAIN);
    // Make the text font bigger
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_center(btn_label);
    
    // Add click event handler
    lv_obj_add_event_cb(m_drive_mode_button, driveModeButtonCallback, LV_EVENT_CLICKED, this);
    
    // Ensure the button is in front of other UI components
    lv_obj_move_foreground(m_drive_mode_button);

    // Create motor indicators: 3 for left (side 0) and 3 for right (side 1)
    for (int i = 0; i < MOTORS_PER_SIDE; i++)
    {
      createMotorIndicator(0, i);
    }
    for (int i = 0; i < MOTORS_PER_SIDE; i++)
    {
      createMotorIndicator(1, i);
    }
  }

  void DiagnosticsView::driveModeButtonCallback(lv_event_t* event)
  {
    // Get the DiagnosticsView instance from the user_data
    DiagnosticsView* view = static_cast<DiagnosticsView*>(lv_event_get_user_data(event));
    if (view) {
      // Toggle the drive mode
      control::DriveMode newMode = (view->m_drive_mode == control::DriveMode::ARCADE) ? 
                          control::DriveMode::TANK : control::DriveMode::ARCADE;
      
      // Update the button text
      view->setDriveMode(newMode);
      
      // Update the global drive mode variable
      currentDriveMode = newMode;
    }
  }

  void DiagnosticsView::setDriveMode(control::DriveMode mode)
  {
    m_drive_mode = mode;
    
    // Update button text
    lv_obj_t* btn_label = lv_obj_get_child(m_drive_mode_button, 0);
    if (btn_label) {
      lv_label_set_text(btn_label, (m_drive_mode == control::DriveMode::ARCADE) ? 
                       "Arcade Drive" : "Tank Drive");
    }
  }

  void DiagnosticsView::createMotorIndicator(int side, int index)
  {
    int x_offset = 0;
    int y_offset = 10 + index * 30; // unchanged

    // Remove gauge creation; create only a label for motor data.
    lv_obj_t *motor_label = lv_label_create(m_drivetrain_panel);
    lv_obj_set_style_text_color(motor_label, COLOR_NORMAL, LV_PART_MAIN);

    // Set initial text and position including the motor port (initially -1)
    lv_label_set_text(motor_label, "P-1: ---°C | ---°");
    if (side == 0)
    {
      lv_obj_align(motor_label, LV_ALIGN_TOP_LEFT, x_offset, y_offset);
    }
    else
    {
      lv_obj_align(motor_label, LV_ALIGN_TOP_RIGHT, x_offset, y_offset);
    }

    // Save the label reference.
    m_motor_labels[side][index] = motor_label;

    // Initialize motor data
    m_motor_data[side][index].temperature = 23_celsius;
    m_motor_data[side][index].position = 0.0_cDeg;
    m_motor_data[side][index].port = -1;
  }

  void DiagnosticsView::updateMotorIndicator(int side, int index)
  {
    // Validate indices
    if (side < 0 || side >= 2 || index < 0 || index >= MOTORS_PER_SIDE)
    {
      return;
    }

    // Get the motor label
    lv_obj_t *motor_label = m_motor_labels[side][index];

    if (!motor_label)
    {
      return;
    }

    // Get the motor data
    const MotorData &data = m_motor_data[side][index];

    // Set color based on temperature.
    lv_color_t text_color = COLOR_NORMAL;
    if (data.temperature > 65.0_celsius)
    {
      text_color = COLOR_ERROR;
    }
    else if (data.temperature > 50.0_celsius)
    {
      text_color = COLOR_WARNING;
    }

    // Update the label text with temperature and position (degrees).
    char label_text[30]; // increased buffer size
    std::snprintf(label_text, sizeof(label_text), "P%d: %.1f°C | %.0f°",
                  data.port,
                  units::to_celsius(data.temperature),
                  to_cDeg(data.position));
    lv_label_set_text(motor_label, label_text);
    lv_obj_set_style_text_color(motor_label, text_color, LV_PART_MAIN);
  }

  void DiagnosticsView::updateDriveTrainMotors(
      std::vector<MotorData> leftMotorReadings,
      std::vector<MotorData> rightMotorReadings)
  {
    // Iterate over left motors
    for (size_t i = 0; i < leftMotorReadings.size() && i < MOTORS_PER_SIDE; i++)
    {
      // Update motor data
      m_motor_data[0][i] = leftMotorReadings[i];
      updateMotorIndicator(0, i);
    }
    // Iterate over right motors
    for (size_t i = 0; i < rightMotorReadings.size() && i < MOTORS_PER_SIDE; i++)
    {
      // Update motor data
      m_motor_data[1][i] = rightMotorReadings[i];
      updateMotorIndicator(1, i);
    }
  }

  void DiagnosticsView::updateIMU(Angle a)
  {
    // Update orientation label
    char orientation_text[20];
    std::snprintf(orientation_text, sizeof(orientation_text),
                  "Heading: %.1f°",
                  to_cDeg(a));
    lv_label_set_text(m_imu_orientation_label, orientation_text);
  }

} // namespace viz
