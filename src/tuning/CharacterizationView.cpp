#include "tuning/CharacterizationView.hpp"
#include <iostream>

namespace tuning {

CharacterizationView::CharacterizationView() {
    init();
}

CharacterizationView::~CharacterizationView() {
    if (m_view) {
        rd_view_del(m_view);
    }
}

void CharacterizationView::init() {
    // Create the view with RoboDash
    m_view = rd_view_create("Characterization");
    lv_obj_t* screen = rd_view_obj(m_view);
    
    // Create title label (at the top of the screen)
    m_title_label = lv_label_create(screen);
    lv_label_set_text(m_title_label, "Drivetrain Characterization");
    lv_obj_align(m_title_label, LV_ALIGN_TOP_MID, 0, 10);
    
    m_tabview = lv_tabview_create(screen, LV_DIR_TOP, 25);
    lv_obj_set_size(m_tabview, 480, 240);
    lv_obj_align(m_tabview, LV_ALIGN_CENTER, 0, 0);
    
    // Create tabs
    lv_obj_t* params_tab = lv_tabview_add_tab(m_tabview, "Parameters");
    lv_obj_t* velocity_tab = lv_tabview_add_tab(m_tabview, "Velocity");
    lv_obj_t* acceleration_tab = lv_tabview_add_tab(m_tabview, "Acceleration");
    
    // Add Velocity chart to display velocity data points
    m_velocity_chart = lv_chart_create(velocity_tab);
    lv_obj_set_size(m_velocity_chart, 440, 180);
    lv_obj_align(m_velocity_chart, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(m_velocity_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(m_velocity_chart, LV_CHART_UPDATE_MODE_SHIFT);
    m_velocity_series = lv_chart_add_series(m_velocity_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    
    // Create Acceleration chart to display acceleration data points
    m_acceleration_chart = lv_chart_create(acceleration_tab);
    lv_obj_set_size(m_acceleration_chart, 440, 180);
    lv_obj_align(m_acceleration_chart, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(m_acceleration_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(m_acceleration_chart, LV_CHART_UPDATE_MODE_SHIFT);
    m_acceleration_series = lv_chart_add_series(m_acceleration_chart, lv_palette_main(LV_PALETTE_ORANGE), LV_CHART_AXIS_PRIMARY_Y);
    
    // Create parameter panel
    lv_obj_t* params_panel = lv_obj_create(params_tab);
    lv_obj_set_size(params_panel, 440, 180);
    lv_obj_align(params_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_border_width(params_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(params_panel, 10, LV_PART_MAIN);
    
    // Parameters panel title
    lv_obj_t* params_title = lv_label_create(params_panel);
    lv_label_set_text(params_title, "Characterization Parameters");
    lv_obj_align(params_title, LV_ALIGN_TOP_MID, 0, 5);
    
    // Parameter labels
    m_ks_label = lv_label_create(params_panel);
    lv_label_set_text(m_ks_label, "kS: 0.00");
    lv_obj_align(m_ks_label, LV_ALIGN_TOP_LEFT, 20, 40);
    
    m_kv_label = lv_label_create(params_panel);
    lv_label_set_text(m_kv_label, "kV: 0.00");
    lv_obj_align(m_kv_label, LV_ALIGN_TOP_LEFT, 20, 80);
    
    m_ka_label = lv_label_create(params_panel);
    lv_label_set_text(m_ka_label, "kA: 0.00");
    lv_obj_align(m_ka_label, LV_ALIGN_TOP_LEFT, 20, 120);
    
    m_track_width_label = lv_label_create(params_panel);
    lv_label_set_text(m_track_width_label, "Track: 0.00 in");
    lv_obj_align(m_track_width_label, LV_ALIGN_TOP_RIGHT, -20, 40);
    
    m_wheel_diameter_label = lv_label_create(params_panel);
    lv_label_set_text(m_wheel_diameter_label, "Wheel: 0.00 in");
    lv_obj_align(m_wheel_diameter_label, LV_ALIGN_TOP_RIGHT, -20, 80);

    // Add PID tab for Linear and Angular PID values
    lv_obj_t* pid_tab = lv_tabview_add_tab(m_tabview, "PID");
    
    // Create labels for Linear PID Gains
    lv_obj_t* linear_title = lv_label_create(pid_tab);
    lv_label_set_text(linear_title, "Linear PID Gains");
    lv_obj_align(linear_title, LV_ALIGN_TOP_LEFT, 10, 10);
    
    // Assuming m_linear_kp_label etc. are declared as members in the header.
    m_linear_kp_label = lv_label_create(pid_tab);
    lv_label_set_text(m_linear_kp_label, "kP: 0.00");
    lv_obj_align(m_linear_kp_label, LV_ALIGN_TOP_LEFT, 10, 40);
    
    m_linear_ki_label = lv_label_create(pid_tab);
    lv_label_set_text(m_linear_ki_label, "kI: 0.00");
    lv_obj_align(m_linear_ki_label, LV_ALIGN_TOP_LEFT, 10, 70);
    
    m_linear_kd_label = lv_label_create(pid_tab);
    lv_label_set_text(m_linear_kd_label, "kD: 0.00");
    lv_obj_align(m_linear_kd_label, LV_ALIGN_TOP_LEFT, 10, 100);
    
    // Create labels for Angular PID Gains
    lv_obj_t* angular_title = lv_label_create(pid_tab);
    lv_label_set_text(angular_title, "Angular PID Gains");
    lv_obj_align(angular_title, LV_ALIGN_TOP_LEFT, 200, 10);
    
    m_angular_kp_label = lv_label_create(pid_tab);
    lv_label_set_text(m_angular_kp_label, "kP: 0.00");
    lv_obj_align(m_angular_kp_label, LV_ALIGN_TOP_LEFT, 200, 40);
    
    m_angular_ki_label = lv_label_create(pid_tab);
    lv_label_set_text(m_angular_ki_label, "kI: 0.00");
    lv_obj_align(m_angular_ki_label, LV_ALIGN_TOP_LEFT, 200, 70);
    
    m_angular_kd_label = lv_label_create(pid_tab);
    lv_label_set_text(m_angular_kd_label, "kD: 0.00");
    lv_obj_align(m_angular_kd_label, LV_ALIGN_TOP_LEFT, 200, 100);

    // Add new PID Graph tab
    lv_obj_t* pidgraph_tab = lv_tabview_add_tab(m_tabview, "PID Graph");
    
    // Create a chart for PID graph
    m_pid_chart = lv_chart_create(pidgraph_tab);
    lv_obj_set_size(m_pid_chart, 440, 180);
    lv_obj_align(m_pid_chart, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(m_pid_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(m_pid_chart, LV_CHART_UPDATE_MODE_SHIFT);
    
    // Create two series: one for Actual and one for Target values
    m_pid_actual_series = lv_chart_add_series(m_pid_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    m_pid_target_series = lv_chart_add_series(m_pid_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
}

void CharacterizationView::updateKs(double kS) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "kS: %.3f", kS);
    lv_label_set_text(m_ks_label, buf);
}

void CharacterizationView::updateKv(double kV) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "kV: %.3f", kV);
    lv_label_set_text(m_kv_label, buf);
}

void CharacterizationView::updateKa(double kA) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "kA: %.3f", kA);
    lv_label_set_text(m_ka_label, buf);
}

void CharacterizationView::updateTrackWidth(double trackWidth) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Track: %.2f in", trackWidth);
    lv_label_set_text(m_track_width_label, buf);
}

void CharacterizationView::updateWheelDiameter(double wheelDiameter) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Wheel: %.2f in", wheelDiameter);
    lv_label_set_text(m_wheel_diameter_label, buf);
}

void CharacterizationView::showStatusMessage(const char* title, const char* message) {
    std::cout << "[" << title << "] " << message << std::endl;
}

// New update method for Linear PID values
void CharacterizationView::updateLinearPID(double kP, double kI, double kD) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "kP: %.3f", kP);
    lv_label_set_text(m_linear_kp_label, buf);
    std::snprintf(buf, sizeof(buf), "kI: %.3f", kI);
    lv_label_set_text(m_linear_ki_label, buf);
    std::snprintf(buf, sizeof(buf), "kD: %.3f", kD);
    lv_label_set_text(m_linear_kd_label, buf);
}

// New update method for Angular PID values
void CharacterizationView::updateAngularPID(double kP, double kI, double kD) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "kP: %.3f", kP);
    lv_label_set_text(m_angular_kp_label, buf);
    std::snprintf(buf, sizeof(buf), "kI: %.3f", kI);
    lv_label_set_text(m_angular_ki_label, buf);
    std::snprintf(buf, sizeof(buf), "kD: %.3f", kD);
    lv_label_set_text(m_angular_kd_label, buf);
}

void CharacterizationView::updatePIDGraph(double actual, double target) {
    // Append new data points to the chart series
    lv_chart_set_next_value(m_pid_chart, m_pid_actual_series, actual);
    lv_chart_set_next_value(m_pid_chart, m_pid_target_series, target);
}

void CharacterizationView::addVelocityDataPoint(double voltage, double velocity) {
    // Append new velocity value to the velocity chart series.
    // 'voltage' is available if needed for further processing.
    lv_chart_set_next_value(m_velocity_chart, m_velocity_series, velocity);
    lv_chart_refresh(m_velocity_chart);
}

void CharacterizationView::addAccelerationDataPoint(double voltage, double acceleration) {
    // Append new acceleration value to the acceleration chart series
    lv_chart_set_next_value(m_acceleration_chart, m_acceleration_series, acceleration);
    lv_chart_refresh(m_acceleration_chart);
}

void CharacterizationView::clearVelocityData() {
    lv_chart_remove_series(m_velocity_chart, m_velocity_series);
    m_velocity_series = lv_chart_add_series(m_velocity_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_refresh(m_velocity_chart);
}

void CharacterizationView::clearAccelerationData() {
    lv_chart_remove_series(m_acceleration_chart, m_acceleration_series);
    m_acceleration_series = lv_chart_add_series(m_acceleration_chart, lv_palette_main(LV_PALETTE_ORANGE), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_refresh(m_acceleration_chart);
}

void CharacterizationView::clearPIDGraphData() {
    lv_chart_remove_series(m_pid_chart, m_pid_actual_series);
    lv_chart_remove_series(m_pid_chart, m_pid_target_series);
    m_pid_actual_series = lv_chart_add_series(m_pid_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    m_pid_target_series = lv_chart_add_series(m_pid_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_refresh(m_pid_chart);
}

// Empty implementations for required methods
void CharacterizationView::updateCharts() {}
void CharacterizationView::showKsTest() {}
void CharacterizationView::showKvTest() {}
void CharacterizationView::showKaTest() {}
void CharacterizationView::showTrackWidthTest() {}

} // namespace tuning
