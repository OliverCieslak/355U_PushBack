#pragma once

#include "robodash/api.h"
#include <vector>
#include <utility>

namespace tuning {

class CharacterizationView {
public:
    CharacterizationView();
    ~CharacterizationView();
    
    void updateKs(double kS);
    void updateKv(double kV);
    void updateKa(double kA);
    void updateTrackWidth(double trackWidth);
    void updateWheelDiameter(double wheelDiameter);
    
    void addVelocityDataPoint(double voltage, double velocity);
    void addAccelerationDataPoint(double voltage, double acceleration);
    
    void clearVelocityData();
    void clearAccelerationData();
    void clearPIDGraphData();
    
    void showKsTest();
    void showKvTest();
    void showKaTest();
    void showTrackWidthTest();
    
    void showStatusMessage(const char* title, const char* message);
    
    // New update methods for PID gains
    void updateLinearPID(double kP, double kI, double kD);
    void updateAngularPID(double kP, double kI, double kD);
    // New update method for PID graph
    void updatePIDGraph(double actual, double target);

private:
    void init();
    void updateCharts();
    
    rd_view_t* m_view = nullptr;
    lv_obj_t* m_tabview = nullptr;
    lv_obj_t* m_title_label = nullptr;
    
    // Parameter labels
    lv_obj_t* m_ks_label = nullptr;
    lv_obj_t* m_kv_label = nullptr;
    lv_obj_t* m_ka_label = nullptr;
    lv_obj_t* m_track_width_label = nullptr;
    lv_obj_t* m_wheel_diameter_label = nullptr;
    
    // Single status label for motor info
    lv_obj_t* m_motor_status = nullptr;
    
    // New PID label pointers
    lv_obj_t* m_linear_kp_label = nullptr;
    lv_obj_t* m_linear_ki_label = nullptr;
    lv_obj_t* m_linear_kd_label = nullptr;
    
    lv_obj_t* m_angular_kp_label = nullptr;
    lv_obj_t* m_angular_ki_label = nullptr;
    lv_obj_t* m_angular_kd_label = nullptr;
    
    // New members for Velocity chart
    lv_obj_t* m_velocity_chart = nullptr;
    lv_chart_series_t* m_velocity_series = nullptr;

    // New members for Acceleration chart
    lv_obj_t* m_acceleration_chart = nullptr;
    lv_chart_series_t* m_acceleration_series = nullptr;
    
    // New members for PID Graph tab
    lv_obj_t* m_pid_chart = nullptr;
    lv_chart_series_t* m_pid_actual_series = nullptr;
    lv_chart_series_t* m_pid_target_series = nullptr;
};

} // namespace tuning
