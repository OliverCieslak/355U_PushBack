#pragma once

#include "liblvgl/lvgl.h"
#include "motion/TrajectoryGenerator.hpp"
#include "robodash/api.h"
#include "units/units.hpp"
#include "utils/DistanceUtils.hpp"
#include <vector>

namespace viz {

class FieldView {
public:
    // Define particle struct for storing particle data
    static const size_t MAX_PARTICLES = 25; // Maximum number of particles to display
    
    struct ParticleData {
        double x;
        double y;
        double theta;
        uint8_t intensity;
    };

public:
    // Constructor and destructor
    FieldView();
    ~FieldView();
    
    // Initialize the view
    void init();
    
    // Draw the trajectory on the canvas
    void drawTrajectory(const motion::Trajectory& trajectory, double generation_time_ms = 0.0);
    
    // Update the robot's position visualization
    void updateRobotPosition(const units::Pose& pose, Time t = 0.0_msec);
    
    // Get the screen object for display
    lv_obj_t* getScreen() const { return rd_view_obj(m_view); }
    
    /**
     * @brief Add a particle to be visualized on the field
     * 
     * @param x X position in inches
     * @param y Y position in inches
     * @param theta Orientation in degrees
     * @param intensity Color intensity (0-255), used for particle weight visualization
     */
    void addParticle(double x, double y, double theta, uint8_t intensity);
    
    /**
     * @brief Add multiple particles at once for batch visualization
     * 
     * @param particles Vector of ParticleData to be visualized on the field
     */
    void addParticles(const std::vector<ParticleData>& particles);
    
    /**
     * @brief Clear all particles from the visualization
     */
    void clearParticles();
    
    /**
     * @brief Draw all stored particles on the field
     */
    void drawParticles();
    
    /**
     * @brief Set the robot pose for visualization
     * 
     * @param x X position in inches
     * @param y Y position in inches
     * @param theta Orientation in degrees
     * @param isEstimated Whether this is the estimated pose (true) or odometry pose (false)
     */
    void setRobotPose(double x, double y, double theta, bool isEstimated = false);

private:
    std::vector<ParticleData> m_particles; // Vector to store particles
    
    // Helper function to convert field coordinates to screen coordinates
    void fieldToScreen(double x_in, double y_in, int& x_px, int& y_px);
    
    // Helper to draw field elements (border, axes, landmarks)
    void drawFieldElements();

    // Field dimensions for display scaling
    static const int FIELD_WIDTH_PX = 240;
    static const int FIELD_HEIGHT_PX = 240;
    static constexpr double FIELD_WIDTH_IN = to_in(utils::fieldWidth);
    static constexpr double FIELD_HEIGHT_IN = to_in(utils::fieldHeight);
    
    // LVGL objects
    rd_view_t* m_view = nullptr;  // RoboDash view
    lv_obj_t* m_trajectory_canvas = nullptr;
    lv_obj_t* m_robot_marker = nullptr;
    lv_color_t* m_canvas_buf = nullptr;
    lv_color_t m_canvas_buf_data[FIELD_WIDTH_PX * FIELD_HEIGHT_PX];
    
    // Info panel and labels
    lv_obj_t* m_info_panel = nullptr;
    lv_obj_t* m_gen_time_label = nullptr;
    lv_obj_t* m_points_label = nullptr;
    lv_obj_t* m_total_time_label = nullptr;
    
    // For tracking robot position history
    int m_prev_robot_x_px{-1};
    int m_prev_robot_y_px{-1};
    bool m_first_pose{true};
};

} // namespace viz