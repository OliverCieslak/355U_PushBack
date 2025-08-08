#include "viz/FieldView.hpp"

namespace viz {

// Static members must be defined in the cpp file, not just declared in the header
const int FieldView::FIELD_WIDTH_PX;
const int FieldView::FIELD_HEIGHT_PX;
const double FieldView::FIELD_WIDTH_IN;
const double FieldView::FIELD_HEIGHT_IN;

FieldView::FieldView() {
    // Create a RoboDash view for trajectory visualization
    m_view = rd_view_create("Field View");
    init();
}

FieldView::~FieldView() {
    // RoboDash manages the view's lifecycle, so we don't need to delete it
    // Just set our pointer to null to avoid dangling reference
    m_view = nullptr;
}

void FieldView::init() {
    // Get the view's object to parent our LVGL objects
    lv_obj_t* parent = rd_view_obj(m_view);
    
    // Create a canvas for drawing
    m_canvas_buf = m_canvas_buf_data;
    
    // Create info panel to the left side
    m_info_panel = lv_obj_create(parent);
    lv_obj_set_size(m_info_panel, 100, FIELD_HEIGHT_PX - 20);
    lv_obj_set_style_bg_color(m_info_panel, lv_color_make(10, 10, 10), LV_PART_MAIN); // Dark background
    lv_obj_set_style_border_width(m_info_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(m_info_panel, lv_color_make(100, 100, 100), LV_PART_MAIN); // Gray border
    lv_obj_set_style_pad_all(m_info_panel, 5, LV_PART_MAIN);
    lv_obj_align(m_info_panel, LV_ALIGN_LEFT_MID, 5, 0);
    
    // Create labels for trajectory information
    m_gen_time_label = lv_label_create(m_info_panel);
    lv_obj_set_style_text_color(m_gen_time_label, lv_color_make(255, 255, 255), LV_PART_MAIN);
    lv_obj_align(m_gen_time_label, LV_ALIGN_TOP_LEFT, 0, 5);
    lv_label_set_text(m_gen_time_label, "Gen: -- ms");
    
    m_points_label = lv_label_create(m_info_panel);
    lv_obj_set_style_text_color(m_points_label, lv_color_make(255, 255, 255), LV_PART_MAIN);
    lv_obj_align(m_points_label, LV_ALIGN_TOP_LEFT, 0, 25);
    lv_label_set_text(m_points_label, "Pts: --");
    
    m_total_time_label = lv_label_create(m_info_panel);
    lv_obj_set_style_text_color(m_total_time_label, lv_color_make(255, 255, 255), LV_PART_MAIN);
    lv_obj_align(m_total_time_label, LV_ALIGN_TOP_LEFT, 0, 45);
    lv_label_set_text(m_total_time_label, "Time: -- s");
    
    // Create a canvas for drawing - position to the right of info panel
    m_trajectory_canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(m_trajectory_canvas, m_canvas_buf, FIELD_WIDTH_PX, FIELD_HEIGHT_PX, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(m_trajectory_canvas, LV_ALIGN_RIGHT_MID, -5, 0);
    lv_canvas_fill_bg(m_trajectory_canvas, lv_color_make(0, 0, 0), LV_OPA_COVER);
    
    // Calculate robot width in pixels (11 inches)
    int robot_width_px = static_cast<int>((11.0 / FIELD_WIDTH_IN) * FIELD_WIDTH_PX);
    int robot_height_px = robot_width_px; // Make it a square (11in x 11in)
    
    // Create robot marker (square to represent actual robot dimensions)
    m_robot_marker = lv_obj_create(parent);
    lv_obj_set_size(m_robot_marker, robot_width_px, robot_height_px);
    lv_obj_set_style_bg_color(m_robot_marker, lv_color_make(0, 255, 255), LV_PART_MAIN); // Cyan
    lv_obj_set_style_border_color(m_robot_marker, lv_color_make(255, 255, 255), LV_PART_MAIN); // White
    lv_obj_set_style_border_width(m_robot_marker, 1, LV_PART_MAIN);
    
    // Add rounded corners for a more visually pleasing look
    lv_obj_set_style_radius(m_robot_marker, 4, LV_PART_MAIN);
    
    // Add a small indicator at the front to show direction
    lv_obj_t *direction_indicator = lv_obj_create(m_robot_marker);
    lv_obj_set_size(direction_indicator, 6, 6);
    lv_obj_set_style_bg_color(direction_indicator, lv_color_make(255, 0, 0), LV_PART_MAIN); // Red
    lv_obj_set_style_radius(direction_indicator, 3, LV_PART_MAIN); // Circular
    lv_obj_set_style_border_width(direction_indicator, 0, LV_PART_MAIN); // No border
    lv_obj_align(direction_indicator, LV_ALIGN_TOP_MID, 0, 2); // Position at front center
    
    // Initialize tracking of robot poses
    m_prev_robot_x_px = -1;
    m_prev_robot_y_px = -1;
    m_first_pose = true;
    
    // Clear canvas and draw field elements
    lv_canvas_fill_bg(m_trajectory_canvas, lv_color_make(0, 0, 0), LV_OPA_COVER);
    drawFieldElements();
    
    // No need to call lv_scr_load as RoboDash handles view switching
}

void FieldView::drawFieldElements() {
    // Create line descriptor
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.width = 1;
    
    // Draw field border
    line_dsc.color = lv_color_make(128, 128, 128); // Gray
    lv_point_t border_pts[5] = {
        {0, 0},
        {FIELD_WIDTH_PX-1, 0},
        {FIELD_WIDTH_PX-1, FIELD_HEIGHT_PX-1},
        {0, FIELD_HEIGHT_PX-1},
        {0, 0}
    };
    
    // Draw border lines
    for (int i = 0; i < 4; i++) {
        lv_point_t line_pts[2] = {
            border_pts[i],
            border_pts[i+1]
        };
        lv_canvas_draw_line(m_trajectory_canvas, line_pts, 2, &line_dsc);
    }
    
    // Draw coordinate axes
    int center_x, center_y;
    fieldToScreen(0, 0, center_x, center_y);
    
    // Horizontal axis
    lv_point_t h_axis_pts[2] = {
        {0, (lv_coord_t)center_y},
        {FIELD_WIDTH_PX, (lv_coord_t)center_y}
    };
    lv_canvas_draw_line(m_trajectory_canvas, h_axis_pts, 2, &line_dsc);
    
    // Vertical axis
    lv_point_t v_axis_pts[2] = {
        {(lv_coord_t)center_x, 0},
        {(lv_coord_t)center_x, FIELD_HEIGHT_PX}
    };
    lv_canvas_draw_line(m_trajectory_canvas, v_axis_pts, 2, &line_dsc);
    
    // Draw the rotated square field element with 3-inch wide segments
    line_dsc.color = lv_color_make(255, 255, 0); // Yellow
    line_dsc.width = 3; // Make it thicker to represent the 3-inch width
    
    // Define the corner points of the square
    int corner_x_px[4], corner_y_px[4];
    fieldToScreen(0.0, 24.0, corner_x_px[0], corner_y_px[0]);   // Top
    fieldToScreen(24.0, 0.0, corner_x_px[1], corner_y_px[1]);   // Right
    fieldToScreen(0.0, -24.0, corner_x_px[2], corner_y_px[2]);  // Bottom
    fieldToScreen(-24.0, 0.0, corner_x_px[3], corner_y_px[3]);  // Left
    
    // Draw the square sides
    for (int i = 0; i < 4; i++) {
        lv_point_t square_pts[2] = {
            {(lv_coord_t)corner_x_px[i], (lv_coord_t)corner_y_px[i]},
            {(lv_coord_t)corner_x_px[(i+1)%4], (lv_coord_t)corner_y_px[(i+1)%4]}
        };
        lv_canvas_draw_line(m_trajectory_canvas, square_pts, 2, &line_dsc);
    }
}

void FieldView::drawTrajectory(const motion::Trajectory& trajectory, double generation_time_ms) {
    // Update info labels
    char buffer[32];
    sprintf(buffer, "Gen: %.1f ms", generation_time_ms);
    lv_label_set_text(m_gen_time_label, buffer);
    
    sprintf(buffer, "Pts: %zu", trajectory.getStates().size());
    lv_label_set_text(m_points_label, buffer);
    
    sprintf(buffer, "Time: %.2f s", to_sec(trajectory.getTotalTime()));
    lv_label_set_text(m_total_time_label, buffer);

    // Clear canvas
    lv_canvas_fill_bg(m_trajectory_canvas, lv_color_make(0, 0, 0), LV_OPA_COVER);
    
    // Draw field elements first
    drawFieldElements();
    
    // Create line descriptor
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.width = 1;
    
    // Draw the trajectory path
    const int NUM_SAMPLES = 100;
    lv_point_t prev_pt;
    bool first_point = true;
    
    line_dsc.color = lv_color_make(255, 0, 0); // Red for trajectory
    line_dsc.width = 1; // Back to 1px width for trajectory
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Sample the trajectory
        double t = i * (to_sec(trajectory.getTotalTime()) / (NUM_SAMPLES - 1));
        auto state = trajectory.sample(t * 1_sec);
        
        // Convert coordinates
        int x_px, y_px;
        fieldToScreen(to_in(state.pose.x), to_in(state.pose.y), x_px, y_px);
        
        // Handle potential narrowing conversion safely
        lv_point_t curr_pt;
        curr_pt.x = static_cast<lv_coord_t>(std::min(x_px, 32767));
        curr_pt.y = static_cast<lv_coord_t>(std::min(y_px, 32767));
        
        // Draw line segment
        if (!first_point) {
            lv_point_t line_pts[2] = {prev_pt, curr_pt};
            lv_canvas_draw_line(m_trajectory_canvas, line_pts, 2, &line_dsc);
        }
        
        prev_pt = curr_pt;
        first_point = false;
    }
    
    // Calculate half robot width in field inches
    double half_robot_width_in = 5.5; // 11/2 inches
    
    // Now draw left and right wheel paths in lighter color
    line_dsc.color = lv_color_make(200, 100, 100); // Lighter red
    line_dsc.width = 1;
    
    // Left wheel path
    first_point = true;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        double t = i * (to_sec(trajectory.getTotalTime()) / (NUM_SAMPLES - 1));
        auto state = trajectory.sample(t * 1_sec);
        
        // Get heading angle and use it to calculate left wheel position
        double theta = to_stRad(state.pose.orientation);
        
        // Calculate left wheel position (perpendicular to heading)
        // Corrected formula using perpendicular vector: x - half_width * cos(theta), y + half_width * sin(theta)
        double left_x = to_in(state.pose.x) - half_robot_width_in * cos(theta);
        double left_y = to_in(state.pose.y) + half_robot_width_in * sin(theta);
        
        int x_px, y_px;
        fieldToScreen(left_x, left_y, x_px, y_px);
        
        lv_point_t curr_pt;
        curr_pt.x = static_cast<lv_coord_t>(std::min(x_px, 32767));
        curr_pt.y = static_cast<lv_coord_t>(std::min(y_px, 32767));
        
        if (!first_point) {
            lv_point_t line_pts[2] = {prev_pt, curr_pt};
            lv_canvas_draw_line(m_trajectory_canvas, line_pts, 2, &line_dsc);
        }
        
        prev_pt = curr_pt;
        first_point = false;
    }
    
    // Right wheel path
    first_point = true;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        double t = i * (to_sec(trajectory.getTotalTime()) / (NUM_SAMPLES - 1));
        auto state = trajectory.sample(t * 1_sec);
        
        // Get heading angle and use it to calculate right wheel position
        double theta = to_stRad(state.pose.orientation);
        
        // Calculate right wheel position (perpendicular to heading)
        // Corrected formula using perpendicular vector: x + half_width * cos(theta), y - half_width * sin(theta)
        double right_x = to_in(state.pose.x) + half_robot_width_in * cos(theta);
        double right_y = to_in(state.pose.y) - half_robot_width_in * sin(theta);
        
        int x_px, y_px;
        fieldToScreen(right_x, right_y, x_px, y_px);
        
        lv_point_t curr_pt;
        curr_pt.x = static_cast<lv_coord_t>(std::min(x_px, 32767));
        curr_pt.y = static_cast<lv_coord_t>(std::min(y_px, 32767));
        
        if (!first_point) {
            lv_point_t line_pts[2] = {prev_pt, curr_pt};
            lv_canvas_draw_line(m_trajectory_canvas, line_pts, 2, &line_dsc);
        }
        
        prev_pt = curr_pt;
        first_point = false;
    }
    
    // Draw time markers at 5-second intervals
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color_make(255, 255, 255); // White color for time markers
    rect_dsc.radius = 2;
    rect_dsc.bg_opa = LV_OPA_COVER;
    
    // Define label descriptor for time labels
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_make(255, 255, 120); // Light yellow
    label_dsc.opa = LV_OPA_COVER;
    
    // Get total time of trajectory
    double totalTime = to_sec(trajectory.getTotalTime());
    
    // Calculate interval points (every 5 seconds)
    const int TIME_INTERVAL = 5; // seconds
    const int MARKER_SIZE = 4;   // pixels
    
    for (int t = 0; t <= static_cast<int>(totalTime); t += TIME_INTERVAL) {
        if (t > totalTime) break; // Don't go beyond trajectory time
        
        // Sample trajectory at this time
        auto state = trajectory.sample(t * 1_sec);
        
        // Convert coordinates to screen position
        int x_px, y_px;
        fieldToScreen(to_in(state.pose.x), to_in(state.pose.y), x_px, y_px);
        
        // Draw a square marker at this position
        // Using correct function signature: lv_canvas_draw_rect(canvas, x, y, w, h, dsc)
        lv_canvas_draw_rect(m_trajectory_canvas, 
                           x_px - MARKER_SIZE/2,  // x
                           y_px - MARKER_SIZE/2,  // y
                           MARKER_SIZE,           // width
                           MARKER_SIZE,           // height
                           &rect_dsc);            // style descriptor
        
        // Create text for time label
        char timeLabel[10];
        sprintf(timeLabel, "%ds", t);
        
        // Draw the time label to the right of the marker
        // Using correct parameters for lv_canvas_draw_text
        lv_point_t label_pos;
        label_pos.x = x_px + MARKER_SIZE;
        label_pos.y = y_px - MARKER_SIZE;
        
        lv_canvas_draw_text(m_trajectory_canvas, 
                           label_pos.x,   // x position
                           label_pos.y,   // y position
                           20,            // max width
                           &label_dsc,    // style descriptor
                           timeLabel);    // text to display
    }
}

void FieldView::updateRobotPosition(const units::Pose& pose, Time t) {
    if (!m_robot_marker) return;
    
    // Convert robot position to screen coordinates
    int x_px, y_px;
    fieldToScreen(to_in(pose.x), to_in(pose.y), x_px, y_px);
    
    // Get marker dimensions
    lv_coord_t width = lv_obj_get_width(m_robot_marker);
    lv_coord_t height = lv_obj_get_height(m_robot_marker);
    
    // Get orientation in radians
    double theta_rad = to_stRad(pose.orientation);
    
    // Center the robot marker on position
    lv_obj_set_pos(m_robot_marker, x_px - width/2, y_px - height/2);
    
    // Set the rotation of the marker to match robot orientation
    // LVGL 8.3 doesn't support direct rotation of objects, so we use transformation matrix
    lv_obj_set_style_transform_angle(m_robot_marker, (int16_t)(theta_rad * 1800.0 / M_PI), LV_PART_MAIN); // Convert radians to tenths of a degree
    lv_obj_set_style_transform_pivot_x(m_robot_marker, width/2, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_y(m_robot_marker, height/2, LV_PART_MAIN);
    
    // Draw line connecting previous position to current position
    if (!m_first_pose && m_prev_robot_x_px >= 0 && m_prev_robot_y_px >= 0) {
        lv_draw_line_dsc_t line_dsc;
        lv_draw_line_dsc_init(&line_dsc);
        line_dsc.width = 1; // Thin line
        line_dsc.color = lv_color_make(0, 255, 255); // Cyan
        
        // Create points array for connection line
        lv_point_t line_pts[2] = {
            {(lv_coord_t)m_prev_robot_x_px, (lv_coord_t)m_prev_robot_y_px},
            {(lv_coord_t)x_px, (lv_coord_t)y_px}
        };
        
        // Draw connecting line
        lv_canvas_draw_line(m_trajectory_canvas, line_pts, 2, &line_dsc);
    }
    
    // Create line descriptor for path trail
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.width = 2;
    line_dsc.color = lv_color_make(0, 255, 255); // Cyan, matching robot
    
    // Calculate heading vector endpoint
    int head_x = x_px + static_cast<int>(15 * cos(theta_rad));
    int head_y = y_px - static_cast<int>(15 * sin(theta_rad));
    
    // Create points array for heading line
    lv_point_t heading_pts[2] = {
        {(lv_coord_t)x_px, (lv_coord_t)y_px},
        {(lv_coord_t)head_x, (lv_coord_t)head_y}
    };
    
    // Draw heading line
    lv_canvas_draw_line(m_trajectory_canvas, heading_pts, 2, &line_dsc);
    
    // Display timestamp near the robot position
    char time_text[32];
    sprintf(time_text, "T: %d ms", (int)to_msec(t));
    
    // Create label descriptor
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_make(255, 255, 255); // White text
    label_dsc.opa = LV_OPA_COVER;
    
    // Draw timestamp text above the robot
    lv_canvas_draw_text(m_trajectory_canvas, 
                        x_px - 20,          // x position
                        y_px - height - 10, // y position - above the robot
                        40,                 // max width
                        &label_dsc,         // style descriptor
                        time_text);         // text to display
    
    // Store current position for next update
    m_prev_robot_x_px = x_px;
    m_prev_robot_y_px = y_px;
    m_first_pose = false;
}

void FieldView::fieldToScreen(double x_in, double y_in, int& x_px, int& y_px) {
    // Scale and convert coordinates (flip Y axis)
    x_px = static_cast<int>((x_in + FIELD_WIDTH_IN/2) * FIELD_WIDTH_PX / FIELD_WIDTH_IN);
    y_px = FIELD_HEIGHT_PX - static_cast<int>((y_in + FIELD_HEIGHT_IN/2) * FIELD_HEIGHT_PX / FIELD_HEIGHT_IN);
    
    // Clamp values to display bounds
    x_px = std::min(std::max(x_px, 0), FIELD_WIDTH_PX);
    y_px = std::min(std::max(y_px, 0), FIELD_HEIGHT_PX);
}

void FieldView::addParticle(double x, double y, double theta, uint8_t intensity) {
    // Create a single particle and add it using the batch method
    ParticleData particle = {x, y, theta, intensity};
    std::vector<ParticleData> particles = {particle};
    addParticles(particles);
}

void FieldView::addParticles(const std::vector<ParticleData>& particles) {
    // Always clear existing particles
    m_particles.clear();
    
    // Only add up to MAX_PARTICLES
    if (particles.size() <= MAX_PARTICLES) {
        // If we have fewer than MAX_PARTICLES, add all of them
        m_particles.assign(particles.begin(), particles.end());
    } else {
        // If we have more than MAX_PARTICLES, add only the most recent MAX_PARTICLES
        m_particles.assign(particles.end() - MAX_PARTICLES, particles.end());
    }
    
    // Redraw all stored particles
    drawParticles();
}

void FieldView::drawParticles() {
    // Clear existing particles by redrawing field elements
    drawFieldElements();
    
    // Draw all particles in our storage
    for (const auto& particle : m_particles) {
        // Convert field coordinates to screen coordinates for this particle
        int screen_x, screen_y;
        fieldToScreen(particle.x, particle.y, screen_x, screen_y);
        
        // Use HSV color format for particles with varying saturation based on weight
        lv_color_t particleColor = lv_color_hsv_to_rgb(270, // Purple hue (270°)
                                                     std::min(100, static_cast<int>(particle.intensity * 100 / 255)),
                                                     100); // Full brightness
        
        // Draw a small rectangle for the particle
        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = particleColor;
        rect_dsc.radius = 2;
        rect_dsc.bg_opa = LV_OPA_COVER;
        
        // Draw a small filled rectangle with rounded corners
        lv_canvas_draw_rect(m_trajectory_canvas, 
                            screen_x - 2,  // x position
                            screen_y - 2,  // y position
                            4,             // width
                            4,             // height
                            &rect_dsc);    // style descriptor
        
        // Draw orientation line
        int line_end_x = screen_x + static_cast<int>(6 * cos(particle.theta * M_PI / 180.0));
        int line_end_y = screen_y - static_cast<int>(6 * sin(particle.theta * M_PI / 180.0));
        
        // Prepare points array for the line
        lv_point_t line_points[2] = {
            {static_cast<lv_coord_t>(screen_x), static_cast<lv_coord_t>(screen_y)},
            {static_cast<lv_coord_t>(line_end_x), static_cast<lv_coord_t>(line_end_y)}
        };
        
        // Prepare line descriptor
        lv_draw_line_dsc_t line_dsc;
        lv_draw_line_dsc_init(&line_dsc);
        line_dsc.color = particleColor;
        line_dsc.width = 1;
        
        // Draw the orientation line
        lv_canvas_draw_line(m_trajectory_canvas, line_points, 2, &line_dsc);
    }
    
    // If we have a previous robot position, redraw it
    if (m_prev_robot_x_px >= 0 && m_prev_robot_y_px >= 0 && !m_first_pose) {
        // We would redraw the robot trail here if needed
    }
}

void FieldView::clearParticles() {
    // Clear the particle vector
    m_particles.clear();
    
    // Redraw the field elements to clear particles visually
    drawFieldElements();
    
    // If we have a previous robot position, redraw it
    if (m_prev_robot_x_px >= 0 && m_prev_robot_y_px >= 0) {
        // Redraw the robot marker
        lv_obj_set_pos(m_robot_marker, m_prev_robot_x_px - 5, m_prev_robot_y_px - 5);
        lv_obj_invalidate(m_robot_marker);
    }
}

void FieldView::setRobotPose(double x, double y, double theta, bool isEstimated) {
    // Convert field coordinates to screen coordinates
    int screen_x, screen_y;
    fieldToScreen(x, y, screen_x, screen_y);
    
    // Use HSV color model: green (120°) for estimated pose, cyan (180°) for odometry pose
    lv_color_t poseColor = isEstimated ? 
                           lv_color_hsv_to_rgb(120, 100, 100) : // Bright green for estimated pose
                           lv_color_hsv_to_rgb(180, 100, 100);  // Cyan for odometry pose
    
    // Create a robot marker if it doesn't exist
    if (m_robot_marker == nullptr) {
        m_robot_marker = lv_obj_create(lv_disp_get_scr_act(NULL));
        lv_obj_set_size(m_robot_marker, 10, 10);
        lv_obj_set_style_bg_color(m_robot_marker, poseColor, 0);
        lv_obj_set_style_border_width(m_robot_marker, 1, 0);
        lv_obj_set_style_border_color(m_robot_marker, lv_color_make(0, 0, 0), 0);
    } else {
        // Update color based on whether this is estimated or odometry pose
        lv_obj_set_style_bg_color(m_robot_marker, poseColor, 0);
    }
    
    // Position the marker
    lv_obj_set_pos(m_robot_marker, screen_x - 5, screen_y - 5);
    
    // Rotate marker to show orientation
    // For LVGL 8, use lv_obj_set_style_transform_angle
    // For earlier versions, might need to draw orientation line separately
    
    // Store current position for history tracking
    m_prev_robot_x_px = screen_x;
    m_prev_robot_y_px = screen_y;
    m_first_pose = false;
}

} // namespace viz
