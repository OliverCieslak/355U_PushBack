#include "auton/PathTestRoutine.hpp"

// External reference to trajectory view defined in main.cpp
extern viz::FieldView fieldView;
extern rd::Console console;

extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;
extern lemlib::V5InertialSensor imu;
extern odometry::SkidSteerOdometry odometrySystem;

extern Length trackWidth;
extern Length wheelDiameter;
extern Number kS;
extern Number kV;
extern Number kA;
extern Mass robotMass;
extern Torque driveTrainTorque;
extern LinearVelocity maxVelocity;
extern LinearAcceleration maxAccel;
extern LinearAcceleration maxCentripetalAccel;

void genPathTest() {
  printf("Gen Path test...");

  // Set initial robot position
    units::Pose initialPose = units::Pose(-46_in, -12_in, from_cDeg(180));
    odometrySystem.resetPose(initialPose);
    // odometrySystem.start();

  // Create a path with waypoints
  /*
  std::vector<units::Pose> path = {
      initialPose,
      units::Pose(48_in, 48_in, 90_cDeg),
      units::Pose(-48_in, 48_in, 180_cDeg),
      units::Pose(-48_in, -48_in, 270_cDeg),
      units::Pose(48_in, -48_in, 0_cDeg),
      units::Pose(0_in, 0_in, -45_cDeg),
  };
  */
  std::vector<units::Pose> path = {
      initialPose,
      units::Pose(-36_in, -58_in, from_cDeg(90)),
      units::Pose(29_in, -58_in, from_cDeg(90)),
      units::Pose(40_in, 37_in, from_cDeg(0)),
      units::Pose(-31_in, -59_in, from_cDeg(270)),
      units::Pose(-47_in, -58_in, from_cDeg(270)),
      units::Pose(-47_in, 12_in, from_cDeg(180)),
  };
  
  printf("Generating traj");

  printf("Max Accel: %.2f in/s²\n", to_inps2(maxAccel));
  printf("Max Centripetal Accel: %.2f in/s²\n", to_inps2(maxCentripetalAccel));
  motion::TrajectoryConfig config(maxVelocity, maxAccel, maxCentripetalAccel);

  // Create drive configuration with safety bounds
  control::DifferentialDriveConfig driveConfig(
      std::max(1.0_in, trackWidth),    // Ensure non-zero track width
      std::max(1.0_in, wheelDiameter), // Ensure non-zero wheel diameter
      kV,
      kA,
      kS);

  // Generate and follow trajectory
  printf("Generating traj");

  // Measure trajectory generation time
  uint32_t start_time = pros::micros();

  motion::Trajectory trajectory = motion::TrajectoryGenerator::generateTrajectory(path, config);

  uint32_t end_time = pros::micros();
  uint32_t generation_time_us = end_time - start_time;
  double generation_time_ms = generation_time_us / 1000.0;
  
  // Print trajectory information to standard output
  printf("\n==== TRAJECTORY INFORMATION ====\n");
  printf("Generation time: %.2f ms\n", generation_time_ms);
  printf("Total time: %.2f seconds\n", to_sec(trajectory.getTotalTime()));
  
  printf("\nTrajectory Samples:\n");
  printf("Time (s) | X (in) | Y (in) | Heading (deg) | Vel (in/s) | Accel (in/s²) | Curv (rad/m)\n");
  printf("-------------------------------------------------------------------\n");           
  for (int i = 0; i < trajectory.getStates().size(); i++) {
      auto state = trajectory.getStates()[i];
      
      printf("%.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f\n",
              to_msec(state.timestamp), 
              to_in(state.pose.x),
              to_in(state.pose.y),
              to_cDeg(state.pose.orientation),
              to_inps(state.velocity),
              to_inps2(state.acceleration),
              to_radpm(state.curvature)
            );
  }
  printf("==== END TRAJECTORY INFO ====\n\n");

  // Visualize the generated trajectory using FieldView, passing the generation time
  fieldView.drawTrajectory(trajectory, generation_time_ms);
}

void runPathTest() {
    // Set motor brake modes
    leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
    rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);
    
    printf("Path test...");

    // Set initial robot position
    units::Pose startingPosition = units::Pose(0_in, 0_in, 45_cDeg);
    odometrySystem.resetPose(startingPosition);

    // Create a path with waypoints
    std::vector<units::Pose> path = {
        startingPosition,
        units::Pose(48_in, 48_in, 90_cDeg),
        units::Pose(-48_in, 48_in, 180_cDeg),
        units::Pose(-48_in, -48_in, 270_cDeg),
        units::Pose(48_in, -48_in, 0_cDeg),
        units::Pose(0_in, 0_in, -45_cDeg),
    };

    printf("Generating trajectory");
    motion::TrajectoryConfig config(maxVelocity, maxAccel, maxCentripetalAccel);

    // Create drive configuration with safety bounds
    control::DifferentialDriveConfig driveConfig(
        std::max(1.0_in, trackWidth),    // Ensure non-zero track width
        std::max(1.0_in, wheelDiameter), // Ensure non-zero wheel diameter
        kV,
        kA,
        kS);

    // Create pose provider function using odometry system
    std::function<units::Pose()> poseProvider = []() {
        // Get the latest pose from the odometry system
        return odometrySystem.getPose();
    };

    printf("Creating profiler");
    // Create motion profiler
    motion::MotionProfilerRamseteController profiler(leftMotors, rightMotors, driveConfig, poseProvider);

    // Generate and follow trajectory
    printf("Generating traj");

    // Measure trajectory generation time
    uint32_t start_time = pros::micros();

    motion::Trajectory trajectory = motion::TrajectoryGenerator::generateTrajectory(path, config);

    uint32_t end_time = pros::micros();
    uint32_t generation_time_us = end_time - start_time;
    double generation_time_ms = generation_time_us / 1000.0;

    // Print trajectory information to standard output
    printf("\n==== TRAJECTORY INFORMATION ====\n");
    printf("Generation time: %.2f ms\n", generation_time_ms);
    printf("Total time: %.2f seconds\n", to_sec(trajectory.getTotalTime()));
    
    printf("\nTrajectory Samples:\n");
    printf("Time (s) | X (in) | Y (in) | Heading (deg) | Vel (in/s) | Accel (in/s²) | Curv (rad/m)\n");
    printf("-------------------------------------------------------------------\n");           
    for (int i = 0; i < trajectory.getStates().size(); i++) {
        auto state = trajectory.getStates()[i];
        
        printf("%.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f\n",
                to_msec(state.timestamp), 
                to_in(state.pose.x),
                to_in(state.pose.y),
                to_cDeg(state.pose.orientation),
                to_inps(state.velocity),
                to_inps2(state.acceleration),
                to_radpm(state.curvature)
              );
    }
    printf("==== END TRAJECTORY INFO ====\n\n");
    
    // Visualize the generated trajectory using FieldView, passing the generation time
    fieldView.drawTrajectory(trajectory, generation_time_ms);
    
    printf("Setting trajectory");
    profiler.setTrajectory(trajectory);
    
    printf("Following path");
    uint32_t startTime = pros::millis();
    uint32_t lastDisplayUpdateTime = startTime;
    
    // Use the new followTrajectory method asynchronously
    profiler.followTrajectory(true); // Run in async mode
    
    // Simple visualization loop with safety timeout
    uint32_t timeoutMs = (uint32_t)(to_sec(trajectory.getTotalTime()) * 1000 * 1.5); // 50% extra time
    
    while (profiler.isFollowing()) {
        uint32_t currentTime = pros::millis();
        
        // Safety timeout
        if (currentTime - startTime > timeoutMs) {
            printf("Timeout reached!");
            break;
        }
        
        // Update robot position on display every 100ms using FieldView
        if (currentTime - lastDisplayUpdateTime >= 100) {
            fieldView.updateRobotPosition(poseProvider(), from_msec(currentTime));
            lastDisplayUpdateTime = currentTime;
        }
        
        // Update status every second
        if (currentTime % 1000 < 10)
        {
          printf("%.1f/%.1fs", to_sec(profiler.getTrajectoryTime()), to_sec(trajectory.getTotalTime()));
        }
        
        pros::delay(10); // Minimize CPU usage while still being responsive
    }
    
    // Ensure motors are stopped
    leftMotors.move(0);
    rightMotors.move(0);
}
