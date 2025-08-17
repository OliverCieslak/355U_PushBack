#include "auton/CompetitionAutons.hpp"
#include "pros/adi.hpp"
#include "AntistallColorSort.hpp" // Add this include for SnailState definition
#include "motion/MotionProfilerRamseteController.hpp"
#include "motion/PurePursuitController.hpp"

void ramseteTest()
{
   printf("Starting ramseteTest\n");
    // Implement the skills autonomous routine here
    units::Pose initialPose = units::Pose(-57_in, -12_in, from_cDeg(180));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    enum class TestMode { Straight, GentleCurve, Full };
    TestMode mode = TestMode::Full;
    bool useSimpleStraightPath = (mode == TestMode::Straight);
    bool useGentleCurve = (mode == TestMode::GentleCurve);
    std::vector<units::Pose> path;
    if (useSimpleStraightPath) {
        path = {
            initialPose,
            units::Pose(-57_in, -40_in, from_cDeg(180)),
        };
        printf("Using simple straight diagnostic path\n");
    } else if (useGentleCurve) {
        // Gentle curve: small heading change and lateral shift to validate curvature sign & angular correction
        path = {
            initialPose,
            units::Pose(-50_in, -26_in, from_cDeg(170)),
            units::Pose(-45_in, -40_in, from_cDeg(165))
        };
        printf("Using gentle curve test path\n");
    } else if (mode == TestMode::Full) {
        path = {
            initialPose,
            units::Pose(-36_in, -58_in, from_cDeg(90)),
            units::Pose(29_in, -58_in, from_cDeg(90)),
            units::Pose(40_in, 37_in, from_cDeg(0)),
            units::Pose(-31_in, -59_in, from_cDeg(270)),
            units::Pose(-47_in, -58_in, from_cDeg(270)),
            units::Pose(-47_in, 12_in, from_cDeg(180)),
        };
    printf("Using full multi-turn path (Ramsete)\n");
    }

    printf("Generating traj");

    LinearVelocity limitedMaxVel = 4_inps;
    LinearAcceleration limitedMaxAccel = 4_inps2;
    printf("Max Accel: %.2f in/s²\n", to_inps2(limitedMaxAccel));
    printf("Max Centripetal Accel: %.2f in/s²\n", to_inps2(maxCentripetalAccel));
    // Temporarily limit max velocity for safety while tuning
    motion::TrajectoryConfig config(limitedMaxVel, limitedMaxAccel, maxCentripetalAccel);

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
    printf("Trajectory generated in %.2f ms\n", generation_time_ms);
    // Create Ramsete controller instance
    static motion::MotionProfilerRamseteController ramseteController(
        leftMotors,
        rightMotors,
        driveConfig,
        []()
        { return odometrySystem.getPose(); });

    // Print first 5 and last 5 trajectory states and curvature summary
    const auto& states = trajectory.getStates();
    int n = states.size();
    printf("Trajectory has %d states\n", n);
    double curvSum = 0.0; double curvMax = 0.0; int curvNonZero = 0;
    for (int i = 0; i < std::min(5, n); ++i) {
        printf("State %d: x=%.2f, y=%.2f, theta=%.2f, t=%.2fms, v=%.2f in/s, curv=%.4f rad/m\n", i,
            to_in(states[i].pose.x),
            to_in(states[i].pose.y),
            to_cDeg(states[i].pose.orientation),
            to_msec(states[i].timestamp),
            to_inps(states[i].velocity),
            to_radpm(states[i].curvature));
        double c = std::abs(to_radpm(states[i].curvature));
        curvSum += c; if (c > curvMax) curvMax = c; if (c > 1e-4) ++curvNonZero;
    }
    for (int i = std::max(0, n-5); i < n; ++i) {
        printf("State %d: x=%.2f, y=%.2f, theta=%.2f, t=%.2fms, v=%.2f in/s, curv=%.4f rad/m\n", i,
            to_in(states[i].pose.x),
            to_in(states[i].pose.y),
            to_cDeg(states[i].pose.orientation),
            to_msec(states[i].timestamp),
            to_inps(states[i].velocity),
            to_radpm(states[i].curvature));
        double c = std::abs(to_radpm(states[i].curvature));
        curvSum += c; if (c > curvMax) curvMax = c; if (c > 1e-4) ++curvNonZero;
    }
    if (n > 0) {
        printf("Curvature summary: avg=%.6f max=%.6f nonZeroStates=%d of %d\n", curvSum / n, curvMax, curvNonZero, n);
    }
    // Follow with Ramsete only
    ramseteController.setTrajectory(trajectory);
    ramseteController.followTrajectory();
    printf("Ramsete test completed\n");
}