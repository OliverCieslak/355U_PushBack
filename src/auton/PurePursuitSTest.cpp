#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "api.h"

// S-shape pure pursuit test (gentle curves)
void purePursuitSTest() {
    printf("Starting purePursuitSTest\n");
    units::Pose initialPose = units::Pose(0_in, 0_in, from_cDeg(0));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    // Capture and print odometry starting pose after reset
    {
        auto startPose = odometrySystem.getPose();
        printf("purePursuitSTest startPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
            to_in(startPose.x), to_in(startPose.y), to_stDeg(startPose.orientation), to_cDeg(startPose.orientation));
    }

    std::vector<units::Pose> path = {
        units::Pose(0_in,   0_in,   from_cDeg(0.0)),    // start
        units::Pose(12_in,  8_in,   from_cDeg(40.0)),   // swing right
        units::Pose(2_in,  16_in,   from_cDeg(340.0)),  // swing left
        units::Pose(18_in, 24_in,   from_cDeg(55.0)),   // swing right
        units::Pose(6_in,  32_in,   from_cDeg(325.0)),  // swing left
        units::Pose(20_in, 40_in,   from_cDeg(0.0))    // exit heading
    };
    printf("Stronger S-path poses (%zu) defined manually.\n", path.size());
    // Generate time-parameterized trajectory (forward)
    // Gentler accel for smoother starts; using maxAccel=12 in/s^2 (acts as slew limiter via time parameterization)
    motion::TrajectoryConfig config(20_inps, 12_inps2, 40_inps2); // max vel, accel, centripetal accel
    config.setCurvatureScale(0.6); // soften curvature to reduce over-rotation / veer
    config.setStartVelocity(0_inps).setEndVelocity(0_inps).setReversed(false);
    auto trajFwd = motion::TrajectoryGenerator::generateTrajectory(path, config);
    auto &statesFwd = trajFwd.getStates();

    control::DifferentialDriveConfig driveConfig(
        std::max(1.0_in, trackWidth),
        std::max(1.0_in, wheelDiameter),
        kV,
        kA,
        kS);

    static motion::PurePursuitController pp(
        leftMotors,
        rightMotors,
        driveConfig,
        [](){ return odometrySystem.getPose(); },
        8_in, 1.5_in
    );

    pp.setRequireFinalHeading(true); // keep explicit to ensure heading enforced in this test
    pp.setTrajectory(trajFwd);
    pp.followPath(false, false);
    pros::delay(500); // let it settle
    auto endPose = odometrySystem.getPose();
    printf("purePursuitSTest endPose   x=%.2f y=%.2f hComp=%.1f\n",
        to_in(endPose.x), to_in(endPose.y), to_cDeg(endPose.orientation));

    // Reverse test temporarily disabled to focus on forward trajectory quality
    // (Re-enable after tuning dynamic lookahead & curvature smoothing.)
}
