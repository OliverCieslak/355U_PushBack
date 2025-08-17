#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "motion/TrajectoryGenerator.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include "api.h"

// Simple straight-line pure pursuit test
void purePursuitStraightTest() {
    printf("Starting purePursuitStraightTest\n");
    units::Pose initialPose = units::Pose(0_in, 0_in, from_cDeg(0));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    std::vector<units::Pose> path = {
        initialPose,
        units::Pose(0_in, 36_in, from_cDeg(0)),
    };

    // Generate time-parameterized trajectory for forward leg
    motion::TrajectoryConfig config(24_inps, 36_inps2, 60_inps2); // max vel, accel, centripetal
    config.setStartVelocity(0_inps).setEndVelocity(0_inps).setReversed(false);
    auto trajFwd = motion::TrajectoryGenerator::generateTrajectory(path, config);

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
        8_in, 12_inps, 1.5_in
    );

    pp.setStanleyGains(Number(0.3), 2_inps, Number(1.0));
    pp.setTrajectory(trajFwd);
    pp.followPath(false, false);
    auto endPose = odometrySystem.getPose();
    printf("purePursuitStraightTest forward endPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
        to_in(endPose.x), to_in(endPose.y), to_stDeg(endPose.orientation), to_cDeg(endPose.orientation));

    // Wait 4 seconds then reversed trajectory (drive backwards)
    pros::delay(4000);
    motion::TrajectoryConfig configRev(24_inps, 36_inps2, 60_inps2);
    configRev.setStartVelocity(0_inps).setEndVelocity(0_inps).setReversed(true); // negative velocities
    auto trajRev = motion::TrajectoryGenerator::generateTrajectory(path, configRev);
    pp.setTrajectory(trajRev);
    pp.followPath(false, false); // reversed handled by velocity sign
    auto returnPose = odometrySystem.getPose();
    printf("purePursuitStraightTest return endPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
        to_in(returnPose.x), to_in(returnPose.y), to_stDeg(returnPose.orientation), to_cDeg(returnPose.orientation));
}
