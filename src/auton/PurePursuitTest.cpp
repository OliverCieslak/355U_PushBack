// Pure pursuit specific autonomous test using same full path as Ramsete test
#include "auton/CompetitionAutons.hpp"
#include "motion/PurePursuitController.hpp"
#include "units/Pose.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include "control/DifferentialDriveConfig.hpp"
#include <algorithm>
#include "motion/TrajectoryGenerator.hpp"

void purePursuitTest() {
    printf("Starting purePursuitTest\n");
    units::Pose initialPose = units::Pose(-57_in, -12_in, from_cDeg(180));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();

    // Reuse full multi-turn path from ramseteTest
    auto c = [](double deg){ return from_cDeg(deg); }; // helper
    std::vector<units::Pose> path = {
        initialPose,
        units::Pose(-36_in, -58_in, c(90)),
        units::Pose(29_in, -58_in, c(90)),
        units::Pose(40_in, 37_in, c(0)),
        units::Pose(-31_in, -59_in, c(270)),
        units::Pose(-47_in, -58_in, c(270)),
        units::Pose(-47_in, 12_in, c(180)),
    };

    // Drive configuration (mirror RamseteTest setup)
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

    // Optional Stanley blending (light correction)
    pp.setStanleyGains(Number(0.4), 2_inps, Number(1.0));

    // Choose forward or reversed travel
    bool driveReversed = false; // set true to back along path

    pp.setPath(path);
    pp.followPath(false, driveReversed);

    auto endPose = odometrySystem.getPose();
    printf("Pure pursuit test completed endPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
        to_in(endPose.x), to_in(endPose.y), to_stDeg(endPose.orientation), to_cDeg(endPose.orientation));
}