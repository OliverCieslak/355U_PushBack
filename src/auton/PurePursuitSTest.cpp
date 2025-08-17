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

    // Define an S curve: rightward then leftward then rightward sweep.
    // Points spaced ~24" apart with +/-12" lateral offset.
    // Compass headings (0 = North, CW+) chosen to approximate path tangents:
    // Segment directions (standard frame):
    //  (0,0)->(24,12): 26.6° standard  => 63.4° compass
    //  (24,12)->(48,0): -26.6° standard => 116.6° compass
    //  (48,0)->(72,-12): -26.6° standard => 116.6° compass
    //  (72,-12)->(96,0): 26.6° standard => 63.4° compass
    // Internal waypoint headings are averaged for smoothness (use 90° compass where curvature changes sign).
    // Note: Pure Pursuit currently ignores waypoint orientation except optional final heading; these are for future use & clarity.
    std::vector<units::Pose> path = {
        units::Pose(0_in,   0_in,   from_cDeg(63.4)),  // start tangent
        units::Pose(24_in, 12_in,   from_cDeg(90.0)),  // inflection midpoint (avg of 63.4 & 116.6)
        units::Pose(48_in,  0_in,   from_cDeg(116.6)), // mid S heading
        units::Pose(72_in, -12_in,  from_cDeg(90.0)),  // second inflection
        units::Pose(96_in,  0_in,   from_cDeg(63.4)),  // end tangent
    };

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

    pp.setStanleyGains(Number(0.35), 2_inps, Number(1.0));
    pp.setPath(path);
    pp.followPath(false, false);
    auto endPose = odometrySystem.getPose();
    printf("purePursuitSTest forward endPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
        to_in(endPose.x), to_in(endPose.y), to_stDeg(endPose.orientation), to_cDeg(endPose.orientation));

    // Wait 4 seconds then construct reversed S path and drive forward along it
    pros::delay(4000);
    units::Pose endPoseCapture = endPose;
    std::vector<units::Pose> returnPath; returnPath.reserve(path.size());
    returnPath.push_back(endPoseCapture);
    for (int i = (int)path.size()-2; i >= 0; --i) {
        returnPath.push_back(units::Pose(path[i].x, path[i].y, path[i].orientation));
    }
    pp.setPath(returnPath);
    pp.followPath(false, false);
    auto returnPose = odometrySystem.getPose();
    printf("purePursuitSTest return endPose x=%.2f y=%.2f hStd=%.1f hComp=%.1f\n",
        to_in(returnPose.x), to_in(returnPose.y), to_stDeg(returnPose.orientation), to_cDeg(returnPose.orientation));
}
