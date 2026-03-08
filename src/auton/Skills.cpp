#include "auton/CompetitionAutons.hpp"
#include "pros/adi.hpp"
#include "auton/IntakeAndPistonState.hpp" // Add this include for SnailState definition
#include "control/DifferentialDriveConfig.hpp"

void autonSkills()
{
    units::Pose initialPose = units::Pose(47_in, -12_in, from_cDeg(180));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    uint32_t t0 = pros::millis();

    wingState = WingState::LEFTUP;

    auto pose = odometrySystem.getPose();
    printf("[%5lums] x=%.1f y=%.1f hC=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));

    scraperPiston.set_value(true);
    snailState = SnailState::Index; // Get ready to match load
    pidDriveController.driveToPoint({47_in, -48_in}, 8.0, 1.75_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML approach x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1.5_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML turn East x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.driveDistance(14_in, 6.0, 1_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML intake drive x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pros::delay(200);
    snailState = SnailState::OFF;

    pidDriveController.driveToPoint({48_in, -48_in}, 8.0, 1.75_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML back out x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    scraperPiston.set_value(false);
    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML turn South x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.driveToPoint({48_in, -58_in}, 8.0, 1.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE corner clear x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(270), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] turn West for cross-field x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.driveToPoint({-32_in, -56_in}, 8.0, 5.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW corner arrived x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(0), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW corner turn North x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    // Drive directly to goal mouth in one move, then turn to face West into goal
    pidDriveController.driveToPoint({-32_in, -48_in}, 8.0, 2.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW_LG mouth x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW_LG turn East (back-in) x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    snailState = SnailState::Long;
    scraperPiston.set_value(true);
    pros::delay(1000);
    pidDriveController.driveToPoint({-56_in, -48_in}, 8.0, 2.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW_LG score load1 x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    snailState = SnailState::Index;
    pros::delay(500);
    pidDriveController.driveToPoint({-32_in, -48_in}, 8.0, 2.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW_LG pull back for load2 x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    snailState = SnailState::Long;
    scraperPiston.set_value(false);
    pros::delay(1000);

    pidDriveController.driveToPoint({-48_in, -48_in}, 8.0, 2.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SW_LG score load2 done, exit goal x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(0), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] turn North for cross-field x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.driveToPoint({-48_in, 48_in}, 8.0, 4.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NW corner arrived x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(-90), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NW_ML turn East x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    snailState = SnailState::Index;
    scraperPiston.set_value(true);
    pidDriveController.driveDistance(14_in, 6.0, 1_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NW_ML intake drive x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pros::delay(200);
    snailState = SnailState::OFF;

    pidDriveController.driveToPoint({-48_in, 48_in}, 8.0, 2.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NW_ML back out x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    scraperPiston.set_value(false);
    pidDriveController.turnToHeading(from_cDeg(0), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NW_ML turn North x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.driveToPoint({-48_in, 56_in}, 8.0, 2.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NW corner clear x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] turn East for cross-field x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.driveToPoint({40_in, 56_in}, 8.0, 4.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE corner arrived x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_ML turn South x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.driveToPoint({40_in, 48_in}, 8.0, 2.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_ML approach x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(90), 8.0, 1.5_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_ML turn East x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.driveToPoint({32_in, 48_in}, 8.0, 2.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_LG mouth x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    snailState = SnailState::Long;
    scraperPiston.set_value(true);
    pros::delay(1000);

    pidDriveController.driveToPoint({56_in, 48_in}, 8.0, 2.0_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_LG score load1 x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    snailState = SnailState::Index;
    pros::delay(500);

    pidDriveController.driveToPoint({32_in, 48_in}, 8.0, 2.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_LG pull back for load2 x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    snailState = SnailState::Long;
    scraperPiston.set_value(false);
    pros::delay(1000);

    pidDriveController.driveToPoint({52_in, 48_in}, 8.0, 2.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] NE_LG score load2 done x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(170), 8.0, 1.5_sec);
    pidDriveController.driveDistance(14_in, 6.0, 1_sec, true);
    pidDriveController.turnToHeading(from_cDeg(180), 8.0, 1.5_sec);
    scraperPiston.set_value(true);
    pidDriveController.driveToPoint({58_in, 0_in}, 8.0, 3.0_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML load3 done, END x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
}

void autonSkillsOneSide()
{
    units::Pose initialPose = units::Pose(47_in, 0_in, from_cDeg(0));
    odometrySystem.resetPose(initialPose);
    odometrySystem.start();
    uint32_t t0 = pros::millis();

    wingState = WingState::LEFTUP;

    auto pose = odometrySystem.getPose();
    printf("[%5lums] x=%.1f y=%.1f hC=%.1f\n",
           (unsigned long)(pros::millis() - t0),
           to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));

    scraperPiston.set_value(true);
    snailState = SnailState::Index; // Get ready to match load
    pidDriveController.driveToPoint({47_in, 47_in}, 8.0, 1.75_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML approach x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(90), 6.0, 2_sec, true);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML turn East x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pidDriveController.driveDistance(17_in, 6.0, 2_sec);
    {
        auto p = odometrySystem.getPose();
        printf("[%5lums] SE_ML intake drive x=%.2f y=%.2f h=%.1f\n",
               (unsigned long)(pros::millis() - t0), to_in(p.x), to_in(p.y), to_cDeg(p.orientation));
    }
    pros::delay(750);
    snailState = SnailState::OFF;

    pidDriveController.driveToPoint({30_in, 47_in}, 7.0, 2.5_sec, true, true);
    pidDriveController.driveDistance(-4_in, 6.0, 0.75_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] afterReverse x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    snailState = SnailState::Long; // Score LG
    pros::delay(1500);
    snailState = SnailState::OFF;

    pidDriveController.driveDistance(15_in, 8.0, 1.75_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] after First Score x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }

    pidDriveController.turnToHeading(from_cDeg(180), 6.0, 2_sec, true);
    pidDriveController.driveToPoint({40_in, -49_in}, 9.0, 5.5_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] after Long Drive x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    // scraperPiston.set_value(true);
    pidDriveController.turnToHeading(from_cDeg(90), 6.0, 2_sec, true);
    // back to LG to square
    pidDriveController.driveDistance(-20_in, 5.0, 1.75_sec);
    snailState = SnailState::Long; // Eject any remaining blocks
    pros::delay(500);
    snailState = SnailState::Index;
    pidDriveController.driveDistance(36_in, 5.0, 3.75_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] at 2nd Match Loader x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    pros::delay(750);

    pidDriveController.driveToPoint({30_in, -49_in}, 9.0, 2.0_sec, true);
    pidDriveController.driveDistance(-4_in, 6.0, 0.75_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] at 2nd Long Goal x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    snailState = SnailState::Long;
    pros::delay(1500);
    scraperPiston.set_value(false);

    pidDriveController.driveToPoint({66_in, -30_in}, 8.0, 3.5_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] arc mid x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(5), 6.0, 2_sec, true);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] arc end x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    pidDriveController.driveDistance(10_in, 6.0, 1.75_sec);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] wall smash x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    pidDriveController.turnToHeading(from_cDeg(-15), 8.0, 1.5_sec, true); // face exactly north
    // scraperPiston.set_value(true);
    snailState = SnailState::Long;
    pidDriveController.driveDistance(-15_in, 8.0, 2_sec);               // run-up
    pidDriveController.turnToHeading(from_cDeg(5), 8.0, .75_sec, true); // face exactly north
    // Raw motor voltage — no PID stall detection to abort the lip climb
    // move() takes -1.0 to 1.0 (same as voltage/12.0 used by PIDDriveController)
    // Zero the vertical tracking wheel so we measure only this drive segment
    verticalTrackingWheel.setAngle(0_stDeg);
    leftMotors.move(1.0);
    rightMotors.move(1.0);
    {
        // Stop once the tracking wheel measures >= 40 in traveled, 3 s safety cap
        // Using std::abs in case wheel direction is inverted
        constexpr double targetIn = 40.0;
        constexpr double wheelCircumIn = 3.14159265 * 2.75; // π × 2.75" diameter
        uint32_t driveStart = pros::millis();
        while (pros::millis() - driveStart < 3000) {
            double distIn = std::abs(to_stDeg(verticalTrackingWheel.getAngle()))
                            / 360.0 * wheelCircumIn;
            if (distIn >= targetIn) break;
            pros::delay(20);
        }
    }
    leftMotors.move(0);
    rightMotors.move(0);
    {
        auto pose = odometrySystem.getPose();
        printf("[%5lums] park x=%.2f y=%.2f hComp=%.1f\n",
               (unsigned long)(pros::millis() - t0),
               to_in(pose.x), to_in(pose.y), to_cDeg(pose.orientation));
    }
    pros::delay(1000);
    snailState = SnailState::OFF;
}
