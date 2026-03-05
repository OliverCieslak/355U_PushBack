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
}