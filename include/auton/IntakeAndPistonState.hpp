#pragma once

#include "pros/apix.h"
#include "hardware/Motor/Motor.hpp"
#include "antistall/AntistallMotor.hpp"

extern antistall::AntistallMotor firstStageIntake;


//extern lemlib::Motor firstStageIntake;
//extern lemlib::Motor SecondStageIntake;
//extern lemlib::Motor basketChain;
//extern antistall::AntistallMotor basketMotor;
extern antistall::AntistallMotor secondStageIntake;
extern pros::ADIDigitalOut WingRight;
extern pros::ADIDigitalOut WingLeft;


extern double conveyorSpin;
// Anti-stall variables for firstStageIntake
extern uint32_t lastAntiStallTime;
extern bool isJiggling;
extern bool jiggleDirection;
extern uint32_t jiggleStartTime;
extern int jiggleCount;


enum class WingState {
    LEFTUP = -1,
    DOWN = 0,
    RIGHTUP = 1
};
extern WingState wingState;

enum class SnailState {
    OFF,
    Index,
    Out,
    Middle,
    Long,
};
extern SnailState snailState;
enum class AllianceColor {
    BLUE ,
    RED ,
    OFF 
};
extern AllianceColor allianceColor;
enum class ColorSortState {
    OFF,
    RED,
    BLUE
};
void intakeAntiStallColorSort();

void getAutonColorState();

extern pros::Optical lowColorSortingSensor;
extern pros::Optical topColorSortingSensor;
extern pros::Optical autonColorSensor;
extern ColorSortState colorSortState;
