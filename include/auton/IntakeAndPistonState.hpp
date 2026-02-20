#pragma once

#include "pros/apix.h"
#include "hardware/Motor/Motor.hpp"
#include "antistall/AntistallMotor.hpp"

extern antistall::AntistallMotor firstStageIntake;


//extern lemlib::Motor firstStageIntake;
extern antistall::AntistallMotor secondStageIntake;
//extern lemlib::Motor basketChain;
//extern antistall::AntistallMotor basketMotor;
extern pros::adi::DigitalOut WingLeft;


extern double conveyorSpin;

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

extern ColorSortState colorSortState;
