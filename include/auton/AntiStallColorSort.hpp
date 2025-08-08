#pragma once

#include "pros/apix.h"
#include "hardware/Motor/Motor.hpp"
#include "antistall/AntistallMotor.hpp"

extern antistall::AntistallMotor firstStageIntake;
extern antistall::AntistallMotor basketMotor;

//extern lemlib::Motor firstStageIntake;
//extern lemlib::Motor SecondStageIntake;
//extern lemlib::Motor basketChain;
//extern antistall::AntistallMotor basketMotor;
extern antistall::AntistallMotor secondStageIntake;
extern antistall::AntistallMotor basketChain;


extern double conveyorSpin;
// Anti-stall variables for firstStageIntake
extern uint32_t lastAntiStallTime;
extern bool isJiggling;
extern bool jiggleDirection;
extern uint32_t jiggleStartTime;
extern int jiggleCount;


enum class ConveyorState {
    REVERSED = -1,
    OFF = 0,
    SPINNING = 1
};
extern ConveyorState conveyorState;

enum class SnailState {
    OFF,
    INTAKE_TO_BASKET,
    SCORE_LOWER_CENTER,
    SCORE_UPPER_CENTER,
    SCORE_LONG_GOAL,
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
