#include "auton/IntakeAndPistonState.hpp"
#include <cmath>

// Anti-stall constants


int colorSort = 0; // Flag to track color sorting state

/*void getAutonColorState()
{
    // Get the color detected by the sorting sensor
    autonColorSensor.set_led_pwm(100);
    double color = autonColorSensor.get_hue();
    topColorSortingSensor.set_led_pwm(100);
    lowColorSortingSensor.set_led_pwm(100);
    // Determine alliance color based on the detected color
    if (color >= 355 && color <= 10) { // 1 typically represents red
        allianceColor = AllianceColor::RED;
    } else if (color >= 210 && color <= 270) { // 2 typically represents blue
        allianceColor = AllianceColor::BLUE;
    } else {
        allianceColor = AllianceColor::OFF; // No valid color detected
    }
}
*/
// Continuously update the detected ball color in a background task
AllianceColor detectedBallColor = AllianceColor::OFF;

int LOW_BLUE_HUE = 210;
int HIGH_BLUE_HUE = 270;
int LOW_RED_HUE = 355;
int HIGH_RED_HUE = 30;

/*AllianceColor topballColor() {
    double hue = topColorSortingSensor.get_hue();
    int proximity = topColorSortingSensor.get_proximity();
    printf("Top Color Sensor - Hue: %.2f, Proximity: %d\n", hue, proximity);
    if (proximity < 30) {
        return AllianceColor::OFF;
    }
    if(hue >= LOW_RED_HUE && hue <= HIGH_RED_HUE) {
        return AllianceColor::RED;
    } else if (hue >= LOW_BLUE_HUE && hue <= HIGH_BLUE_HUE) {
        return AllianceColor::BLUE;
    } else {
        return AllianceColor::OFF;
    }
}

AllianceColor lowballColor() {
    double hue = lowColorSortingSensor.get_hue();
    if(hue >= LOW_RED_HUE && hue <= HIGH_RED_HUE) {
        return AllianceColor::RED;
    } else if (hue >= LOW_BLUE_HUE && hue <= HIGH_BLUE_HUE) {
        return AllianceColor::BLUE;
    } else {
        return AllianceColor::OFF;
    }
}
*/
void intakeAntiStallColorSort()
{
    SnailState actualSnailState = snailState;
    /*if(colorSortState != ColorSortState::OFF && actualSnailState != SnailState::OFF) {
        printf("Color Sort State: %d\n", (int)colorSortState);
        AllianceColor topColor = topballColor();
        AllianceColor lowColor = lowballColor();
        if (topColor != AllianceColor::OFF || lowColor != AllianceColor::OFF) {
            if((int)topColor != (int)(int)colorSortState || (int)lowColor != (int) colorSortState) {
                actualSnailState = SnailState::Long;
            }
        }
    }
    */
    switch(wingState)
    {

        case WingState::LEFTUP:
            WingLeft.set_value(true);
            WingRight.set_value(false);
            break;
        case WingState::DOWN:
            WingLeft.set_value(false);
            WingRight.set_value(false);
            break;
        case WingState::RIGHTUP:
            WingLeft.set_value(false);
            WingRight.set_value(true);
            break;
    }
    switch (actualSnailState)
    {
        case SnailState::OFF:
			firstStageIntake.move(0);
			
			secondStageIntake.move(0);
            
            break;
        case SnailState::Index:
			
			
			secondStageIntake.move(0.0);
            firstStageIntake.move(-1.0);
            break;
        case SnailState::Out:
			
			
			secondStageIntake.move(-1.0);
            firstStageIntake.move(1.0);
            break;
        case SnailState::Middle:
			
			
			secondStageIntake.move(1.0);
            firstStageIntake.move(-1.0);
            break;
        case SnailState::Long:
			
			
			secondStageIntake.move(1.0);
            firstStageIntake.move(-1.0);
            break;
    }
}

