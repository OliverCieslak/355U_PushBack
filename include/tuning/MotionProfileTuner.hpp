#pragma once

#include "hardware/Motor/MotorGroup.hpp"
#include "pros/apix.h"
#include "pros/misc.hpp"
#include "tuning/CharacterizationView.hpp"
#include "tuning/FeedforwardTuner.hpp"
#include "utils/Utils.hpp"

void tuneKs();
void tuneKv();
void tuneKa();
bool loadFeedForwardCalibrationValues();
void saveFeedForwardCalibrationValues();