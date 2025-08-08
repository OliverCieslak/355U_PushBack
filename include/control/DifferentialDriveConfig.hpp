#pragma once

#include "units/units.hpp"

namespace control {

/**
 * @brief Configuration for a differential drive robot
 */
struct DifferentialDriveConfig {
    Length trackWidth;           // Distance between left and right wheels
    Length wheelDiameter;        // Diameter of wheels
    Number kV;                   // Velocity feedforward constant (volts per velocity)
    Number kA;                   // Acceleration feedforward constant (volts per acceleration)
    Number kS;                   // Static friction feedforward constant (volts)
    
    /**
     * @brief Construct a differential drive configuration
     * 
     * @param trackWidth Distance between left and right wheels
     * @param wheelDiameter Diameter of the wheels
     * @param kV Velocity feedforward constant
     * @param kA Acceleration feedforward constant
     * @param kS Static friction feedforward constant
     */
    DifferentialDriveConfig(
        Length trackWidth, 
        Length wheelDiameter,
        Number kV = Number(1.0),
        Number kA = Number(0.0),
        Number kS = Number(0.0)
    ) : trackWidth(trackWidth), wheelDiameter(wheelDiameter), kV(kV), kA(kA), kS(kS) {}
};

}