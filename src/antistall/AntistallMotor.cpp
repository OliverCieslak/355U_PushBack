#include "antistall/AntistallMotor.hpp"

namespace antistall {

void AntistallMotor::doAntistall() {
    uint32_t currentTime = pros::millis();
    
    // Check for stall every STALL_CHECK_INTERVAL ms
    if (currentTime - lastAntiStallTime >= STALL_CHECK_INTERVAL) {
        lastAntiStallTime = currentTime;
        
        // If not currently jiggling and motor is stalled, start jiggling
        if (!isJiggling && isMotorStalled() && abs(targetPower) > 0.1) {
            isJiggling = true;
            jiggleDirection = true;
            jiggleStartTime = currentTime;
            jiggleCount = 0;
            return;
        }
    }
    
    // Handle jiggling sequence
    if (isJiggling) {
        uint32_t jiggleElapsed = currentTime - jiggleStartTime;
        
        if (jiggleElapsed >= JIGGLE_DURATION) {
            // Switch direction or end jiggling
            if (jiggleDirection) {
                // Was moving forward, now move backward
                jiggleDirection = false;
                jiggleStartTime = currentTime;
            } else {
                // Was moving backward, now move forward and increment count
                jiggleDirection = true;
                jiggleStartTime = currentTime;
                jiggleCount++;
                
                // Check if we've completed enough jiggle cycles
                if (jiggleCount >= MAX_JIGGLE_CYCLES) {
                    isJiggling = false;
                    jiggleCount = 0;
                    // Return to normal operation
                    lemlib::Motor::move(targetPower);
                    return;
                }
            }
        }
        
        // Apply jiggle movement
        double jigglePower = jiggleDirection ? JIGGLE_POWER : -JIGGLE_POWER;
        lemlib::Motor::move(jigglePower);
        return;
    }
    
    // Normal operation - no jiggling needed
    lemlib::Motor::move(targetPower);
}

}