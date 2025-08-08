#pragma once

#include "hardware/Motor/Motor.hpp"
#include "hardware/Port.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"

namespace antistall {


class AntistallMotor : public lemlib::Motor {

    public:
        AntistallMotor(lemlib::ReversibleSmartPort port, AngularVelocity outputVelocity)
            : lemlib::Motor(port, outputVelocity) {};

        AntistallMotor(lemlib::ReversibleSmartPort port, 
                AngularVelocity outputVelocity,
                Current stallCurrentThreshold,
                AngularVelocity stallVelocityThreshold,
                double jigglePower = 1.0,
                uint32_t jiggleDuration = 400,
                uint32_t maxJiggleCycles = 3)
            : lemlib::Motor(port, outputVelocity),
                STALL_CURRENT_THRESHOLD(stallCurrentThreshold),
                STALL_VELOCITY_THRESHOLD(stallVelocityThreshold),
                JIGGLE_POWER(jigglePower),
                JIGGLE_DURATION(jiggleDuration),
                MAX_JIGGLE_CYCLES(maxJiggleCycles)
            {};

        void doAntistall();
        bool isMotorStalled() const {
            // Get current draw and actual velocity
            Current current = getCurrent();
            AngularVelocity velocity = units::abs(getActualVelocity());
            
            // Motor is stalled if it's drawing high current but moving slowly
            return (current > STALL_CURRENT_THRESHOLD && velocity < STALL_VELOCITY_THRESHOLD);
        }

        int32_t move(Number percent) override {
            targetPower = percent;
            return lemlib::Motor::move(percent);
        }

        void setStallCurrent(Current current) {
            STALL_CURRENT_THRESHOLD = current;
        }

        void setStallVelocity(AngularVelocity velocity) {
            STALL_VELOCITY_THRESHOLD = velocity;
        }

        void setJigglePower(double power) {
            JIGGLE_POWER = power;
        }

        void setJiggleDuration(uint32_t duration) {
            JIGGLE_DURATION = duration;
        }

        void setMaxJiggleCycles(uint32_t cycles) {
            MAX_JIGGLE_CYCLES = cycles;
        }

        void setStallCheckInterval(uint32_t interval) {
            STALL_CHECK_INTERVAL = interval;
        }
    private:
        double targetPower = 0.0; // Target power to apply when not jiggling
        Current STALL_CURRENT_THRESHOLD = 1.0_amp;  // Amps - adjust based on testing
        AngularVelocity STALL_VELOCITY_THRESHOLD = 5.0_rpm; // RPM - adjust based on testing  
        uint32_t JIGGLE_DURATION = 400;     // ms for each jiggle direction
        uint32_t MAX_JIGGLE_CYCLES = 3;     // number of forward-backward cycles
        double JIGGLE_POWER = 1.0;          // power for jiggling
 
        uint32_t STALL_CHECK_INTERVAL = 20; // ms
 
        uint32_t lastAntiStallTime = 0;     // Last time we checked for stall
        bool isJiggling = false;            // Flag to indicate if we are currently jiggling
        bool jiggleDirection = true;        // Current direction of jiggling (true = forward, false = backward)
        uint32_t jiggleStartTime = 0;      // Start time of the current jiggle
        uint32_t jiggleCount = 0;          // Number of jiggle cycles completed
};
}