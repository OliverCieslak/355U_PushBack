#pragma once

#include "hardware/Motor/Motor.hpp"
#include "hardware/Port.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <cmath>

namespace antistall {


class AntistallMotor : public lemlib::Motor {

    public:
        /**
         * @brief Antistall recovery phases:
         *   IDLE      – normal operation, monitoring for stalls
         *   REVERSING – actively reversing to unjam
         *   PAUSED    – brief coast/brake after reverse before re-engaging
         *   COOLDOWN  – back to target power, ignoring stalls temporarily
         */
        enum class Phase { IDLE, REVERSING, PAUSED, COOLDOWN };

        AntistallMotor(lemlib::ReversibleSmartPort port, AngularVelocity outputVelocity)
            : lemlib::Motor(port, outputVelocity) {};

        AntistallMotor(lemlib::ReversibleSmartPort port, 
                AngularVelocity outputVelocity,
                Current stallCurrentThreshold,
                AngularVelocity stallVelocityThreshold,
                double reversePower = 0.7,
                uint32_t reverseDuration = 200,
                uint32_t maxRetries = 3,
                uint32_t stallDebounceThreshold = 3,
                uint32_t stallCheckInterval = 20,
                uint32_t pauseDuration = 50,
                uint32_t cooldownDuration = 500)
            : lemlib::Motor(port, outputVelocity),
                stallCurrentThreshold(stallCurrentThreshold),
                stallVelocityThreshold(stallVelocityThreshold),
                reversePower(reversePower),
                baseReverseDuration(reverseDuration),
                maxRetries(maxRetries),
                stallDebounceThreshold(stallDebounceThreshold),
                stallCheckInterval(stallCheckInterval),
                pauseDuration(pauseDuration),
                cooldownDuration(cooldownDuration)
            {};

        void doAntistall();

        bool isMotorStalled() const {
            Current current = getCurrent();
            AngularVelocity velocity = units::abs(getActualVelocity());
            return (current > stallCurrentThreshold && velocity < stallVelocityThreshold);
        }

        int32_t move(Number percent) override {
            targetPower = percent;
            // During recovery, don't send new power — doAntistall() owns the motor
            if (phase != Phase::IDLE) {
                // If commanded to stop, cancel recovery immediately
                if (std::fabs(static_cast<double>(percent)) <= 0.1) {
                    phase = Phase::IDLE;
                    retryCount = 0;
                    stallDebounceCount = 0;
                    lemlib::Motor::move(0);
                }
                return 0;
            }
            return lemlib::Motor::move(percent);
        }

        Phase getPhase() const { return phase; }

        void setStallCurrent(Current current) { stallCurrentThreshold = current; }
        void setStallVelocity(AngularVelocity velocity) { stallVelocityThreshold = velocity; }
        void setReversePower(double power) { reversePower = power; }
        void setReverseDuration(uint32_t duration) { baseReverseDuration = duration; }
        void setMaxRetries(uint32_t retries) { maxRetries = retries; }
        void setStallCheckInterval(uint32_t interval) { stallCheckInterval = interval; }
        void setStallDebounceThreshold(uint32_t threshold) { stallDebounceThreshold = threshold; }
        void setPauseDuration(uint32_t duration) { pauseDuration = duration; }
        void setCooldownDuration(uint32_t duration) { cooldownDuration = duration; }

    private:
        double targetPower = 0.0;

        // Stall detection
        Current stallCurrentThreshold = 2.5_amp;
        AngularVelocity stallVelocityThreshold = 10.0_rpm;
        uint32_t stallCheckInterval = 20;       // ms between stall checks
        uint32_t stallDebounceThreshold = 3;    // consecutive stall readings to trigger
        uint32_t stallDebounceCount = 0;
        uint32_t lastStallCheckTime = 0;

        // Recovery parameters
        double reversePower = 0.7;              // power magnitude for reverse pulse
        uint32_t baseReverseDuration = 200;     // ms to reverse (scales up on retries)
        uint32_t pauseDuration = 50;            // ms to coast after reverse
        uint32_t cooldownDuration = 500;        // ms after re-engaging before checking stalls
        uint32_t maxRetries = 3;                // max reverse attempts before giving up

        // State machine
        Phase phase = Phase::IDLE;
        uint32_t phaseStartTime = 0;
        uint32_t retryCount = 0;
};
}