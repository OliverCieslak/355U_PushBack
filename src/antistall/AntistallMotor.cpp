#include "antistall/AntistallMotor.hpp"

namespace antistall {

void AntistallMotor::doAntistall() {
    uint32_t now = pros::millis();

    switch (phase) {

        case Phase::REVERSING: {
            // Scale reverse duration up on repeated retries (1x, 1.5x, 2x ...)
            uint32_t actualReverseDuration = baseReverseDuration + 
                (baseReverseDuration / 2) * retryCount;
            if (now - phaseStartTime >= actualReverseDuration) {
                // Done reversing — coast briefly to let things settle
                phase = Phase::PAUSED;
                phaseStartTime = now;
                lemlib::Motor::move(0);
            }
            // else: keep reversing (power was already set when entering this phase)
            return;
        }

        case Phase::PAUSED: {
            if (now - phaseStartTime >= pauseDuration) {
                // Re-engage at target power
                phase = Phase::COOLDOWN;
                phaseStartTime = now;
                lemlib::Motor::move(targetPower);
            }
            return;
        }

        case Phase::COOLDOWN: {
            if (now - phaseStartTime >= cooldownDuration) {
                // Check if still stalled after re-engaging
                if (isMotorStalled() && std::fabs(targetPower) > 0.1) {
                    retryCount++;
                    if (retryCount >= maxRetries) {
                        // Give up — return to normal and let it be
                        phase = Phase::IDLE;
                        retryCount = 0;
                        stallDebounceCount = 0;
                    } else {
                        // Try again with longer reverse
                        phase = Phase::REVERSING;
                        phaseStartTime = now;
                        double sign = (targetPower >= 0.0) ? 1.0 : -1.0;
                        lemlib::Motor::move(-sign * reversePower);
                    }
                } else {
                    // Unjammed successfully
                    phase = Phase::IDLE;
                    retryCount = 0;
                    stallDebounceCount = 0;
                }
            }
            return;
        }

        case Phase::IDLE:
        default:
            break;
    }

    // === IDLE: monitor for stalls ===

    if (now - lastStallCheckTime < stallCheckInterval) {
        return;
    }
    lastStallCheckTime = now;

    if (std::fabs(targetPower) <= 0.1) {
        stallDebounceCount = 0;
        return;
    }

    if (isMotorStalled()) {
        stallDebounceCount++;
        if (stallDebounceCount >= stallDebounceThreshold) {
            // Start recovery: reverse immediately
            phase = Phase::REVERSING;
            phaseStartTime = now;
            retryCount = 0;
            stallDebounceCount = 0;

            double sign = (targetPower >= 0.0) ? 1.0 : -1.0;
            lemlib::Motor::move(-sign * reversePower);
        }
    } else {
        stallDebounceCount = 0;
    }
}

}