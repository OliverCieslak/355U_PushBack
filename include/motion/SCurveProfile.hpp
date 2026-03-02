#pragma once

#include <cmath>
#include <algorithm>

namespace motion {

/**
 * @brief S-curve motion profile state at a point in time
 */
struct SCurveState {
    double position;     ///< position (inches or degrees)
    double velocity;     ///< velocity (inches/s or degrees/s)
    double acceleration; ///< acceleration (inches/s² or degrees/s²)
};

/**
 * @brief Generates a 7-segment S-curve motion profile
 *
 * An S-curve profile limits jerk (rate of change of acceleration) to produce
 * smoother, more consistent motion than a trapezoidal profile. The profile
 * has up to 7 phases:
 *
 * 1. Increasing acceleration  (jerk = +J_max)
 * 2. Constant acceleration    (jerk = 0, accel = +A_max)
 * 3. Decreasing acceleration  (jerk = -J_max, accel → 0)
 * 4. Constant velocity cruise (jerk = 0, accel = 0)
 * 5. Increasing deceleration  (jerk = -J_max, accel → -A_max)
 * 6. Constant deceleration    (jerk = 0, accel = -A_max)
 * 7. Decreasing deceleration  (jerk = +J_max, accel → 0)
 *
 * Short distances automatically compress or eliminate phases. The profile
 * is symmetric: acceleration and deceleration ramps are identical.
 *
 * All internal computation uses raw doubles (inches, seconds). Callers
 * using the VEX unit system should convert before construction and after
 * sampling.
 */
class SCurveProfile {
public:
    /**
     * @brief Construct an S-curve profile
     *
     * @param distance         Total distance to travel (positive)
     * @param maxVelocity      Maximum velocity (positive)
     * @param maxAcceleration  Maximum acceleration (positive)
     * @param maxJerk          Maximum jerk (positive)
     */
    SCurveProfile(double distance, double maxVelocity,
                  double maxAcceleration, double maxJerk);

    /**
     * @brief Default constructor — zero-length profile
     */
    SCurveProfile();

    /**
     * @brief Sample the profile at a given time
     *
     * @param time  Time in seconds from profile start
     * @return SCurveState  Position, velocity, and acceleration
     */
    SCurveState sample(double time) const;

    /** @brief Total duration of the profile (seconds) */
    double totalTime() const { return m_totalTime; }

    /** @brief True once the profile is complete */
    bool isFinished(double time) const { return time >= m_totalTime; }

    /** @brief Actual max velocity achieved (may be < requested for short moves) */
    double achievedMaxVelocity() const { return m_v_max; }

    /** @brief Total distance of the profile */
    double distance() const { return m_distance; }

private:
    void computeProfile(double distance, double maxVelocity,
                        double maxAcceleration, double maxJerk);

    // Profile parameters
    double m_j_max = 0;    ///< Maximum jerk
    double m_a_max = 0;    ///< Achieved max acceleration
    double m_v_max = 0;    ///< Achieved max velocity
    double m_distance = 0; ///< Total distance

    // Phase durations
    double m_T_j = 0;      ///< Jerk phase duration
    double m_T_a = 0;      ///< Constant acceleration phase duration
    double m_T_v = 0;      ///< Cruise phase duration
    double m_totalTime = 0; ///< Total profile duration

    // Phase boundary times (cumulative)
    double m_t1 = 0, m_t2 = 0, m_t3 = 0, m_t4 = 0;
    double m_t5 = 0, m_t6 = 0, m_t7 = 0;

    // State values at phase boundaries (precomputed for fast sampling)
    double m_v1 = 0, m_v2 = 0, m_v3 = 0;
    double m_v5 = 0, m_v6 = 0;
    double m_p1 = 0, m_p2 = 0, m_p3 = 0;
    double m_p4 = 0, m_p5 = 0, m_p6 = 0;
};

} // namespace motion
