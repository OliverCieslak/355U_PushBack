#include "motion/SCurveProfile.hpp"
#include <cmath>
#include <algorithm>

namespace motion {

SCurveProfile::SCurveProfile()
    : SCurveProfile(0.0, 1.0, 1.0, 1.0) {}

SCurveProfile::SCurveProfile(double distance, double maxVelocity,
                             double maxAcceleration, double maxJerk) {
    computeProfile(std::abs(distance), std::abs(maxVelocity),
                   std::abs(maxAcceleration), std::abs(maxJerk));
}

void SCurveProfile::computeProfile(double distance, double maxVelocity,
                                   double maxAcceleration, double maxJerk) {
    m_distance = distance;

    // Guard against degenerate inputs
    if (distance <= 1e-6 || maxVelocity <= 1e-6 ||
        maxAcceleration <= 1e-6 || maxJerk <= 1e-6) {
        m_j_max = maxJerk > 0 ? maxJerk : 1.0;
        m_a_max = 0; m_v_max = 0;
        m_T_j = 0; m_T_a = 0; m_T_v = 0;
        m_totalTime = 0;
        m_t1 = m_t2 = m_t3 = m_t4 = m_t5 = m_t6 = m_t7 = 0;
        m_v1 = m_v2 = m_v3 = m_v5 = m_v6 = 0;
        m_p1 = m_p2 = m_p3 = m_p4 = m_p5 = m_p6 = 0;
        return;
    }

    m_j_max = maxJerk;

    // ---------------------------------------------------------------
    // Helper: compute the distance consumed by one accel half (phases
    // 1-3) for given peak velocity V, max accel A, and jerk J.
    // Returns the distance (position at end of phase 3).
    // ---------------------------------------------------------------
    auto computeAccelDistance = [](double V, double A, double J,
                                  double& outTj, double& outTa,
                                  double& outAchievedA) -> double {
        double Tj = A / J;
        double Ta = V / A - Tj;

        if (Ta < 0) {
            // Jerk-limited: can't reach max accel before needing to de-jerk
            Tj = std::sqrt(V / J);
            Ta = 0;
            A = J * Tj; // reduced peak acceleration
        }

        outTj = Tj;
        outTa = Ta;
        outAchievedA = A;

        double v1 = 0.5 * J * Tj * Tj;
        double p1 = (1.0 / 6.0) * J * Tj * Tj * Tj;

        double v2 = v1 + A * Ta;
        double p2 = p1 + v1 * Ta + 0.5 * A * Ta * Ta;

        double p3 = p2 + v2 * Tj + 0.5 * A * Tj * Tj
                     - (1.0 / 6.0) * J * Tj * Tj * Tj;
        return p3;
    };

    // ---------------------------------------------------------------
    // Step 1: try with the requested maxVelocity
    // ---------------------------------------------------------------
    double Tj, Ta, achievedA;
    double d_accel = computeAccelDistance(maxVelocity, maxAcceleration,
                                          maxJerk, Tj, Ta, achievedA);

    // ---------------------------------------------------------------
    // Step 2: if distance is too short for accel+decel, reduce V_max
    //         via binary search.
    // ---------------------------------------------------------------
    if (2.0 * d_accel > distance) {
        double lo = 0.0, hi = maxVelocity;
        for (int iter = 0; iter < 60; ++iter) {
            double mid = (lo + hi) * 0.5;
            double dTj, dTa, dA;
            double d = computeAccelDistance(mid, maxAcceleration, maxJerk,
                                           dTj, dTa, dA);
            if (2.0 * d > distance) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        maxVelocity = lo;
        d_accel = computeAccelDistance(maxVelocity, maxAcceleration, maxJerk,
                                      Tj, Ta, achievedA);
    }

    m_a_max = achievedA;
    m_v_max = maxVelocity;
    m_T_j = Tj;
    m_T_a = Ta;

    // ---------------------------------------------------------------
    // Step 3: cruise phase
    // ---------------------------------------------------------------
    double cruiseDist = distance - 2.0 * d_accel;
    m_T_v = (m_v_max > 1e-6) ? (cruiseDist / m_v_max) : 0.0;
    if (m_T_v < 0) m_T_v = 0;

    // ---------------------------------------------------------------
    // Step 4: phase boundary times
    // ---------------------------------------------------------------
    m_t1 = m_T_j;
    m_t2 = m_T_j + m_T_a;
    m_t3 = 2.0 * m_T_j + m_T_a;
    m_t4 = 2.0 * m_T_j + m_T_a + m_T_v;
    m_t5 = 3.0 * m_T_j + m_T_a + m_T_v;
    m_t6 = 3.0 * m_T_j + 2.0 * m_T_a + m_T_v;
    m_t7 = 4.0 * m_T_j + 2.0 * m_T_a + m_T_v;
    m_totalTime = m_t7;

    // ---------------------------------------------------------------
    // Step 5: precompute state values at phase boundaries
    // ---------------------------------------------------------------
    double J = m_j_max;
    double A = m_a_max;

    // End of phase 1
    m_v1 = 0.5 * J * m_T_j * m_T_j;
    m_p1 = (1.0 / 6.0) * J * m_T_j * m_T_j * m_T_j;

    // End of phase 2
    m_v2 = m_v1 + A * m_T_a;
    m_p2 = m_p1 + m_v1 * m_T_a + 0.5 * A * m_T_a * m_T_a;

    // End of phase 3 (velocity should equal m_v_max)
    m_v3 = m_v2 + A * m_T_j - 0.5 * J * m_T_j * m_T_j;
    m_p3 = m_p2 + m_v2 * m_T_j
           + 0.5 * A * m_T_j * m_T_j
           - (1.0 / 6.0) * J * m_T_j * m_T_j * m_T_j;

    // End of phase 4 (cruise)
    m_p4 = m_p3 + m_v_max * m_T_v;

    // End of phase 5
    m_v5 = m_v_max - 0.5 * J * m_T_j * m_T_j;
    m_p5 = m_p4 + m_v_max * m_T_j
           - (1.0 / 6.0) * J * m_T_j * m_T_j * m_T_j;

    // End of phase 6
    m_v6 = m_v5 - A * m_T_a;
    m_p6 = m_p5 + m_v5 * m_T_a - 0.5 * A * m_T_a * m_T_a;
}

SCurveState SCurveProfile::sample(double time) const {
    // Degenerate profile
    if (m_totalTime <= 1e-9) {
        return {m_distance, 0.0, 0.0};
    }

    // Clamp to profile bounds
    if (time <= 0.0) {
        return {0.0, 0.0, 0.0};
    }
    if (time >= m_totalTime) {
        return {m_distance, 0.0, 0.0};
    }

    double J = m_j_max;
    double A = m_a_max;

    // Phase 1: increasing acceleration (jerk = +J)
    if (time <= m_t1) {
        double t = time;
        double a = J * t;
        double v = 0.5 * J * t * t;
        double p = (1.0 / 6.0) * J * t * t * t;
        return {p, v, a};
    }

    // Phase 2: constant acceleration (jerk = 0, accel = +A)
    if (time <= m_t2) {
        double t = time - m_t1;
        double a = A;
        double v = m_v1 + A * t;
        double p = m_p1 + m_v1 * t + 0.5 * A * t * t;
        return {p, v, a};
    }

    // Phase 3: decreasing acceleration (jerk = -J)
    if (time <= m_t3) {
        double t = time - m_t2;
        double a = A - J * t;
        double v = m_v2 + A * t - 0.5 * J * t * t;
        double p = m_p2 + m_v2 * t + 0.5 * A * t * t
                   - (1.0 / 6.0) * J * t * t * t;
        return {p, v, a};
    }

    // Phase 4: constant velocity cruise
    if (time <= m_t4) {
        double t = time - m_t3;
        return {m_p3 + m_v_max * t, m_v_max, 0.0};
    }

    // Phase 5: beginning deceleration (jerk = -J)
    if (time <= m_t5) {
        double t = time - m_t4;
        double a = -J * t;
        double v = m_v_max - 0.5 * J * t * t;
        double p = m_p4 + m_v_max * t - (1.0 / 6.0) * J * t * t * t;
        return {p, v, a};
    }

    // Phase 6: constant deceleration (jerk = 0, accel = -A)
    if (time <= m_t6) {
        double t = time - m_t5;
        double a = -A;
        double v = m_v5 - A * t;
        double p = m_p5 + m_v5 * t - 0.5 * A * t * t;
        return {p, v, a};
    }

    // Phase 7: ending deceleration (jerk = +J)
    {
        double t = time - m_t6;
        double a = -A + J * t;
        double v = m_v6 - A * t + 0.5 * J * t * t;
        double p = m_p6 + m_v6 * t - 0.5 * A * t * t
                   + (1.0 / 6.0) * J * t * t * t;
        return {p, v, a};
    }
}

} // namespace motion
