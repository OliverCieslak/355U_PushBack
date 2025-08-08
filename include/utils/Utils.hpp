#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"
#include <vector>
#include <cmath>

namespace utils
{

    /**
     * @brief Runs sin(x)/x for a given x
     *
     * Sinc function (sin(x)/x), which is well-defined at x=0
     * This is useful for odometry calculations
     *
     * @param angle The angle in radians
     * @return sin(angle)/angle if angle != 0, 1 otherwise
     */
    inline double sinc(double x)
    {
        if (std::abs(x) < 1e-10)
            return 1.0;
        return std::sin(x) / x;
    }

    /**
     * @brief Runs sin(x)/x for a given x
     *
     * Sinc function for Angle type
     *
     * @param angle The angle
     * @return sin(angle)/angle if angle != 0, 1 otherwise
     */
    inline double sinc(Angle angle)
    {
        double rad = to_stRad(angle);
        return sinc(rad);
    }

    /**
     * @brief Convert linear velocity to angular velocity
     *
     * @param linearVelocity Linear velocity
     * @param wheelDiameter Diameter of the wheel
     * @return AngularVelocity Angular velocity
     */
    AngularVelocity toAngular(LinearVelocity linearVelocity, Length wheelDiameter);

    /**
     * @brief Convert linear acceleration to angular acceleration
     *
     * @param linearAcceleration Linear acceleration
     * @param wheelDiameter Diameter of the wheel
     * @return AngularAcceleration Angular acceleration
     */
    AngularAcceleration toAngular(LinearAcceleration linearAcceleration, Length wheelDiameter);

    /**
     * @brief Convert angular velocity to linear velocity
     *
     * @param angularVelocity Angular velocity
     * @param wheelDiameter Diameter of the wheel
     * @return LinearVelocity Linear velocity
     */
    LinearVelocity toLinear(AngularVelocity angularVelocity, Length wheelDiameter);

    /**
     * @brief Convert angular acceleration to linear acceleration
     *
     * @param angularAcceleration Angular acceleration
     * @param wheelDiameter Diameter of the wheel
     * @return LinearAcceleration Linear acceleration
     */
    LinearAcceleration toLinear(AngularAcceleration angularAcceleration, Length wheelDiameter);

    /**
     * @brief Calculate the slope of a linear regression line
     *
     * @param x The x-values
     * @param y The y-values
     * @return double The slope of the regression line
     */
    double calculateLinearRegressionSlope(const std::vector<double> &x, const std::vector<double> &y);

} // namespace utils
