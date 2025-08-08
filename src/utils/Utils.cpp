#include "utils/Utils.hpp"
#include <numeric>
#include <stdexcept>

namespace utils {

AngularVelocity toAngular(LinearVelocity linearVelocity, Length wheelDiameter) {
    // Angular velocity = linear velocity / radius
    // ω = v / r where r = d/2
    return from_radps(to_mps(linearVelocity) * 2.0 / to_m(wheelDiameter));
}

AngularAcceleration toAngular(LinearAcceleration linearAcceleration, Length wheelDiameter) {
    // Angular acceleration = linear acceleration / radius
    // α = a / r where r = d/2
    return from_radps2(to_mps2(linearAcceleration) * 2.0 / to_m(wheelDiameter));
}

LinearVelocity toLinear(AngularVelocity angularVelocity, Length wheelDiameter) {
    // Linear velocity = angular velocity * radius
    // v = ω * r where r = d/2
    return from_mps(to_radps(angularVelocity) * to_m(wheelDiameter) / 2.0);
}

LinearAcceleration toLinear(AngularAcceleration angularAcceleration, Length wheelDiameter) {
    // Linear acceleration = angular acceleration * radius
    // a = α * r where r = d/2
    return from_mps2(to_radps2(angularAcceleration) * to_m(wheelDiameter) / 2.0);
}

double calculateLinearRegressionSlope(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) {
        throw std::invalid_argument("Input vectors must have the same size and contain at least 2 elements");
    }

    const size_t n = x.size();
    
    // Calculate means
    double x_mean = std::accumulate(x.begin(), x.end(), 0.0) / n;
    double y_mean = std::accumulate(y.begin(), y.end(), 0.0) / n;
    
    // Calculate slope: slope = sum((x_i - x_mean)(y_i - y_mean)) / sum((x_i - x_mean)^2)
    double numerator = 0.0;
    double denominator = 0.0;
    
    for (size_t i = 0; i < n; i++) {
        double x_diff = x[i] - x_mean;
        double y_diff = y[i] - y_mean;
        numerator += x_diff * y_diff;
        denominator += x_diff * x_diff;
    }
    
    if (std::abs(denominator) < 1e-10) {
        throw std::runtime_error("Cannot calculate slope: x values are all identical");
    }
    
    return numerator / denominator;
}

} // namespace utils
