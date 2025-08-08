#pragma once

#include <cstdint>

#define PI 3.141592653589793

namespace utils
{
    /**
     * @brief The Quake inverse square root function
     *
     * @see https://en.wikipedia.org/wiki/Fast_inverse_square_root
     *
     * Tested on V5 hardware using PROS 4.1.1
     *
     * ```cpp
     * // Add volatile to prevent optimization
     * volatile float yS = 0;
     * volatile float yQ = 0;
     *
     * // First benchmark - standard sqrt
     * uint64_t starttimeSqrt = pros::micros();
     * for(int i=1; i<1000001; i++) { // Start from 1 to avoid zero
     *   yS = std::sqrt((float)i);
     * }
     * uint64_t endtimeSqrt = pros::micros();
     *
     * // Add a small delay between tests
     * pros::delay(10);
     *
     * // Second benchmark - Q_rsqrt
     * uint64_t starttimeQSqrt = pros::micros();
     * for(int i=1; i<1000001; i++) { // Start from 1 to avoid zero
     *   yQ = 1.0 / utils::Q_rsqrt((float)i);
     * }
     *
     * uint64_t endtimeQSqrt = pros::micros();
     * uint64_t sqrtTime = endtimeSqrt-starttimeSqrt;
     * uint64_t QsqrtTime = endtimeQSqrt - starttimeQSqrt;
     * printf("sqrt time: %llu us\n", sqrtTime);
     * printf("Q_rsqrt time: %llu us\n", QsqrtTime);
     * // calculate how much faster Q_rsqrt is:
     * printf("Improvement: %.2f\n",(float)sqrtTime/(float)QsqrtTime);
     * printf("yS: %f\n", yS);
     * printf("yQ: %f (1/yQ: %f)\n", yQ, 1.0f/yQ);
     * ```
     *
     * Results (prior to inlining)
     * sqrt time: 356219 us
     * Q_rsqrt time: 88718 us
     * Improvement: 4.02
     * yS: 1000.000000
     * yQ: 1001.698303
     *
     * Results (after inlining)
     * sqrt time: 357714 us
     * Q_rsqrt time: 70378 us
     * Improvement: 5.08
     * yS: 1000.000000
     * yQ: 1001.698303
     *
     * @param number
     * @return float
     */
    inline float Q_rsqrt(float number)
    {
        long i;
        float x2, y;
        const float threehalfs = 1.5F;

        x2 = number * 0.5F;
        y = number;
        i = *(long *)&y;           // evil floating point bit level hacking
        i = 0x5f3759df - (i >> 1); // what the...?
        y = *(float *)&i;
        y = y * (threehalfs - (x2 * y * y)); // 1st iteration
        // y = y * (threehalfs - (x2 * y * y));  // 2nd iteration, can be removed

        return y;
    }

    /**
     * @brief The Quake square root function with no division
     *
     * @see https://en.wikipedia.org/wiki/Fast_inverse_square_root
     *
     * Tested on V5 hardware using PROS 4.1.1
     *
     * ```cpp
     * // Add volatile to prevent optimization
     * volatile float yS = 0;
     * volatile float yQ = 0;
     *
     * // First benchmark - standard sqrt
     * uint64_t starttimeSqrt = pros::micros();
     * for(int i=1; i<1000001; i++) { // Start from 1 to avoid zero
     *   yS = std::sqrt((float)i);
     * }
     * uint64_t endtimeSqrt = pros::micros();
     *
     * // Add a small delay between tests
     * pros::delay(10);
     *
     * // Second benchmark - Q_rsqrt
     * uint64_t starttimeQSqrt = pros::micros();
     * for(int i=1; i<1000001; i++) { // Start from 1 to avoid zero
     *   yQ = fastSqrt((float)i);
     * }
     *
     * uint64_t endtimeQSqrt = pros::micros();
     * uint64_t sqrtTime = endtimeSqrt-starttimeSqrt;
     * uint64_t QsqrtTime = endtimeQSqrt - starttimeQSqrt;
     * printf("sqrt time: %llu us\n", sqrtTime);
     * printf("fastSqrt time: %llu us\n", QsqrtTime);
     * // calculate how much faster Q_rsqrt is:
     * printf("Improvement: %.2f\n",(float)sqrtTime/(float)QsqrtTime);
     * printf("yS: %f\n", yS);
     * printf("yQ: %f\n", yQ);
     * ```
     *
     * Results:
     * sqrt time: 356149 us
     * fastSqrt time: 56636 us
     * Improvement: 6.29
     * yS: 1000.000000
     * yQ: 998.304565
     *
     * @param number
     * @return float
     */
    inline float fastSqrt(float number) {
        float xhalf = 0.5f * number;
        int32_t i = *(int32_t*)&number;         // reinterpret float bits as int
        i = 0x5f3759df - (i >> 1);              // initial guess for Newton's method
        float y = *(float*)&i;                  // convert back to float
        // one iteration of Newton's method for the inverse square root
        y = y * (1.5f - (xhalf * y * y));
        // Multiply by the original number to get the square root
        return number * y;
    }

    /**
     * Fast approximation of the exponential function e^x
     *
     * @see https://stackoverflow.com/questions/47025373/fastest-implementation-of-the-natural-exponential-function-using-sse
     *
     * This is similar to the Quake Fast Inverse Square:
     *
     * 1. Does a rough approximation by manipulating the binary representation
     * 2. Extracts the mantissa
     * 3. Makes the result more accurate by applying a correction formula
     *
     * Tested on V5 hardware using PROS 4.1.1
     *
     * ```cpp
     * // Add volatile to prevent optimization
     * volatile double yS = 0;
     * volatile double yF = 0;
     *
     * // First benchmark - standard sqrt
     * uint64_t starttimeExp = pros::micros();
     * for(int i=-500000; i<50000; i++) {
     *   yS = std::exp((double)(i-500000)/100000.0);
     * }
     * uint64_t endtimeExp = pros::micros();
     *
     * // Add a small delay between tests
     * pros::delay(10);
     *
     * // Second benchmark - FastExp
     * uint64_t starttimeFast = pros::micros();
     * for(int i=-500000; i<50000; i++) {
     *   yF = utils::fastExp((double)(i-500000)/100000.0);
     * }
     *
     * uint64_t endtimeFast = pros::micros();
     * uint64_t expTime = endtimeExp-starttimeExp;
     * uint64_t fastTime = endtimeFast - starttimeFast;
     * printf("exp time: %llu us\n", expTime);
     * printf("FastExp time: %llu us\n", fastTime);
     * // calculate how much faster FastExp is:
     * printf("Improvement: %.2f\n",(float)expTime/(float)fastTime);
     * printf("yS: %f\n", yS);
     * printf("yF: %f\n", yF);
     * ```
     *
     * Results:
     * exp time: 1491607 us
     * FastExp time: 60584 us
     * Improvement: 24.62
     * yS: 0.011109
     * yF: 0.011108
     *
     * @param x The exponent value
     * @return Approximation of e^x
     */
    inline float fastExp(register float x)
    {
        union
        {
            float f;
            int32_t i;
        } reinterpreter;

        reinterpreter.i = (int32_t)(12102203.0f * x) + 127 * (1 << 23);
        int32_t m = (reinterpreter.i >> 7) & 0xFFFF; // copy mantissa
        // empirical values for small maximum relative error (8.34e-5):
        reinterpreter.i +=
            ((((((((1277 * m) >> 14) + 14825) * m) >> 14) - 79749) * m) >> 11) - 626;
        return reinterpreter.f;
    }

    /**
     * @brief Fast approximation of natural logarithm
     *
     * Uses bit manipulation of IEEE 754 format to approximate ln(x)
     *
     * @see https://github.com/romeric/fastapprox/blob/master/fastapprox/src/fastlog.h
     *
     * Tested on V5 hardware using PROS 4.1.1
     *
     * ```cpp
     * // Add volatile to prevent optimization
     * volatile float yS = 0;
     * volatile float yF = 0;
     * volatile float maxError = 0;
     *
     * // Test accuracy
     * for(int i=1; i<=10000; i++) {
     *   float x = i / 100.0f;
     *   yS = std::log(x);
     *   yF = utils::fastLog(x);
     *   if(std::abs(yS-yF) > maxError) {
     *     maxError = std::abs(yS-yF);
     *   }
     * }
     *
     * // First benchmark - standard log
     * uint64_t starttimeLog = pros::micros();
     * for(int i=1; i<=1000000; i++) {
     *   yS = std::log((float)i / 100000.0f);
     * }
     * uint64_t endtimeLog = pros::micros();
     *
     * // Add a small delay between tests
     * pros::delay(10);
     *
     * // Second benchmark - FastLog
     * uint64_t starttimeFast = pros::micros();
     * for(int i=1; i<=1000000; i++) {
     *   yF = utils::fastLog((float)i / 100000.0f);
     * }
     * uint64_t endtimeFast = pros::micros();
     *
     * uint64_t logTime = endtimeLog - starttimeLog;
     * uint64_t fastTime = endtimeFast - starttimeFast;
     * printf("log time: %llu us\n", logTime);
     * printf("FastLog time: %llu us\n", fastTime);
     * printf("Improvement: %.2f\n", (float)logTime/(float)fastTime);
     * printf("Max error: %f\n", maxError);
     * ```
     *
     * Results:
     * log time: 1649173 us
     * FastLog time: 96434 us
     * Improvement: 17.10
     * Max error: 2.038731
     *
     * @param x Input value (must be positive)
     * @return float Approximation of ln(x)
     */
    inline float fastLog(float x)
    {
        union
        {
            float f;
            uint32_t i;
        } vx = {x};

        union
        {
            uint32_t i;
            float f;
        } mx = {(vx.i & 0x007FFFFF) | 0x3F000000};

        float y = vx.i;
        y *= 1.1920928955078125e-7f; // 1/2^23

        return y - 124.22551499f - 1.498030302f * mx.f -
               1.72587999f / (0.3520887068f + mx.f);
    }

    /**
     * @brief Fast sine approximation
     *
     * @see https://stackoverflow.com/questions/18662261/fastest-implementation-of-sine-cosine-and-square-root-in-c-doesnt-need-to-b
     *
     * ```cpp
     * 	// Add volatile to prevent optimization
     * 	volatile double yS = 0;
     * 	volatile double yF = 0;
     * 	volatile double maxError180 = 0;
     * 	volatile double maxError360 = 0;
     * 	for(int i=-180; i<=180; i++) {
     * 	  yS = std::sin((double)i/180.0*PI);
     *    yF = utils::fastSin((double)i/180.0*PI);
     *    if(std::abs(yS-yF) > maxError180) {
     *      maxError180 = std::abs(yS-yF);
     *    }
     *  }
     *  for(int i=0; i<=360; i++) {
     * 	  yS = std::sin((double)i/180.0*PI);
     *    yF = utils::fastSin((double)i/180.0*PI);
     *    if(std::abs(yS-yF) > maxError360) {
     *      maxError360 = std::abs(yS-yF);
     *    }
     *  }
     *  // First benchmark - standard
     *  uint64_t starttimeSin = pros::micros();
     *  for(int j=0; j<1000; j++) {
     *    for(int i=-180; i<=180; i++) {
     *      yS = std::sin((double)i/180.0*PI);
     *    }
     *  }
     *  uint64_t endtimeSin = pros::micros();
     * 	// Add a small delay between tests
     * 	pros::delay(10);
     *
     * 	// Second benchmark - Fast
     * 	uint64_t starttimeFast = pros::micros();
     *  for(int j=0; j<1000; j++) {
     *    for(int i=-180; i<=180; i++) {
     *      yF = utils::fastSin((double)i/180.0*PI);
     *    }
     * 	}
     *  uint64_t endtimeFast = pros::micros();
     * 	uint64_t sinTime = endtimeSin-starttimeSin;
     * 	uint64_t fastTime = endtimeFast - starttimeFast;
     * 	printf("sin time: %llu us\n", sinTime);
     * 	printf("FastSin time: %llu us\n", fastTime);
     * 	// calculate how much faster FastExp is:
     * 	printf("Improvement: %.2f\n",(float)sinTime/(float)fastTime);
     * 	printf("maxError180: %f\n", maxError180);
     * 	printf("maxError360: %f\n", maxError360);
     * ```
     *
     * Results:
     * sin time: 676452 us
     * FastSin time: 127574 us
     * Improvement: 5.30
     * maxError180: 0.005199
     * maxError360: 0.005199
     *
     * @param x The angle in radians
     * @return float The sine of the angle
     */
    inline double fastSin(double x)
    {
        // Fast modulo to get x in [0,2π)
        x = fmod(x, 2 * PI);
        if (x < 0)
            x += 2 * PI; // Handle negative angles

        x /= 2 * PI;
        x -= (int)x;

        if (x <= 0.5)
        {
            double t = 2 * x * (2 * x - 1);
            return (PI * t) / ((PI - 4) * t - 1);
        }
        else
        {
            double t = 2 * (1 - x) * (1 - 2 * x);
            return -(PI * t) / ((PI - 4) * t - 1);
        }
    }

    /**
     * @brief Fast cosine approximation
     *
     * @param x Angle in radians
     * @return double Cosine of the angle
     */
    inline double fastCos(double x)
    {
        return fastSin(x + 0.5 * PI);
    }

    /**
     * @brief Fast Tangent approximation
     *
     * Results:
     * tan time: 1354158 us
     * FastTan time: 267134 us
     * Improvement: 5.07
     * maxError180: 0.043836
     * maxError360: 0.044015
     *
     * @param x angle in radians
     * @return double The tangent of the angle
     */
    inline double fastTan(double x) { return fastSin(x) / fastCos(x); }

    /**
     * @brief Fast approximation of normal distribution
     *
     * @see https://stackoverflow.com/questions/2325472/generate-random-numbers-following-a-normal-distribution-in-c-c?rq=4
     * 
     * Uses the polar form of Box-Muller transform with fast math approximations.
     * Generates normally distributed random numbers with specified mean and standard deviation.
     *
     * ```cpp
     * // Add volatile to prevent optimization
     * volatile float yS = 0;
     * volatile float yF = 0;
     *
     * std::mt19937 rng(42); // Fixed seed for reproducibility
     * std::normal_distribution<float> stdNormal(0.0f, 1.0f);
     *
     * // First benchmark - standard normal distribution
     * uint64_t starttimeStd = pros::micros();
     * for(int i=0; i<1000000; i++) {
     *   yS = stdNormal(rng);
     * }
     * uint64_t endtimeStd = pros::micros();
     *
     * // Add a small delay between tests
     * pros::delay(10);
     *
     * // Second benchmark - Fast normal distribution
     * uint64_t starttimeFast = pros::micros();
     * for(int i=0; i<1000000; i++) {
     *   yF = utils::fastNormal(0.0f, 1.0f, rng);
     * }
     * uint64_t endtimeFast = pros::micros();
     *
     * uint64_t stdTime = endtimeStd - starttimeStd;
     * uint64_t fastTime = endtimeFast - starttimeFast;
     * printf("std::normal_distribution time: %llu us\n", stdTime);
     * printf("FastNormal time: %llu us\n", fastTime);
     * printf("Improvement: %.2f\n", (float)stdTime/(float)fastTime);
     * ```
     *
     * Results:
     * std::normal_distribution time: 1120848 us
     * FastNormal time: 209727 us
     * Improvement: 5.34
     *
     * @param mean The mean of the normal distribution
     * @param stddev The standard deviation of the normal distribution
     * @param rng Random number generator (e.g., std::mt19937)
     * @return float A random value from the approximated normal distribution
     */
    template <typename RNG>
    inline float fastNormal(float mean, float stddev, RNG &rng)
    {
        static bool hasSpare = false;
        static float spare;

        if (hasSpare)
        {
            hasSpare = false;
            return mean + stddev * spare;
        }

        float u, v, s;
        do
        {
            // Generate uniform random numbers in [-1,1]
            u = 2.0f * (float)rng() / (float)rng.max() - 1.0f;
            v = 2.0f * (float)rng() / (float)rng.max() - 1.0f;
            s = u * u + v * v;
        } while (s >= 1.0f || s == 0.0f);

        // Traditional method: s = std::sqrt(-2.0f * std::log(s) / s);
        // Using our fast functions:
        float r = -2.0f * fastLog(s) / s;
        s = fastSqrt(r);

        hasSpare = true;
        spare = v * s;
        return mean + stddev * u * s;
    }

    /**
     * @brief Fast approximation of hypotenuse length (sqrt(x²+y²))
     *
     * Uses the Quake inverse square root function for improved performance.
     * This function calculates the Euclidean norm (length of a vector) quickly.
     *
     * @param x First value
     * @param y Second value
     * @return float Approximation of sqrt(x²+y²)
     */
    inline float fastHypot(float x, float y)
    {
        // Calculate the sum of squares
        float sum_sq = x * x + y * y;
        
        // Avoid division by zero
        if (sum_sq < 1e-10f) {
            return 0.0f;
        }
        
        // Use inverse of Q_rsqrt to calculate the square root
        return fastSqrt(sum_sq);
    }
}