#include "localization/ParticleFilter.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include <iostream>
#include <iomanip>
#include <unordered_map>

namespace localization
{

    ParticleFilter::ParticleFilter(
        odometry::SkidSteerOdometry &odometry,
        const units::Pose &initialPose,
        size_t numParticles,
        Length motionNoise,
        Angle angleNoise,
        Time updateInterval) : m_odometry(odometry),
                               m_estimatedPose(initialPose),
                               m_lastOdometryPose(odometry.getPose()),
                               m_numParticles(numParticles),
                               m_motionNoise(motionNoise),
                               m_angleNoise(angleNoise),
                               m_updateInterval(updateInterval),
                               m_rng(std::random_device{}()),
                               m_normalDist(0.0, 1.0),
                               m_prevVelocityPose(initialPose),
                               m_lastVelocityTime(pros::micros())
    {
        // Pre-allocate all vectors that will be used during runtime
        m_particles.reserve(numParticles);
        m_tempParticles.reserve(numParticles);     // New vector for resampling to avoid allocations
        m_cumulativeWeights.reserve(numParticles); // Pre-allocate weights vector
        m_sensorMeasurements.reserve(10);          // Pre-allocate for typical number of sensors

        initializeParticles(initialPose);

        // Initialize the sensor log timestamp
        m_lastSensorLogTime = 0;
    }

    ParticleFilter::~ParticleFilter()
    {
        stop();
    }

    void ParticleFilter::initializeParticles(const units::Pose &initialPose)
    {
        m_particles.clear();
        // No need to reserve again, already done in constructor

        // Determine initial noise based on our certainty
        Length posNoise = m_motionNoise;
        Angle angNoise = 2.0_stDeg;

        for (size_t i = 0; i < m_numParticles; i++)
        {
            // Add randomized particles around the initial pose
            Length x = initialPose.x + posNoise * m_normalDist(m_rng);
            Length y = initialPose.y + posNoise * m_normalDist(m_rng);
            Angle theta = from_stDeg(to_stDeg(initialPose.orientation) + to_stDeg(angNoise) * m_normalDist(m_rng));

            // Create a pose and ensure it's within field boundaries
            units::Pose particlePose(x, y, theta);
            particlePose = utils::constrainToField(particlePose);

            m_particles.emplace_back(particlePose, 1.0 / m_numParticles);
        }

        m_estimatedPose = utils::constrainToField(initialPose);
        m_lastOdometryPose = m_odometry.getPose();
    }

    void ParticleFilter::start()
    {
        if (m_isRunning)
            return;

        m_isRunning = true;
        m_updateTask = new pros::Task([this]
                                      { this->updateTaskTrampoline(this); });
    }

    void ParticleFilter::stop()
    {
        if (!m_isRunning)
            return;

        m_isRunning = false;
        if (m_updateTask)
        {
            m_updateTask->remove();
            delete m_updateTask;
            m_updateTask = nullptr;
        }
    }

    units::Pose ParticleFilter::update()
    {
        // Start timing the whole update
        uint32_t updateStartTime = pros::micros();

        // Get current odometry reading
        units::Pose currentOdomPose = m_odometry.getPose();
        // Record the time immediately after getting sensor readings for velocity calculations
        uint32_t currentSensorTime = pros::micros();

        // Calculate odometry change since last update
        units::Pose odometryDelta = units::Pose(
            currentOdomPose.x - m_lastOdometryPose.x,
            currentOdomPose.y - m_lastOdometryPose.y,
            from_stDeg(to_stDeg(currentOdomPose.orientation) - to_stDeg(m_lastOdometryPose.orientation)));

        // Check if it's time to log performance metrics
        if (m_performanceLoggingEnabled)
        {
            m_updatesSinceLastLog++;
            uint32_t currentTime = pros::millis();
            if (currentTime - m_lastPerformanceLogTime >= m_performanceLogInterval)
            {
                std::cout << "Odometry Delta: (" << to_in(odometryDelta.x) << " in, " << to_in(odometryDelta.y) << " in, " << to_stDeg(odometryDelta.orientation) << " deg)\n";
            }
        }
        

        m_lastOdometryPose = currentOdomPose;

        // Time motion update
        uint32_t motionStartTime = pros::micros();
        motionUpdate(odometryDelta);
        m_motionUpdateTimeUs = pros::micros() - motionStartTime;

        // Apply sensor update if sensors are available
        if (!m_sensors.empty())
        {
            // Time sensor update
            uint32_t sensorStartTime = pros::micros();
            sensorUpdate();
            m_sensorUpdateTimeUs = pros::micros() - sensorStartTime;

            // Time resampling
            uint32_t resampleStartTime = pros::micros();
            resample();
            m_resampleTimeUs = pros::micros() - resampleStartTime;
        }
        else
        {
            m_sensorUpdateTimeUs = 0;
            m_resampleTimeUs = 0;
        }

        // Calculate estimated pose
        m_estimatedPose = calculateEstimatedPose();
        
        // Update velocity calculations
        uint32_t deltaTimeUs = currentSensorTime - m_lastVelocityTime;
        
        // Only calculate velocity if enough time has passed (avoid division by very small numbers)
        if (deltaTimeUs > 5000) { // 5ms minimum
            // Calculate linear velocity
            Length dx = m_estimatedPose.x - m_prevVelocityPose.x;
            Length dy = m_estimatedPose.y - m_prevVelocityPose.y;
            Length distance = units::sqrt(dx * dx + dy * dy);
            m_linearVelocity = distance / from_usec(deltaTimeUs);
            
            // Calculate angular velocity (handle angle wrapping)
            Angle dtheta = from_stDeg(to_stDeg(m_estimatedPose.orientation) - to_stDeg(m_prevVelocityPose.orientation));
            // Normalize the angle difference to -180 to 180 degrees
            dtheta = units::constrainAngle180(dtheta);
            m_angularVelocity = dtheta / from_usec(deltaTimeUs);
            
            // Store current values for next calculation
            m_prevVelocityPose = m_estimatedPose;
            m_lastVelocityTime = currentSensorTime;  // Use sensor reading time instead of calculation time
        }

        // Calculate total update time
        m_totalUpdateTimeUs = pros::micros() - updateStartTime;

        // Update moving averages with exponential smoothing
        const double ALPHA = 0.2; // Smoothing factor
        m_avgMotionUpdateTimeUs = (1 - ALPHA) * m_avgMotionUpdateTimeUs + ALPHA * m_motionUpdateTimeUs;
        m_avgSensorUpdateTimeUs = (1 - ALPHA) * m_avgSensorUpdateTimeUs + ALPHA * m_sensorUpdateTimeUs;
        m_avgResampleTimeUs = (1 - ALPHA) * m_avgResampleTimeUs + ALPHA * m_resampleTimeUs;
        m_avgTotalUpdateTimeUs = (1 - ALPHA) * m_avgTotalUpdateTimeUs + ALPHA * m_totalUpdateTimeUs;

        // Check if it's time to log performance metrics
        if (m_performanceLoggingEnabled)
        {
            m_updatesSinceLastLog++;
            uint32_t currentTime = pros::millis();
            if (currentTime - m_lastPerformanceLogTime >= m_performanceLogInterval)
            {
                logPerformanceMetrics();
                m_lastPerformanceLogTime = currentTime;
                m_updatesSinceLastLog = 0;
            }
        }

        return m_estimatedPose;
    }

    void ParticleFilter::motionUpdate(const units::Pose &odometryDelta)
    {
        // Calculate position noise based on the magnitude of the movement
        Length distanceMoved = units::sqrt(odometryDelta.x * odometryDelta.x + odometryDelta.y * odometryDelta.y);

        // Increase position noise a bit to allow more flexibility for sensor corrections
        Length positionNoise = 0_in;
        if (distanceMoved > 0.01_in)
        {
            positionNoise = 0.002_in + m_motionNoise * (distanceMoved / 1_in) * 1.5;
        }

        // Reduce angle noise significantly since IMU is reliable for short periods
        Angle reducedAngleNoise = m_angleNoise * 0.05; // Reduced from 0.1 to 0.05 to minimize angle drift

        // Update each particle's position based on odometry and add noise
        for (auto &particle : m_particles)
        {
            // Rotate the delta to the particle's frame - pre-calculate trig values
            Angle theta = particle.pose.orientation;
            double cos_theta = utils::fastCos(to_stRad(theta));
            double sin_theta = utils::fastSin(to_stRad(theta));

            // This is the correct transformation from global to particle frame
            Length dx_local = odometryDelta.x * cos_theta + odometryDelta.y * sin_theta;
            Length dy_local = -odometryDelta.x * sin_theta + odometryDelta.y * cos_theta;

            // Add noise proportional to movement using fastNormal instead of m_normalDist
            dx_local += positionNoise * utils::fastNormal(0.0, 1.0, m_rng);
            dy_local += positionNoise * utils::fastNormal(0.0, 1.0, m_rng);

            // Convert back to global frame for updating particle position
            Length dx_global = dx_local * cos_theta - dy_local * sin_theta;
            Length dy_global = dx_local * sin_theta + dy_local * cos_theta;

            // Add orientation change with MINIMAL noise since IMU is reliable
            double rotationMagnitude = std::abs(to_stDeg(odometryDelta.orientation));
            // Near-zero noise when stationary, slightly more when rotating
            double noiseFactor = rotationMagnitude < 0.1 ? 0.001 : (rotationMagnitude < 1.0 ? 0.005 : 0.01);

            Angle dtheta = odometryDelta.orientation +
                           reducedAngleNoise * rotationMagnitude * noiseFactor * utils::fastNormal(0.0, 1.0, m_rng);

            // Update the particle pose
            particle.pose.x += dx_global;
            particle.pose.y += dy_global;
            particle.pose.orientation += dtheta;

            // Constrain the particle to remain within field boundaries
            particle.pose = utils::constrainToField(particle.pose);
        }
    }

    void ParticleFilter::sensorUpdate()
    {
        // DEBUG: Print current estimated pose for context
        units::Pose currentPose = calculateEstimatedPose();
        // Removed sensor debug logging for cleaner output
        
        // Evaluate each sensor to find the best ones to use
        m_sensorScores.clear(); // Reuse the vector instead of creating a new one
        m_sensorScores.reserve(m_sensors.size());
        
        // Clear and prepare the sensor measurements cache
        m_sensorMeasurements.clear();
        m_sensorMeasurements.reserve(m_sensors.size());

        for (const auto &sensor : m_sensors)
        {
            // Get the actual sensor measurement
            Length measurement = sensor.getMeasurement();
            
            // Cache the measurement for logging
            m_sensorMeasurements.push_back(measurement);

            // Compare actual vs expected distance for validation
            Length expectedDistance = sensor.getExpectedMeasurement(currentPose);

            // Get sensor confidence first - V5 distance sensor returns 0-63
            int confidence = sensor.getConfidence();
            
            // STRICT sensor validation - reject sensors with poor readings
            bool sensorValid = true;
            std::string rejectReason = "";
            
            // 1. Check confidence level - slightly relaxed for single-sensor scenarios
            if (confidence < 40) {  // Raised from 12 to 40 for higher data quality
                sensorValid = false;
                rejectReason = "LOW CONFIDENCE (" + std::to_string(confidence) + "/63)";
            }
            
            // 2. Check if reading is within physical sensor range
            else if (!utils::isValidDistanceSensorReading(measurement)) {
                sensorValid = false;
                rejectReason = "OUT OF SENSOR RANGE";
            }
            
            // 3. Check if reading makes sense given field size (max diagonal ~102")
            else if (to_in(measurement) > 110.0) {  // Stricter than sensor max, based on field geometry
                sensorValid = false;
                rejectReason = "EXCEEDS FIELD SIZE";
            }
            
            // 4. Check if reading is suspiciously close to sensor error value
            else if (std::abs(to_in(measurement) - 393.66) < 1.0) {  // Known error value
                sensorValid = false;
                rejectReason = "KNOWN ERROR VALUE";
            }
            
            // 5. Check if expected distance is reasonable (sensor pointing toward field boundary)
            else if (to_in(expectedDistance) > 110.0) {
                sensorValid = false;
                rejectReason = "EXPECTED DISTANCE OUT OF FIELD";
            }
            
            // 6. Check if actual vs expected distance difference indicates obstruction (game object, etc.)
            else if (std::abs(to_in(measurement - expectedDistance)) > 50.0) {
                sensorValid = false;
                rejectReason = "LARGE DISTANCE DISCREPANCY";
            }
            
            if (!sensorValid) {
                // Sensor rejected - skip without detailed logging
                continue;
            }

            // Current sensor status: Front (working), Right/Back/Left (hardware failures - 9999mm readings)
            // System automatically rejects failed sensors via validation checks above

            // Use confidence-based scoring with additional validation
            double score = (confidence + 1) / 64.0; // Normalize to 0-1 range
            
            // Boost score for readings that closely match expected values
            double errorMagnitude = std::abs(to_in(measurement - expectedDistance));
            if (errorMagnitude < 2.0) {  // Very close match
                score *= 1.2;
            } else if (errorMagnitude > 10.0) {  // Large error
                score *= 0.5;
            }

            m_sensorScores.emplace_back(sensor.id, score, measurement);
        }

        // Sort sensors by their scores (best first)
        std::sort(m_sensorScores.begin(), m_sensorScores.end());

        // Take the top 2 sensors (if available)
        int numSensorsToUse = std::min(size_t(2), m_sensorScores.size());

        // If we have no valid sensors, use default weights
        if (numSensorsToUse == 0 || m_sensorScores[0].score <= 0.0)
        {
            // Assign uniform weights when no reliable sensors are available
            const double uniformWeight = 1.0 / m_particles.size();
            for (auto &particle : m_particles)
            {
                particle.weight = uniformWeight;
            }
            return;
        }

        // Keep only the top sensors we want to use
        if (numSensorsToUse < m_sensorScores.size())
        {
            m_sensorScores.resize(numSensorsToUse);
        }

        // Update weights using only the best sensors
        updateParticleWeights(m_sensorScores);
    }

    void ParticleFilter::updateParticleWeights(const std::vector<utils::SensorScore> &selectedSensors)
    {
        // Get current odometry heading for comparison
        Angle odomHeading = from_stDeg(to_stDeg(m_odometry.getPose().orientation));

        // For each particle, calculate how well it matches the sensor readings
        double maxLogWeight = -std::numeric_limits<double>::infinity();
        int processedParticles = 0;

        // Track the best particle for logging
        const Particle *bestParticle = nullptr;

        // Define threshold for early termination (in log space)
        const double MIN_LOG_WEIGHT_THRESHOLD = -20.0; // Equivalent to ~2.06e-9 in linear space

        for (auto &particle : m_particles)
        {
            double logPositionWeight = 0.0; // Using log weights to avoid underflow

            // Add heading component - compare particle heading with odometry heading
            Angle headingError = from_stDeg(to_stDeg(particle.pose.orientation) - to_stDeg(odomHeading));
            // Constrain to smallest angle difference (-180 to 180 degrees)
            headingError = units::constrainAngle180(headingError);
            // Calculate heading weight using Gaussian model
            double headingErrorDeg = to_stDeg(headingError);
            // Scale factor determines importance of heading (lower = more important)
            const double HEADING_SCALE_FACTOR = 10.0;
            double headingLogWeight = -1.0 * headingErrorDeg * headingErrorDeg * HEADING_SCALE_FACTOR;

            // Add heading component to total weight
            logPositionWeight += headingLogWeight;

            int particleValidSensorCount = 0;
            for (const auto &sensorScore : selectedSensors)
            {
                int sensorID = sensorScore.sensorID;
                Length actualMeasurement = sensorScore.measurement;

                // Find the corresponding sensor object
                auto sensorIter = std::find_if(
                    m_sensors.begin(),
                    m_sensors.end(),
                    [sensorID](const DistanceSensor &s)
                    { return s.id == sensorID; });
                if (sensorIter == m_sensors.end()){
                    std::cout << "WARNING!!!!! Sensor not found" << std::endl;
                    continue;
                }

                // Calculate expected measurement for this particle
                Length expectedMeasurement = sensorIter->getExpectedMeasurement(particle.pose);

                // Calculate sensor noise based on the measured distance (dynamic model)
                Length sensorNoise = utils::calculateV5DistanceSensorNoise(actualMeasurement);

                double confidenceFactor = sensorScore.score;

                // Calculate log probability (Gaussian model in log space)
                Length error = (actualMeasurement - expectedMeasurement);
                Area errorSquared = error * error;
                double logProbability = -1 * to_in2(errorSquared) * confidenceFactor;

                // Add to log position weight (multiplication becomes addition in log space)
                logPositionWeight += logProbability;
                particleValidSensorCount++;
            }

            // Use a uniform weight if no valid sensors were available
            if (particleValidSensorCount == 0)
            {
                particle.weight = 1.0 / m_particles.size();
            }
            else
            {
                // Store the log weight temporarily
                particle.weight = logPositionWeight;
                processedParticles++;
            }

            // Track the maximum log weight and best particle
            if (logPositionWeight > maxLogWeight)
            {
                maxLogWeight = logPositionWeight;
                bestParticle = &particle;
            }
        }

        // Subtract max log weight to avoid underflow when exponentiating
        double sumWeights = 0.0;
        for (auto &particle : m_particles)
        {
            if (particle.weight == 1.0 / m_particles.size()) {
                sumWeights += particle.weight;
                continue; // Skip uniform weights
            }
            // Convert from log weight to linear weight
            particle.weight = utils::fastExp(particle.weight - maxLogWeight);
            // Use a more appropriate minimum weight
            const double MIN_WEIGHT = 1e-30;
            particle.weight = std::max(particle.weight, MIN_WEIGHT);

            sumWeights += particle.weight;
        }

        // Normalize weights
        if (sumWeights > 0.0)
        {
            double invSumWeights = 1.0 / sumWeights;
            for (auto &particle : m_particles)
            {
                particle.weight *= invSumWeights;
            }
        }
    }

    void ParticleFilter::normalizeWeights()
    {
        // Calculate statistics for debugging
        double totalWeight = 0.0;
        double maxWeight = 0.0;
        double minWeight = 1.0;

        for (const auto &particle : m_particles)
        {
            totalWeight += particle.weight;
            maxWeight = std::max(maxWeight, particle.weight);
            minWeight = std::min(minWeight, particle.weight);
        }

        if (m_performanceLoggingEnabled)
        {
            std::cout << std::scientific << std::setprecision(8);
            std::cout << "Before normalization - Total: " << totalWeight
                      << ", Max: " << maxWeight
                      << ", Min: " << minWeight
                      << ", Ratio: " << (maxWeight > 0 ? minWeight / maxWeight : 1.0);
            std::cout << std::fixed << std::setprecision(2) << std::endl;
        }

        // Avoid division by zero
        if (totalWeight > 0.000001)
        {
            double invTotalWeight = 1.0 / totalWeight;
            for (auto &particle : m_particles)
            {
                // Multiply by inverse instead of dividing
                particle.weight *= invTotalWeight;
            }
        }
        else
        {
            // If all weights are zero, reset to uniform weights
            double uniformWeight = 1.0 / m_particles.size();
            for (auto &particle : m_particles)
            {
                particle.weight = uniformWeight;
            }
            if (m_performanceLoggingEnabled)
            {
                std::cout << "WARNING: All weights near zero - resetting to uniform" << std::endl;
            }
        }
    }

    void ParticleFilter::resample()
    {
        // Calculate effective sample size (ESS)
        double sumSquaredWeights = 0.0;
        double maxWeight = 0.0;

        for (const auto &particle : m_particles)
        {
            sumSquaredWeights += particle.weight * particle.weight;
            maxWeight = std::max(maxWeight, particle.weight);
        }

        // Avoid division by zero
        double effectiveSampleSize = (sumSquaredWeights > 1e-10) ? (1.0 / sumSquaredWeights) : 0.0;

        // Only resample if effective sample size is below threshold
        double threshold = m_numParticles * 0.75;

        // Force resampling more frequently to prevent particle stagnation
        static int updatesSinceResample = 0;
        updatesSinceResample++;

        if (effectiveSampleSize >= threshold)
        {
            return; // Skip resampling when particles are already well distributed
        }

        updatesSinceResample = 0; // Reset counter after resampling
        m_tempParticles.clear();  // Clear but keep capacity

        // Get current odometry pose for accurate heading-aligned particles
        units::Pose odomPose = m_odometry.getPose();

        // Keep the top N% of particles directly (increased for single-sensor scenarios)
        const double KEEP_PERCENTAGE = 0.35; // Increased from 0.20 for better single-sensor performance
        size_t keepCount = static_cast<size_t>(m_numParticles * KEEP_PERCENTAGE);
        
        // Always add some particles that respect the odometry heading
        const double ODOM_ALIGNED_PERCENTAGE = 0.15; // 15% of particles will be odometry-heading aligned
        size_t odomAlignedCount = static_cast<size_t>(m_numParticles * ODOM_ALIGNED_PERCENTAGE);
        
        // Calculate how many to resample normally
        size_t resampleCount = m_numParticles - keepCount - odomAlignedCount;

        // Sort particles by weight (only partially sort to get top keepCount)
        std::vector<size_t> indices(m_particles.size());
        for (size_t i = 0; i < indices.size(); ++i)
        {
            indices[i] = i;
        }
        
        std::partial_sort(
            indices.begin(),
            indices.begin() + keepCount,
            indices.end(),
            [this](size_t a, size_t b)
            { return m_particles[a].weight > m_particles[b].weight; });

        // Add the best particles directly to the new population
        for (size_t i = 0; i < keepCount; ++i)
        {
            m_tempParticles.push_back(m_particles[indices[i]]);
        }

        // For the rest, use low variance resampling
        // Reuse cumulative weights vector
        m_cumulativeWeights.clear();

        // Create cumulative weight array for faster sampling
        double sum = 0.0;
        for (const auto &particle : m_particles)
        {
            sum += particle.weight;
            m_cumulativeWeights.push_back(sum);
        }

        // Initialize tracking for unique particles
        std::vector<bool> particleSelected(m_particles.size(), false);
        size_t uniqueParticlesCount = keepCount; // Start with the ones we kept

        // Mark the directly kept particles as selected
        for (size_t i = 0; i < keepCount; ++i)
        {
            particleSelected[indices[i]] = true;
        }

        // Resample using low variance sampling algorithm for the remaining particles
        std::uniform_real_distribution<double> uniformDist(0.0, 1.0 / resampleCount);
        double r = uniformDist(m_rng);
        size_t i = 0;

        for (size_t j = 0; j < resampleCount; j++)
        {
            double u = r + j * (1.0 / resampleCount);

            while (u > m_cumulativeWeights[i] && i < m_cumulativeWeights.size() - 1)
            {
                i++;
            }

            // Track unique particles
            if (!particleSelected[i])
            {
                particleSelected[i] = true;
                uniqueParticlesCount++;
            }

            m_tempParticles.push_back(m_particles[i]);
        }

        // Add odometry-aligned particles (new section)
        // These particles will have positions from existing particles but orientations close to odometry
        for (size_t i = 0; i < odomAlignedCount; i++)
        {
            // Choose a particle to get position from (prefer from the best half)
            size_t sourceIndex = static_cast<size_t>(m_rng() % (m_particles.size() / 2));
            
            // Create a new particle with similar position but heading aligned with odometry
            units::Pose basePose = m_particles[sourceIndex].pose;
            
            // Small position variation
            Length x = basePose.x + m_motionNoise * 0.5 * m_normalDist(m_rng);
            Length y = basePose.y + m_motionNoise * 0.5 * m_normalDist(m_rng);
            
            // Very small heading variation from odometry (0.5 degree std dev)
            Angle theta = units::constrainAngle180(from_stDeg(to_stDeg(odomPose.orientation) + to_stDeg(0.5_stDeg * m_normalDist(m_rng))));
            
            // Create new pose and constrain to field
            units::Pose alignedPose(x, y, theta);
            alignedPose = utils::constrainToField(alignedPose);
            
            // Add to particle set with uniform weight
            m_tempParticles.emplace_back(alignedPose, 1.0 / m_numParticles);
        }

        // Add diversity when needed, but with much lower heading variance
        if (effectiveSampleSize < m_numParticles * 0.1)
        {
            const size_t numRandomParticles = m_numParticles * 0.05; // Add 5% random particles

            // Calculate the movement since last update
            static units::Pose lastEstimatedPose = m_estimatedPose;
            Area distanceMovedSquared = (m_estimatedPose.x - lastEstimatedPose.x) * (m_estimatedPose.x - lastEstimatedPose.x) +
                                        (m_estimatedPose.y - lastEstimatedPose.y) * (m_estimatedPose.y - lastEstimatedPose.y);
            lastEstimatedPose = m_estimatedPose;

            // If we're nearly stationary, focus on keeping headings aligned
            bool isStationary = (distanceMovedSquared < 0.01_in2);

            // Replace some of the resampled particles with random ones
            for (size_t i = 0; i < numRandomParticles && m_tempParticles.size() > keepCount + numRandomParticles; i++)
            {
                // Remove a resampled particle (not from the kept ones)
                m_tempParticles.pop_back();

                // Generate random pose near estimated pose
                Length x = m_estimatedPose.x + m_motionNoise * 5.0 * m_normalDist(m_rng);
                Length y = m_estimatedPose.y + m_motionNoise * 5.0 * m_normalDist(m_rng);
                
                // Initialize theta with a default value, then conditionally update it
                Angle theta = odomPose.orientation; // Initialize with odometry heading
                
                // Much lower heading variance for random particles, especially when stationary
                if (isStationary) {
                    // When stationary, very tightly constrain to odometry heading
                    theta = units::constrainAngle180(from_stDeg(to_stDeg(odomPose.orientation) + to_stDeg(0.5_stDeg * m_normalDist(m_rng))));
                } else {
                    theta = units::constrainAngle180(from_stDeg(to_stDeg(m_estimatedPose.orientation) + to_stDeg(2_stDeg * m_normalDist(m_rng))));
                }
                
                units::Pose randomPose(x, y, theta);
                randomPose = utils::constrainToField(randomPose);

                // Add the random particle
                m_tempParticles.emplace_back(randomPose, 1.0 / m_numParticles);
            }
        }

        // Reset all particles to uniform weights
        double baseWeight = 1.0 / m_numParticles;

        for (auto &particle : m_tempParticles)
        {
            particle.weight = baseWeight;
        }

        // Swap rather than move to avoid reallocations
        m_particles.swap(m_tempParticles);
    }

    units::Pose ParticleFilter::calculateEstimatedPose()
    {
        double totalWeight = 0.0;
        Length weightedX = 0_m;
        Length weightedY = 0_m;

        // For orientation, we need to average sin and cos to handle wrap-around
        double sinSum = 0.0;
        double cosSum = 0.0;

        // Get the odometry pose to blend with particle filter pose
        units::Pose odomPose = m_odometry.getPose();

        for (const auto &particle : m_particles)
        {
            double w = particle.weight;
            totalWeight += w;
            weightedX += particle.pose.x * w;
            weightedY += particle.pose.y * w;

            // Handle circular averaging of orientation
            sinSum += utils::fastSin(to_stRad(particle.pose.orientation)) * w;
            cosSum += utils::fastCos(to_stRad(particle.pose.orientation)) * w;
        }

        // Avoid division by zero
        if (totalWeight > 0.0)
        {
            weightedX = weightedX / totalWeight;
            weightedY = weightedY / totalWeight;

            // Calculate weighted orientation from particles
            Angle particleTheta = from_stRad(std::atan2(sinSum, cosSum));

            // Blend particle filter angle with odometry angle, strongly favoring odometry
            const double ODOM_ANGLE_TRUST_FACTOR = 0.97; // Increased from 0.95 to 0.97 for even more trust in odometry/IMU angle

            Angle blendedTheta = odomPose.orientation * ODOM_ANGLE_TRUST_FACTOR +
                                 particleTheta * (1.0 - ODOM_ANGLE_TRUST_FACTOR);
                                 
            // For position, use particle filter's estimate which is influenced by sensors
            return units::Pose(weightedX, weightedY, blendedTheta);
        }

        // Return last estimate if we can't calculate a new one
        return m_estimatedPose;
    }

    void ParticleFilter::addDistanceSensor(
        int sensorID,
        units::Pose sensorPosition, // Changed from units::Vector2D<Length> and removed sensorDirection
        std::function<Length()> getMeasurement,
        std::function<int()> getConfidence,
        std::function<Length(const units::Pose&)> getExpectedMeasurement)
    {
        m_sensors.push_back({sensorID,
                             sensorPosition, // Only passing position now, which contains both position and orientation
                             getMeasurement,
                             getConfidence,
                             getExpectedMeasurement});
    }

    units::Pose ParticleFilter::getPose() const
    {
        m_mutex.take(20);
        units::Pose pose = m_estimatedPose;
        m_mutex.give();
        return pose;
    }

    void ParticleFilter::resetPose(const units::Pose &initialPose)
    {
        m_mutex.take(20);
        initializeParticles(initialPose);
        m_lastOdometryPose = m_odometry.getPose();
        m_mutex.give();
    }

    void ParticleFilter::updateTaskTrampoline(void *param)
    {
        ParticleFilter *filter = static_cast<ParticleFilter *>(param);
        filter->taskUpdate();
    }

    void ParticleFilter::enableVisualization(bool enable, viz::FieldView *fieldView, int numParticlesToDraw)
    {
        m_visualizationEnabled = enable;
        m_fieldView = fieldView;
        m_numParticlesToDraw = numParticlesToDraw;

        // Reset visualization timer
        m_lastVisualizationTime = pros::millis();
    }

    std::vector<Particle> ParticleFilter::getTopParticles(size_t n) const
    {
        if (n >= m_particles.size())
        {
            return m_particles; // Return all particles if n is too large
        }

        // Create a temporary vector with just the needed size to avoid excess allocation
        std::vector<Particle> topParticles;
        topParticles.reserve(n);

        // Use partial sort instead of full sort to save processing time
        // Make a copy of just the elements we need to sort
        std::vector<Particle> sortableCopy(m_particles.begin(), m_particles.end());

        // Only sort as many elements as we need (partial sort is more efficient)
        std::partial_sort(
            sortableCopy.begin(),
            sortableCopy.begin() + n,
            sortableCopy.end(),
            [](const Particle &a, const Particle &b)
            { return a.weight > b.weight; });

        // Copy just the top n particles
        topParticles.assign(sortableCopy.begin(), sortableCopy.begin() + n);

        return topParticles;
    }

    void ParticleFilter::taskUpdate()
    {
        while (m_isRunning)
        {
            {
                m_mutex.take(20);
                update();
                
                // Visualize top particles once per second if enabled
                if (m_visualizationEnabled && m_fieldView != nullptr)
                {
                    uint32_t currentTime = pros::millis();
                    if (currentTime - m_lastVisualizationTime >= 500)
                    {
                        // Get top particles
                        auto topParticles = getTopParticles(m_numParticlesToDraw);

                        // Draw particles on the field view
                        m_fieldView->clearParticles();
                        for (const auto &particle : topParticles)
                        {
                            // Convert each particle to field view coordinates
                            double x = to_in(particle.pose.x);
                            double y = to_in(particle.pose.y);
                            double theta = to_cDeg(particle.pose.orientation);

                            // Map weight to intensity for visualization
                            // Calculate intensity such that even small weights are visible
                            // but higher weights stand out more (logarithmic mapping)
                            double relativeWeight = particle.weight / topParticles[0].weight; // Normalize to top particle
                            uint8_t intensity = static_cast<uint8_t>(std::min(255.0,
                                                                              100.0 + 155.0 * relativeWeight));

                            // Add particle to field view
                            m_fieldView->addParticle(x, y, theta, intensity);
                        }

                        // Also display the estimated pose in a distinct color
                        m_fieldView->setRobotPose(
                            to_in(m_estimatedPose.x),
                            to_in(m_estimatedPose.y),
                            to_cDeg(m_estimatedPose.orientation),
                            true // Use distinct color for estimated pose
                        );

                        m_lastVisualizationTime = currentTime;
                    }
                }

                m_mutex.give();
            }
            pros::delay(to_msec(m_updateInterval));
        }
    }

    void ParticleFilter::enablePerformanceLogging(bool enable, uint32_t logIntervalMs)
    {
        m_performanceLoggingEnabled = enable;
        m_performanceLogInterval = logIntervalMs;
        m_lastPerformanceLogTime = pros::millis();
        m_lastSensorLogTime = pros::millis(); // Reset this too to ensure first log happens promptly
        m_updatesSinceLastLog = 0;

        // Reset averages
        m_avgMotionUpdateTimeUs = 0;
        m_avgSensorUpdateTimeUs = 0;
        m_avgResampleTimeUs = 0;
        m_avgTotalUpdateTimeUs = 0;

        std::cout << "\n========== PARTICLE FILTER PERFORMANCE LOGGING "
                  << (enable ? "STARTED" : "STOPPED") << " ==========\n";
        std::cout << "Performance logging enabled: " << (enable ? "YES" : "NO") << "\n";
        std::cout << "Number of particles: " << m_numParticles << "\n";
        std::cout << "Number of sensors: " << m_sensors.size() << "\n";
        std::cout << "Logging interval: " << logIntervalMs << " ms\n";
        std::cout << "=============================================================\n\n";
    }

    void ParticleFilter::logPerformanceMetrics()
    {
        // Format timing data for display
        double motionTimeMs = m_avgMotionUpdateTimeUs / 1000.0;
        double sensorTimeMs = m_avgSensorUpdateTimeUs / 1000.0;
        double resampleTimeMs = m_avgResampleTimeUs / 1000.0;
        double totalTimeMs = m_avgTotalUpdateTimeUs / 1000.0;

        // Print formatted performance metrics
        std::cout << "\n===== PARTICLE FILTER PERFORMANCE (" << m_numParticles
                  << " particles, " << m_sensors.size() << " sensors) =====\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Motion update:  " << motionTimeMs << " ms ("
                  << (motionTimeMs / totalTimeMs * 100) << "%)\n";
        std::cout << "  Sensor update:  " << sensorTimeMs << " ms ("
                  << (sensorTimeMs / totalTimeMs * 100) << "%)\n";
        std::cout << "  Resample:       " << resampleTimeMs << " ms ("
                  << (resampleTimeMs / totalTimeMs * 100) << "%)\n";
        std::cout << "  Total update:   " << totalTimeMs << " ms\n";
        std::cout << "  Updates:        " << m_updatesSinceLastLog << " since last log\n";
        std::cout << "============================================================\n\n";
    }
    
    std::pair<LinearVelocity, AngularVelocity> ParticleFilter::getVelocity() const
    {
        m_mutex.take(20);
        auto velocities = std::make_pair(m_linearVelocity, m_angularVelocity);
        m_mutex.give();
        return velocities;
    }
} // namespace localization
