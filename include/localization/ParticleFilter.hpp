#pragma once

#include "odometry/SkidSteerOdometry.hpp"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "utils/DistanceUtils.hpp"
#include "utils/FastMath.hpp"
#include "viz/FieldView.hpp"
#include <functional>
#include <random>
#include <vector>
#include <unordered_map>

namespace localization {

/**
 * @brief A class representing a single particle in the particle filter
 */
struct Particle {
    units::Pose pose;   // Position and orientation of the particle
    double weight;      // Weight of the particle (likelihood)
    
    Particle(const units::Pose& pose, double weight = 1.0)
        : pose(pose), weight(weight) {}
};

/**
 * @brief Class that implements a particle filter for robot localization
 */
class ParticleFilter {
public:
    /**
     * @brief Construct a new Particle Filter object
     * 
     * @param odometry Reference to odometry for motion updates
     * @param initialPose Initial pose of the robot with reasonable certainty
     * @param numParticles Number of particles to use (default: 500)
     * @param motionNoise Standard deviation for motion uncertainty (default: 0.5 inches)
     * @param angleNoise Minimum standard deviation for angle uncertainty (default: .5 degrees)
     * @param updateInterval Interval between filter updates (default: 30ms)
     * 
     */
    ParticleFilter(
        odometry::SkidSteerOdometry& odometry,
        const units::Pose& initialPose,
        size_t numParticles = 500,
        Length motionNoise = 0.5_in,
        Angle angleNoise = 0.5_stDeg,
        Time updateInterval = 10_msec
    );
    
    /**
     * @brief Destructor to ensure task is properly stopped
     */
    ~ParticleFilter();
    
    /**
     * @brief Start the particle filter update task
     */
    void start();
    
    /**
     * @brief Stop the particle filter update task
     */
    void stop();
    
    /**
     * @brief Get the current estimated pose of the robot
     * 
     * @return units::Pose The current estimated pose
     */
    units::Pose getPose() const;
    
    /**
     * @brief Reset the filter with a new initial pose
     * 
     * @param initialPose The new initial pose
     */
    void resetPose(const units::Pose& initialPose);
    
    /**
     * @brief Add a distance sensor measurement function
     * 
     * @param sensorID Unique ID for this sensor
     * @param sensorPosition Position and orientation of the sensor relative to the robot center
     * @param getMeasurement Function that returns the current sensor measurement
     * @param getConfidence Function that returns the sensor's confidence value (0-63)
     * @param getExpectedMeasurement Function that calculates expected distance for a pose
     */
    void addDistanceSensor(
        int sensorID,
        units::Pose sensorPosition, // Changed from units::Vector2D<Length> and removed sensorDirection
        std::function<Length()> getMeasurement,
        std::function<int()> getConfidence,
        std::function<Length(const units::Pose&)> getExpectedMeasurement
    );
    
    /**
     * @brief Update the particle filter using the latest sensor and odometry data
     * 
     * @return units::Pose The current estimated pose
     */
    units::Pose update();
    
    /**
     * @brief Enable or disable visualization of the top particles
     * 
     * @param enable Whether to enable visualization
     * @param fieldView Pointer to the FieldView to draw on (nullptr to disable)
     * @param numParticlesToDraw Number of top particles to visualize (default: 10)
     */
    void enableVisualization(bool enable, viz::FieldView* fieldView = nullptr, int numParticlesToDraw = 10);
    
    /**
     * @brief Get the top N particles with highest weights
     * 
     * @param n Number of particles to return
     * @return std::vector<Particle> Vector of top n particles
     */
    std::vector<Particle> getTopParticles(size_t n) const;

    /**
     * @brief Enable or disable performance logging to terminal
     * 
     * @param enable Whether to enable logging
     * @param logIntervalMs Interval between logs in milliseconds (default: 5000)
     */
    void enablePerformanceLogging(bool enable, uint32_t logIntervalMs = 5000);

    /**
     * @brief Get the current robot velocities
     * 
     * @return std::pair<LinearVelocity, AngularVelocity> Linear and angular velocity
     */
    std::pair<LinearVelocity, AngularVelocity> getVelocity() const;

private:
    odometry::SkidSteerOdometry& m_odometry;
    std::vector<Particle> m_particles;
    std::vector<Particle> m_tempParticles;  // Pre-allocated vector for resampling
    std::vector<double> m_cumulativeWeights; // Pre-allocated vector for weights
    std::vector<utils::SensorScore> m_sensorScores{}; // Initialize with empty braces to avoid default constructor
    units::Pose m_estimatedPose;
    units::Pose m_lastOdometryPose;
    size_t m_numParticles;
    Length m_motionNoise;
    Angle m_angleNoise;
    Time m_updateInterval;
    
    // Random number generation
    std::mt19937 m_rng;
    std::normal_distribution<double> m_normalDist;
    
    // Mutex for thread-safe access
    mutable pros::Mutex m_mutex;
    
    // Task management
    pros::Task* m_updateTask = nullptr;
    bool m_isRunning = false;
    
    // Sensor definitions
    struct DistanceSensor {
        int id;
        units::Pose position; // Changed from units::Vector2D<Length> to units::Pose
        std::function<Length()> getMeasurement;
        std::function<int()> getConfidence;  // Added for V5 distance sensor confidence level
        std::function<Length(const units::Pose&)> getExpectedMeasurement;
    };
    std::vector<DistanceSensor> m_sensors;
    
    // Visualization options
    bool m_visualizationEnabled = false;
    viz::FieldView* m_fieldView = nullptr;
    int m_numParticlesToDraw = 10;
    uint32_t m_lastVisualizationTime = 0; // For rate limiting visualization
    
    // Performance tracking
    bool m_performanceLoggingEnabled = false;
    uint32_t m_performanceLogInterval = 5000; // 5 seconds
    uint32_t m_lastPerformanceLogTime = 0;
    uint32_t m_lastSensorLogTime = 0;  // Add timestamp for sensor log rate limiting
    
    // Timing tracking (in microseconds)
    uint32_t m_motionUpdateTimeUs = 0;
    uint32_t m_sensorUpdateTimeUs = 0;
    uint32_t m_resampleTimeUs = 0;
    uint32_t m_totalUpdateTimeUs = 0;
    
    // Moving averages for smoother reporting
    double m_avgMotionUpdateTimeUs = 0;
    double m_avgSensorUpdateTimeUs = 0;
    double m_avgResampleTimeUs = 0;
    double m_avgTotalUpdateTimeUs = 0;
    
    // Count of updates since last log
    uint32_t m_updatesSinceLastLog = 0;
    
    /**
     * @brief Log the current performance metrics to terminal
     */
    void logPerformanceMetrics();
    
    /**
     * @brief Initialize particles around a given pose
     * 
     * @param initialPose The initial pose
     */
    void initializeParticles(const units::Pose& initialPose);
    
    /**
     * @brief Perform motion update on all particles
     * 
     * @param odometryDelta Change in pose from odometry
     */
    void motionUpdate(const units::Pose& odometryDelta);
    
    /**
     * @brief Update particle weights based on sensor measurements
     */
    void sensorUpdate();
    
    /**
     * @brief Resample particles based on their weights
     */
    void resample();
    
    /**
     * @brief Calculate estimated pose from weighted particles
     * 
     * @return units::Pose The estimated pose
     */
    units::Pose calculateEstimatedPose();
    
    /**
     * @brief Static trampoline function for the PROS task
     * 
     * @param param Pointer to the ParticleFilter instance
     */
    static void updateTaskTrampoline(void* param);
    
    /**
     * @brief The task update loop that runs continuously
     */
    void taskUpdate();
    
    /**
     * @brief Update particle weights based on sensor measurements using specified sensors
     * 
     * @param selectedSensors Vector of sensor IDs to use
     */
    void updateParticleWeights(const std::vector<utils::SensorScore>& selectedSensors);
    
    /**
     * @brief Normalize the weights of all particles
     */
    void normalizeWeights();
    
    // Cached sensor measurements to avoid repeated allocations
    std::vector<Length> m_sensorMeasurements;
    std::unordered_map<int, Length> m_selectedSensorMeasurements;

    // Velocity tracking
    uint32_t m_lastVelocityTime = 0;
    LinearVelocity m_linearVelocity = 0_mps;
    AngularVelocity m_angularVelocity = 0_radps;
    units::Pose m_prevVelocityPose;
};

} // namespace localization
