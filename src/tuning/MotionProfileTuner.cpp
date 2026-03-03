#include "tuning/MotionProfileTuner.hpp"
#include <algorithm>
#include <cmath>

extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;

extern Length wheelDiameter;
extern LinearAcceleration maxAccel;
extern Number kS;  // Static friction (volts)
extern Number kV;  // Velocity feedforward (volts per velocity)
extern Number kA;  // Acceleration feedforward (volts per acceleration)

/**
 * Load the calibration values from the SD card if they exist
 * and if the current values are 0
 * @return true if values were loaded successfully, false otherwise
 */
bool loadFeedForwardCalibrationValues() {
  // Check if SD card is installed
  if (!pros::usd::is_installed()) {
    printf("[FF Calib] Warning: SD card not found, using default values\n");
    return false;
  }
  
  // Only load if current values are 0 (unset)
  if (kS != 0 && kV != 0 && kA != 0) {
    return false;  // Values already set
  }
  
  // Path to the calibration file
  const char* file_path = "/usd/ff_drive_calibration.txt";
  FILE *file;
  
  // Open file for reading
  file = fopen(file_path, "r");
  if (!file) {
    printf("[FF Calib] Info: No calibration file found\n");
    return false;
  }
  
  bool foundKs = false;
  bool foundKv = false;
  bool foundKa = false;
  char line[128];
  
  // Read the file line by line
  while (fgets(line, sizeof(line), file)) {
    // Skip comment lines
    if (line[0] == '#') continue;
    
    // Parse for kS
    if (strstr(line, "kS:") && kS == 0) {
      float value;
      if (sscanf(line, "kS: %f", &value) == 1) {
        kS = value;
        foundKs = true;
      }
    }
    
    // Parse for kV
    else if (strstr(line, "kV:") && kV == 0) {
      float value;
      if (sscanf(line, "kV: %f", &value) == 1) {
        kV = value;
        foundKv = true;
      }
    }
    
    // Parse for kA
    else if (strstr(line, "kA:") && kA == 0) {
      float value;
      if (sscanf(line, "kA: %f", &value) == 1) {
        kA = value;
        foundKa = true;
      }
    }
  }
  
  fclose(file);
  
  // Show status message
  if (foundKs || foundKv || foundKa) {
    printf("[FF Calib] Loaded from SD: kS=%.6f kV=%.6f kA=%.6f\n", (double)kS, (double)kV, (double)kA);
    return true;
  } else {
    printf("[FF Calib] Warning: No valid calibration values found\n");
    return false;
  }
}

/**
 * Save the calibration values to a file on the SD card
 */
void saveFeedForwardCalibrationValues() {
  // Check if SD card is installed
  if (!pros::usd::is_installed()) {
    printf("[FF Calib] Warning: SD card not found, values not saved\n");
    return;
  }
  
  // Path to the calibration file
  const char* file_path = "/usd/ff_drive_calibration.txt";
  FILE *file;
  
  // Create file if it doesn't exist
  file = fopen(file_path, "a");
  fclose(file);
  
  // Open file for writing
  file = fopen(file_path, "w");
  if (!file) {
    printf("[FF Calib] Error: Failed to open file for writing\n");
    return;
  }
  
  // Write the calibrated values
  fprintf(file, "# Drive Train Calibration Values\n");
  fprintf(file, "kS: %.6f\n", kS);
  fprintf(file, "kV: %.6f\n", kV);
  fprintf(file, "kA: %.6f\n", kA);
  
  fclose(file);
  printf("[FF Calib] Saved to SD: kS=%.6f kV=%.6f kA=%.6f\n", (double)kS, (double)kV, (double)kA);
}

/**
 * Dedicated routine for tuning the kS parameter (static friction)
 */
void tuneKs() {  
  // Create the feedforward tuner
  tuning::FeedforwardTuner feedforwardTuner;
  
  // Set the wheel diameter
  feedforwardTuner.setWheelDiameter(wheelDiameter);
  
  // Clear any existing callbacks
  feedforwardTuner.clearCallbacks();
  
  // Add callback to print data points
  feedforwardTuner.addVelocityDataCallback(
    [](double voltage, double velocity) {
      printf("  V=%.3f vel=%.3f\n", voltage, velocity);
    }
  );
  
  printf("=== kS Calibration ===\n");
  printf("Safety: Max voltage 10.0V | Timeout 15s\n");
  
  try {
    constexpr int numRuns = 7;
    std::vector<double> ksResults;
    ksResults.reserve(numRuns);
    for (int i = 0; i < numRuns; ++i) {
      printf("\n--- kS Calibration Run %d/%d ---\n", i+1, numRuns);
      double result = feedforwardTuner.tuneKs();
      ksResults.push_back(result);
      pros::delay(1000); // Pause between runs
    }

    // Compute mean and standard deviation
    double sum = 0.0;
    for (double v : ksResults) sum += v;
    double mean = sum / ksResults.size();
    double sqSum = 0.0;
    for (double v : ksResults) sqSum += (v - mean) * (v - mean);
    double stddev = ksResults.size() > 1 ? std::sqrt(sqSum / (ksResults.size() - 1)) : 0.0;

    kS = mean;

    printf("\n==== kS Calibration Results ====\n");
    for (int i = 0; i < numRuns; ++i) {
      printf("Run %d: %.4f V\n", i+1, ksResults[i]);
    }
    printf("Mean: %.4f V\n", mean);
    printf("Stddev: %.4f V\n", stddev);
    printf(">>> kS = %.6f <<<\n", mean);

    // Save calibration value to SD card
    saveFeedForwardCalibrationValues();

  } catch (const std::exception& e) {
    printf("kS tuning failed: %s\n", e.what());
  }
}

/**
 * Dedicated routine for tuning the kV parameter (velocity constant)
 */
void tuneKv() {
  // Create the feedforward tuner
  tuning::FeedforwardTuner feedforwardTuner;
  
  // Set the wheel diameter
  feedforwardTuner.setWheelDiameter(wheelDiameter);
  
  // Clear any existing callbacks
  feedforwardTuner.clearCallbacks();
  
  // Add callback to print data points
  feedforwardTuner.addVelocityDataCallback(
    [](double voltage, double velocity) {
      printf("  V=%.3f vel=%.3f\n", voltage, velocity);
    }
  );
  
  printf("=== kV Calibration ===\n");
  printf("Safety: Max voltage 8.0V | Max speed 60 in/s | Timeout 30s\n");
  
  try {
    // Run the kV tuning routine
    kV = feedforwardTuner.tuneKv(kS);
    
    printf(">>> kV = %.6f V/(in/s) <<<\n", (double)kV);
    
    pros::delay(3000);
    
    // Save calibration values to SD card
    saveFeedForwardCalibrationValues();
    
  } catch (const std::exception& e) {
    printf("kV tuning failed: %s\n", e.what());
  }
}

/**
 * Dedicated routine for tuning the kA parameter (acceleration constant)
 */
void tuneKa() {
  // Create the feedforward tuner
  tuning::FeedforwardTuner feedforwardTuner;
  
  // Set the wheel diameter
  feedforwardTuner.setWheelDiameter(wheelDiameter);
  
  // Clear any existing callbacks
  feedforwardTuner.clearCallbacks();
  
  // Add callback to print data points
  feedforwardTuner.addAccelerationDataCallback(
    [](double acceleration, double voltage) {
      printf("  accel=%.3f V=%.3f\n", acceleration, voltage);
    }
  );
  
  printf("=== kA Calibration ===\n");
  
  // Run the kA tuning routine
  kA = feedforwardTuner.tuneKa(kS, kV);
  
  printf(">>> kA = %.6f <<<\n", (double)kA);
  
  // Save calibration values to SD card
  saveFeedForwardCalibrationValues();
}

/**
 * Dedicated routine for tuning the maxJerk parameter.
 *
 * Measures the acceleration ramp after a voltage step, then uses linear
 * regression on the rising portion to compute the slope (= jerk).
 *
 * Signal processing:
 *   1. 5ms samples → 150ms sliding-window average on velocity
 *   2. Central difference with 50ms span for acceleration
 *   3. Linear regression on the acceleration ramp (10%→peak) for jerk
 *
 * This avoids the noise amplification of double-differentiation and gives
 * a robust jerk estimate from many data points rather than just two.
 *
 * The robot needs ~2 ft of straight driving space.
 */
void tuneMaxJerk() {
    printf("=== Max Jerk Characterization ===\n");
    printf("Robot will drive forward then backward, 3 times each.\n");
    printf("Ensure ~2 ft of clear space ahead.\n");
    pros::delay(2000);

    leftMotors.setBrakeMode(lemlib::BrakeMode::COAST);
    rightMotors.setBrakeMode(lemlib::BrakeMode::COAST);

    constexpr int NUM_TRIALS = 3;
    constexpr double STEP_VOLTAGE = 8.0;        // Volts
    constexpr int SAMPLE_PERIOD_MS = 5;          // 5ms sample rate for more data
    constexpr int SETTLE_MS = 800;               // Wait for complete stop
    constexpr int DRIVE_MS = 500;                // Less distance (~2 ft)
    constexpr int TOTAL_SAMPLES = DRIVE_MS / SAMPLE_PERIOD_MS; // 100 samples
    constexpr int SMOOTH_WINDOW = 31;            // 155ms sliding window (31 × 5ms)
    constexpr int DIFF_SPAN = 10;                // 50ms central difference (10 × 5ms each side)
    constexpr double RAMP_THRESH = 0.10;         // Start of ramp = 10% of peak accel
    constexpr double SAFETY_FACTOR = 0.50;

    std::vector<double> measuredJerks;
    measuredJerks.reserve(NUM_TRIALS * 2);

    auto runTrial = [&](double voltSign, int trialNum) {
        printf("\n--- Trial %d (dir=%s) ---\n", trialNum, voltSign > 0 ? "fwd" : "rev");

        // Ensure fully stopped
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(SETTLE_MS);

        // Collect raw velocity samples
        std::vector<double> rawVel;
        rawVel.reserve(TOTAL_SAMPLES);

        double dt = SAMPLE_PERIOD_MS / 1000.0;
        uint32_t now = pros::millis();

        // Command voltage step
        double pct = (voltSign * STEP_VOLTAGE) / 12.0;
        leftMotors.move(pct);
        rightMotors.move(pct);

        for (int i = 0; i < TOTAL_SAMPLES; ++i) {
            pros::Task::delay_until(&now, SAMPLE_PERIOD_MS);
            double lv = prosLeftMotors.get_actual_velocity() * to_in(wheelDiameter) * M_PI / 60.0;
            double rv = prosRightMotors.get_actual_velocity() * to_in(wheelDiameter) * M_PI / 60.0;
            rawVel.push_back(std::fabs((lv + rv) / 2.0));
        }

        // Stop
        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(SETTLE_MS);

        // Heavy sliding-window average on velocity (155ms window)
        std::vector<double> smoothVel(TOTAL_SAMPLES, 0.0);
        int halfWin = SMOOTH_WINDOW / 2;
        for (int i = 0; i < TOTAL_SAMPLES; ++i) {
            double sum = 0.0;
            int count = 0;
            int lo = std::max(0, i - halfWin);
            int hi = std::min(TOTAL_SAMPLES - 1, i + halfWin);
            for (int j = lo; j <= hi; ++j) {
                sum += rawVel[j];
                ++count;
            }
            smoothVel[i] = sum / count;
        }

        // Compute acceleration using wide central difference for noise rejection
        // a[i] = (v[i+DIFF_SPAN] - v[i-DIFF_SPAN]) / (2 * DIFF_SPAN * dt)
        int validStart = DIFF_SPAN;
        int validEnd = TOTAL_SAMPLES - DIFF_SPAN;
        std::vector<double> accel(TOTAL_SAMPLES, 0.0);
        for (int i = validStart; i < validEnd; ++i) {
            accel[i] = (smoothVel[i + DIFF_SPAN] - smoothVel[i - DIFF_SPAN])
                      / (2.0 * DIFF_SPAN * dt);
        }

        // Find peak acceleration (only in valid range)
        double peakAccel = 0.0;
        int peakIdx = 0;
        for (int i = validStart; i < validEnd; ++i) {
            if (accel[i] > peakAccel) {
                peakAccel = accel[i];
                peakIdx = i;
            }
        }

        if (peakAccel < 5.0) {
            printf("  WARNING: Peak accel too low (%.1f in/s²), skipping\n", peakAccel);
            return;
        }

        // Find start of acceleration ramp (10% of peak)
        double threshVal = RAMP_THRESH * peakAccel;
        int rampStart = -1;
        for (int i = validStart; i < peakIdx; ++i) {
            if (accel[i] >= threshVal) {
                rampStart = i;
                break;
            }
        }

        int rampN = (rampStart >= 0) ? (peakIdx - rampStart + 1) : 0;
        if (rampN < 4) {
            printf("  WARNING: Ramp too short (%d pts), skipping\n", rampN);
            return;
        }

        // Linear regression: accel = slope*t + intercept on ramp region
        // slope = jerk
        double sumT = 0, sumA = 0, sumTT = 0, sumTA = 0;
        for (int i = rampStart; i <= peakIdx; ++i) {
            double t = (i - rampStart) * dt;
            double a = accel[i];
            sumT  += t;
            sumA  += a;
            sumTT += t * t;
            sumTA += t * a;
        }
        double denom = rampN * sumTT - sumT * sumT;
        if (std::fabs(denom) < 1e-12) {
            printf("  WARNING: Regression degenerate, skipping\n");
            return;
        }
        double slope = (rampN * sumTA - sumT * sumA) / denom;

        double rampDur = (peakIdx - rampStart) * dt;
        printf("  Peak accel: %.1f in/s²  at t=%.0f ms\n", peakAccel, peakIdx * dt * 1000.0);
        printf("  Ramp: %.0f ms  (%d pts, from t=%.0f ms)\n",
               rampDur * 1000.0, rampN, rampStart * dt * 1000.0);
        printf("  Jerk (regression): %.1f in/s³\n", slope);

        if (slope > 0) {
            measuredJerks.push_back(slope);
        } else {
            printf("  WARNING: Negative jerk slope, skipping\n");
        }
    };

    for (int i = 0; i < NUM_TRIALS; ++i) {
        runTrial(1.0, i + 1);
        runTrial(-1.0, i + 1);
    }

    if (measuredJerks.empty()) {
        printf("\nERROR: No valid trials. Check motors and encoder connections.\n");
        return;
    }

    // Statistics
    double sum = 0.0;
    for (double j : measuredJerks) sum += j;
    double mean = sum / measuredJerks.size();
    double minVal = *std::min_element(measuredJerks.begin(), measuredJerks.end());
    double recommended = minVal * SAFETY_FACTOR;

    printf("\n==== Max Jerk Results ====\n");
    for (size_t i = 0; i < measuredJerks.size(); ++i) {
        printf("  Trial %zu: %.1f in/s³\n", i + 1, measuredJerks[i]);
    }
    printf("Mean:  %.1f in/s³\n", mean);
    printf("Min:   %.1f in/s³\n", minVal);
    printf("Safety factor: %.0f%%\n", SAFETY_FACTOR * 100.0);
    printf("\n>>> Recommended maxJerk = %.1f in/s³ <<<\n", recommended);
    printf("\nSet in main.cpp:\n");
    printf("  LinearJerk maxJerk = %.1f_inps3;\n", recommended);
    double accelRatio = (to_inps2(maxAccel) > 0.01) ? (recommended / to_inps2(maxAccel)) : 0.0;
    printf("Or: LinearJerk maxJerk = %.2f * maxAccel / 1_sec;\n", accelRatio);
}