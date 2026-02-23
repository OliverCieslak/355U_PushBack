#include "tuning/MotionProfileTuner.hpp"

extern lemlib::MotorGroup leftMotors;
extern lemlib::MotorGroup rightMotors;
extern pros::MotorGroup prosLeftMotors;
extern pros::MotorGroup prosRightMotors;

extern Length wheelDiameter;
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