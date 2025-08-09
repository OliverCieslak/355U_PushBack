#include "tuning/MotionProfileTuner.hpp"

extern tuning::CharacterizationView characterizationView;
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
    characterizationView.showStatusMessage("Warning", "SD card not found, using default values");
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
    characterizationView.showStatusMessage("Info", "No calibration file found");
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
  
  // Update the characterization view with loaded values
  if (foundKs) characterizationView.updateKs(kS);
  if (foundKv) characterizationView.updateKv(kV);
  if (foundKa) characterizationView.updateKa(kA);
  
  // Show status message
  if (foundKs || foundKv || foundKa) {
    characterizationView.showStatusMessage("Success", "Calibration values loaded from SD card");
    return true;
  } else {
    characterizationView.showStatusMessage("Warning", "No valid calibration values found");
    return false;
  }
}

/**
 * Save the calibration values to a file on the SD card
 */
void saveFeedForwardCalibrationValues() {
  // Check if SD card is installed
  if (!pros::usd::is_installed()) {
    characterizationView.showStatusMessage("Warning", "SD card not found, values not saved");
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
    characterizationView.showStatusMessage("Error", "Failed to open file for writing");
    return;
  }
  
  // Write the calibrated values
  fprintf(file, "# Drive Train Calibration Values\n");
  fprintf(file, "kS: %.6f\n", kS);
  fprintf(file, "kV: %.6f\n", kV);
  fprintf(file, "kA: %.6f\n", kA);
  
  fclose(file);
  characterizationView.showStatusMessage("Success", "Calibration values saved to SD card");
}

/**
 * Dedicated routine for tuning the kS parameter (static friction)
 * Enhanced with safety limits and better error handling
 */
void tuneKs() {  
  // Create the feedforward tuner
  tuning::FeedforwardTuner feedforwardTuner;
  
  // Set the wheel diameter from the characterization view
  feedforwardTuner.setWheelDiameter(wheelDiameter);
  
  // Clear any existing callbacks
  feedforwardTuner.clearCallbacks();
  
  // Add callback to update the characterization view with data points
  feedforwardTuner.addVelocityDataCallback(
    [](double voltage, double velocity) {
      characterizationView.addVelocityDataPoint(voltage, velocity);
      // Show safety status
      if (voltage >= 2.8) {
        characterizationView.showStatusMessage("Safety", "WARNING: Approaching 3.0V limit");
      }
    }
  );
  
  characterizationView.showKsTest();
  characterizationView.showStatusMessage("Safety", "Max voltage: 3.0V | Timeout: 15s");
  
  try {
    // Run the kS tuning routine with safety features
    kS = feedforwardTuner.tuneKs();
    
    // Update characterization view with the result
    characterizationView.updateKs(kS);
    characterizationView.showStatusMessage("kS Measured", ("Value: " + std::to_string(kS)).c_str());
    
    // Display final result
    printf("kS Calibration (Safe)\n");
    printf("kS: %.4f V\n", kS);
    
    // Save calibration value to SD card
    saveFeedForwardCalibrationValues();
    
  } catch (const std::exception& e) {
    characterizationView.showStatusMessage("Error", e.what());
    printf("kS tuning failed: %s\n", e.what());
  }
  
  characterizationView.showKsTest();
}

/**
 * Dedicated routine for tuning the kV parameter (velocity constant)
 * Enhanced with safety limits and speed monitoring
 */
void tuneKv() {
  // Create the feedforward tuner
  tuning::FeedforwardTuner feedforwardTuner;
  
  // Set the wheel diameter from the characterization view
  feedforwardTuner.setWheelDiameter(wheelDiameter);
  
  // Clear any existing callbacks
  feedforwardTuner.clearCallbacks();
  
  // Add callback to update the characterization view with data points
  feedforwardTuner.addVelocityDataCallback(
    [](double voltage, double velocity) {
      characterizationView.addVelocityDataPoint(voltage, velocity);
      // Show safety status
      if (voltage >= 7.5) {
        characterizationView.showStatusMessage("Safety", "WARNING: Approaching 8.0V limit");
      }
      if (velocity >= 55.0) {
        characterizationView.showStatusMessage("Safety", "WARNING: Approaching 60 in/s limit");
      }
    }
  );
  
  characterizationView.showKvTest();
  characterizationView.showStatusMessage("Safety", "Max voltage: 8.0V | Max speed: 60 in/s | Timeout: 30s");
  
  try {
    // Run the kV tuning routine with safety features
    kV = feedforwardTuner.tuneKv(kS);
      
    // Update characterization view with the result
    characterizationView.updateKv(kV);
    characterizationView.showStatusMessage("kV Measured", ("Value: " + std::to_string(kV)).c_str());
    
    // Display final result
    printf("kV Calibration (Safe)\n");
    printf("kV: %.4f V/(in/s)\n", kV);
    
    // Display for a few seconds
    pros::delay(3000);
    
    // Save calibration values to SD card
    saveFeedForwardCalibrationValues();
    
  } catch (const std::exception& e) {
    characterizationView.showStatusMessage("Error", e.what());
    printf("kV tuning failed: %s\n", e.what());
  }
}

/**
 * Dedicated routine for tuning the kA parameter (acceleration constant)
 */
void tuneKa() {
  // Create the feedforward tuner
  tuning::FeedforwardTuner feedforwardTuner;
  
  // Set the wheel diameter from the characterization view
  feedforwardTuner.setWheelDiameter(wheelDiameter);
  
  // Clear any existing callbacks
  feedforwardTuner.clearCallbacks();
  
  // Add callback to update the characterization view with data points
  feedforwardTuner.addAccelerationDataCallback(
    [](double acceleration, double voltage) {
      characterizationView.addAccelerationDataPoint(acceleration, voltage);
    }
  );
  
  characterizationView.showKaTest();
  
  // Run the kA tuning routine
  kA = feedforwardTuner.tuneKa(kS, kV);
  
  // Update characterization view with the result
  characterizationView.updateKa(kA);
  characterizationView.showStatusMessage("kA Measured", ("Value: " + std::to_string(kA)).c_str());
  
  // Save calibration values to SD card
  saveFeedForwardCalibrationValues();
}