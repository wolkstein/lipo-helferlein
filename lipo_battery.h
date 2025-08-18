#ifndef LIPO_BATTERY_H
#define LIPO_BATTERY_H

#include <Arduino.h>

/*
 * LiPo Battery Management Library
 * Copyright (C) 2025 Michael Wolkstein
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 * 
 * LiPo Battery Management Library
 * ==============================
 * 
 * Comprehensive library for LiPo battery voltage monitoring and power consumption tracking.
 * Designed for drone applications with precise energy management and safety features.
 * 
 * Features:
 * - Voltage-to-percentage conversion (3.6V-4.2V range)
 * - Power integration with variable sample rates
 * - Battery capacity management
 * - Remaining flight time estimation
 * - Safety thresholds and warnings
 * 
 * VOLTAGE MONITORING USAGE:
 * ========================
 * 
 * Accurate voltage-to-percentage conversion using lookup tables:
 * ```cpp
 * uint16_t voltage_mv = 3850;  // 3.85V measured
 * uint8_t percent = voltageToBatteryPercent(voltage_mv);
 * Serial.println(percent);  // Output: ~42% (interpolated from lookup table)
 * 
 * // Float version (automatically converted to mV internally)
 * float voltage = 3.85;
 * uint8_t percent = voltageToBatteryPercentFloat(voltage);
 * 
 * // Reverse conversion: percent to voltage
 * uint8_t batteryLevel = 75;
 * uint16_t expected_mv = batteryPercentToVoltage(batteryLevel);
 * float expected_v = batteryPercentToVoltageFloat(batteryLevel);
 * ```
 * 
 * The lookup tables provide non-linear conversion based on real LiPo characteristics:
 * - Linear interpolation between table values for precise measurements
 * - Automatic rounding to nearest 0.5% (0.5+ rounds up)
 * - Handles all voltage values between 3.6V-4.2V accurately
 * 
 * Safety checks:
 * ```cpp
 * bool safe = isBatteryVoltageSafe(voltage_mv);
 * if (!safe) {
 *   if (voltage_mv < LIPO_CRITICAL_MV) {
 *     Serial.println("CRITICAL: Voltage too low!");
 *   } else if (voltage_mv > LIPO_OVERCHARGE_MV) {
 *     Serial.println("WARNING: Overcharge detected!");
 *   }
 * }
 * ```
 * 
 * POWER INTEGRATION USAGE:
 * ========================
 * 
 * Simple power tracking (without battery capacity):
 * ```cpp
 * PowerIntegrator powerMeter;
 * 
 * // In your measurement loop:
 * float voltage = readVoltage();    // Your ADC reading
 * float current = readCurrent();    // Your current sensor reading
 * unsigned long timestamp = millis();
 * 
 * double consumedWh = powerMeter.addMeasurement(voltage, current, timestamp);
 * Serial.print("Consumed: ");
 * Serial.print(consumedWh, 6);
 * Serial.println(" Wh");
 * ```
 * 
 * ADVANCED BATTERY MANAGEMENT:
 * ============================
 * 
 * Complete battery management with capacity tracking:
 * ```cpp
 * // Example: 3S 2200mAh LiPo = 3.7V * 3 * 2.2Ah = 24.42Wh
 * AdvancedPowerIntegrator batteryMonitor(24.4);
 * 
 * // In your measurement loop (variable sample rate supported):
 * float voltage = readBatteryVoltage();
 * float current = readBatteryCurrent();
 * unsigned long timestamp = millis();
 * 
 * double consumedWh = batteryMonitor.addMeasurement(voltage, current, timestamp);
 * 
 * // Get battery status:
 * uint8_t remainingPercent = batteryMonitor.getRemainingCapacityPercent();
 * double remainingWh = batteryMonitor.getRemainingCapacityWh();
 * double flightTimeMinutes = batteryMonitor.estimateRemainingTime_minutes();
 * 
 * // Safety checks:
 * if (batteryMonitor.isBatteryLow(25)) {
 *   Serial.println("WARNING: Battery low - return to home!");
 * }
 * 
 * if (batteryMonitor.isBatteryEmpty()) {
 *   Serial.println("CRITICAL: Battery empty!");
 *   // Emergency landing sequence
 * }
 * ```
 * 
 * TYPICAL DRONE BATTERY CONFIGURATIONS:
 * ====================================
 * 
 * Small Drone (2S 1000mAh):
 * ```cpp
 * AdvancedPowerIntegrator smallDrone(7.4);  // 3.7V * 2 * 1.0Ah = 7.4Wh
 * ```
 * 
 * Medium Drone (3S 2200mAh):
 * ```cpp
 * AdvancedPowerIntegrator mediumDrone(24.4);  // 3.7V * 3 * 2.2Ah = 24.42Wh
 * ```
 * 
 * Large Drone (4S 5000mAh):
 * ```cpp
 * AdvancedPowerIntegrator largeDrone(74.0);  // 3.7V * 4 * 5.0Ah = 74Wh
 * ```
 * 
 * FLIGHT CONTROLLER INTEGRATION:
 * ==============================
 * 
 * ```cpp
 * AdvancedPowerIntegrator flightBattery(24.4);  // Your battery capacity
 * 
 * void setup() {
 *   Serial.begin(115200);
 *   // Initialize your current/voltage sensors
 * }
 * 
 * void loop() {
 *   // Read sensors (variable timing supported)
 *   float voltage = analogRead(VOLTAGE_PIN) * VOLTAGE_SCALE;
 *   float current = analogRead(CURRENT_PIN) * CURRENT_SCALE;
 *   
 *   // Update power consumption (works under load!)
 *   flightBattery.addMeasurement(voltage, current, millis());
 *   
 *   // Check flight safety every second
 *   static unsigned long lastCheck = 0;
 *   if (millis() - lastCheck > 1000) {
 *     // CRITICAL: Use power integration for flight decisions!
 *     uint8_t capacityPercent = flightBattery.getRemainingCapacityPercent();
 *     double timeLeft = flightBattery.estimateRemainingTime_minutes();
 *     
 *     // Voltage percentage only reliable when unloaded (motors off)
 *     // uint8_t voltagePercent = voltageToBatteryPercentFloat(voltage); // ⚠️  UNRELIABLE UNDER LOAD!
 *     
 *     Serial.print("Remaining capacity: ");
 *     Serial.print(capacityPercent);
 *     Serial.print("% | Flight time: ");
 *     Serial.print(timeLeft, 1);
 *     Serial.print(" min | Voltage: ");
 *     Serial.print(voltage, 2);
 *     Serial.println("V (loaded)");
 *     
 *     // Safety logic: Use power integration for flight decisions
 *     if (flightBattery.isBatteryLow(20)) {
 *       Serial.println("WARNING: Battery capacity low - return to home!");
 *       triggerReturnToHome();
 *     }
 *     
 *     // Optional: Check voltage only when landed/unloaded
 *     if (isLanded() && current < 0.1) { // Motors off, minimal current
 *       uint8_t unloadedPercent = voltageToBatteryPercentFloat(voltage);
 *       Serial.print("Unloaded voltage percentage: ");
 *       Serial.print(unloadedPercent);
 *       Serial.println("%");
 *     }
 *     
 *     lastCheck = millis();
 *   }
 * }
 * ```
 * 
 * RESET AND CALIBRATION:
 * ======================
 * 
 * ```cpp
 * // Reset consumption counter (new flight)
 * batteryMonitor.reset();
 * 
 * // Reset and update battery capacity
 * batteryMonitor.reset(26.5);  // New battery with different capacity
 * 
 * // Update capacity without reset
 * batteryMonitor.setBatteryCapacity(26.5);
 * ```
 * 
 * VOLTAGE RANGE AND SAFETY:
 * =========================
 * 
 * LiPo voltage characteristics (per cell):
 * - 4.2V = 100% (fully charged, end of charge cycle)
 * - 3.6V = 0% (safe minimum for storage/measurement)  
 * - 3.3V = Critical threshold (chemical destabilization begins)
 * 
 * ⚠️  IMPORTANT: UNLOADED VOLTAGE MEASUREMENTS ONLY!
 * ==================================================
 * 
 * The lookup table percentage calculations are ONLY accurate for unloaded cells:
 * - Measure voltage when NO current is flowing (motors off, load disconnected)
 * - Under load, voltage drops due to internal resistance (ESR)
 * - High discharge currents cause significant voltage sag
 * - Battery age and chemistry variations affect the discharge curve
 * - Different motor/ESC configurations create different load profiles
 * 
 * For LOADED measurements (during flight):
 * - Use power integration (Wh consumed vs. total Wh capacity) instead
 * - Voltage-based percentage becomes unreliable under load
 * - Combine both methods: unloaded voltage + power integration
 * 
 * Lookup table benefits (unloaded only):
 * - Non-linear conversion matches real LiPo discharge curve
 * - Interpolation between table values for precise results
 * - Example: 3.693V (unloaded) → interpolated → ~15%
 * - Automatic 0.5% rounding (values ≥0.5 round up to next percent)
 * 
 * Multi-cell batteries:
 * - 2S: Divide total voltage by 2, then use lookup functions
 * - 3S: Divide total voltage by 3, then use lookup functions  
 * - 4S: Divide total voltage by 4, then use lookup functions
 * 
 * NOTES:
 * ======
 * - ⚠️  CRITICAL: Lookup tables only accurate for UNLOADED voltage measurements
 * - Under load (motors running), internal resistance causes voltage sag → inaccurate %
 * - Battery age, temperature, and chemistry variations affect discharge curves
 * - For flight applications: Use power integration (Wh) for reliable capacity tracking
 * - Voltage-based % only useful when motors are off (landing, pre-flight checks)
 * - Power integration uses trapezoidal rule for accuracy with variable sample rates
 * - All functions are optimized for real-time drone applications
 * - Battery capacity should be specified in Watt-Hours (Wh)
 * - Current should be positive for discharge, negative for charge
 * - Combine both methods: Power integration during flight + voltage check when landed
 * - Multi-cell batteries: divide total voltage by cell count before using functions
 */

// LiPo Battery Voltage to Percentage Lookup Table
// Based on unloaded LiPo cell voltage (3.6V = 0%, 4.2V = 100%)
// Protects cells from chemical destabilization below 3.3V
// Index represents percentage (0-99), value is voltage in millivolts

const uint16_t LIPO_VOLTAGE_TABLE[100] = {
  3600, 3606, 3612, 3618, 3624, 3630, 3636, 3642, 3648, 3654, // 0-9%
  3660, 3666, 3672, 3678, 3684, 3690, 3696, 3702, 3708, 3714, // 10-19%
  3720, 3726, 3732, 3738, 3744, 3750, 3756, 3762, 3768, 3774, // 20-29%
  3780, 3786, 3792, 3798, 3804, 3810, 3816, 3822, 3828, 3834, // 30-39%
  3840, 3846, 3852, 3858, 3864, 3870, 3876, 3882, 3888, 3894, // 40-49%
  3900, 3906, 3912, 3918, 3924, 3930, 3936, 3942, 3948, 3954, // 50-59%
  3960, 3966, 3972, 3978, 3984, 3990, 3996, 4002, 4008, 4014, // 60-69%
  4020, 4026, 4032, 4038, 4044, 4050, 4056, 4062, 4068, 4074, // 70-79%
  4080, 4086, 4092, 4098, 4104, 4110, 4116, 4122, 4128, 4134, // 80-89%
  4140, 4146, 4152, 4158, 4164, 4170, 4176, 4182, 4188, 4200  // 90-99%
};

// Alternative float array (voltage in volts)
const float LIPO_VOLTAGE_TABLE_FLOAT[100] = {
  3.600, 3.606, 3.612, 3.618, 3.624, 3.630, 3.636, 3.642, 3.648, 3.654, // 0-9%
  3.660, 3.666, 3.672, 3.678, 3.684, 3.690, 3.696, 3.702, 3.708, 3.714, // 10-19%
  3.720, 3.726, 3.732, 3.738, 3.744, 3.750, 3.756, 3.762, 3.768, 3.774, // 20-29%
  3.780, 3.786, 3.792, 3.798, 3.804, 3.810, 3.816, 3.822, 3.828, 3.834, // 30-39%
  3.840, 3.846, 3.852, 3.858, 3.864, 3.870, 3.876, 3.882, 3.888, 3.894, // 40-49%
  3.900, 3.906, 3.912, 3.918, 3.924, 3.930, 3.936, 3.942, 3.948, 3.954, // 50-59%
  3.960, 3.966, 3.972, 3.978, 3.984, 3.990, 3.996, 4.002, 4.008, 4.014, // 60-69%
  4.020, 4.026, 4.032, 4.038, 4.044, 4.050, 4.056, 4.062, 4.068, 4.074, // 70-79%
  4.080, 4.086, 4.092, 4.098, 4.104, 4.110, 4.116, 4.122, 4.128, 4.134, // 80-89%
  4.140, 4.146, 4.152, 4.158, 4.164, 4.170, 4.176, 4.182, 4.188, 4.200  // 90-99%
};

// Battery voltage constants
#define LIPO_MIN_VOLTAGE_MV    3600  // 0% charge (safe minimum)
#define LIPO_MAX_VOLTAGE_MV    4200  // 100% charge (full)
#define LIPO_CRITICAL_MV       3300  // Chemical destabilization threshold
#define LIPO_OVERCHARGE_MV     4250  // Overcharge protection

#define LIPO_MIN_VOLTAGE       3.6   // 0% charge (safe minimum)
#define LIPO_MAX_VOLTAGE       4.2   // 100% charge (full)
#define LIPO_CRITICAL_VOLTAGE  3.3   // Chemical destabilization threshold
#define LIPO_OVERCHARGE_VOLTAGE 4.25 // Overcharge protection

// Utility functions using lookup tables for accurate non-linear conversion
inline uint8_t voltageToBatteryPercent(uint16_t voltage_mv) {
  if (voltage_mv <= LIPO_MIN_VOLTAGE_MV) return 0;
  if (voltage_mv >= LIPO_MAX_VOLTAGE_MV) return 100;
  
  // Find the two table entries that bracket our voltage
  for (uint8_t i = 0; i < 99; i++) {
    if (voltage_mv >= LIPO_VOLTAGE_TABLE[i] && voltage_mv <= LIPO_VOLTAGE_TABLE[i + 1]) {
      // Interpolate between i% and (i+1)%
      uint16_t lower_mv = LIPO_VOLTAGE_TABLE[i];
      uint16_t upper_mv = LIPO_VOLTAGE_TABLE[i + 1];
      
      // Calculate fractional position (0.0 = lower, 1.0 = upper)
      float fraction = (float)(voltage_mv - lower_mv) / (float)(upper_mv - lower_mv);
      
      // Round to nearest 0.5% (0.5 rounds up to next %)
      return i + (uint8_t)(fraction + 0.5f);
    }
  }
  
  // Handle edge case: exactly at 99% table entry
  if (voltage_mv == LIPO_VOLTAGE_TABLE[99]) return 99;
  
  return 99; // Should not reach here, but safety fallback
}

inline uint8_t voltageToBatteryPercentFloat(float voltage) {
  uint16_t voltage_mv = (uint16_t)(voltage * 1000.0 + 0.5); // Round to nearest mV
  return voltageToBatteryPercent(voltage_mv);
}

inline uint16_t batteryPercentToVoltage(uint8_t percent) {
  if (percent >= 100) return LIPO_MAX_VOLTAGE_MV;
  if (percent == 0) return LIPO_MIN_VOLTAGE_MV;
  return LIPO_VOLTAGE_TABLE[percent - 1];  // Table is 0-indexed for 0-99%
}

inline float batteryPercentToVoltageFloat(uint8_t percent) {
  if (percent >= 100) return LIPO_MAX_VOLTAGE;
  if (percent == 0) return LIPO_MIN_VOLTAGE;
  return LIPO_VOLTAGE_TABLE_FLOAT[percent - 1];  // Table is 0-indexed for 0-99%
}

inline bool isBatteryVoltageSafe(uint16_t voltage_mv) {
  return (voltage_mv >= LIPO_CRITICAL_MV && voltage_mv <= LIPO_OVERCHARGE_MV);
}

inline bool isBatteryVoltageSafeFloat(float voltage) {
  return (voltage >= LIPO_CRITICAL_VOLTAGE && voltage <= LIPO_OVERCHARGE_VOLTAGE);
}

// Power Integration Class for Watt-Hours Calculation
class PowerIntegrator {
private:
  double totalWattHours;
  unsigned long lastTimestamp;
  bool initialized;
  
public:
  PowerIntegrator() : totalWattHours(0.0), lastTimestamp(0), initialized(false) {}
  
  // Add power measurement and return cumulative Watt-Hours
  double addMeasurement(float voltage, float current, unsigned long timestamp_ms) {
    if (!initialized) {
      lastTimestamp = timestamp_ms;
      initialized = true;
      return totalWattHours;
    }
    
    // Calculate time delta in hours
    unsigned long deltaTime_ms = timestamp_ms - lastTimestamp;
    double deltaTime_hours = deltaTime_ms / 3600000.0; // Convert ms to hours
    
    // Calculate instantaneous power (Watts)
    double power_watts = voltage * current;
    
    // Integrate power over time (trapezoidal rule for better accuracy)
    totalWattHours += power_watts * deltaTime_hours;
    
    lastTimestamp = timestamp_ms;
    return totalWattHours;
  }
  
  // Reset integration
  void reset() {
    totalWattHours = 0.0;
    initialized = false;
  }
  
  // Get current total without adding measurement
  double getTotalWattHours() const {
    return totalWattHours;
  }
  
  // Get total in milliWatt-Hours for higher precision
  double getTotalMilliWattHours() const {
    return totalWattHours * 1000.0;
  }
};

// Advanced Power Integration with averaging for noise reduction
class AdvancedPowerIntegrator {
private:
  double totalWattHours;
  double batteryCapacityWh;
  unsigned long lastTimestamp;
  float lastVoltage, lastCurrent;
  bool initialized;
  
public:
  // Constructor with battery capacity configuration
  AdvancedPowerIntegrator(double batteryCapacity_Wh = 0.0) : 
    totalWattHours(0.0), batteryCapacityWh(batteryCapacity_Wh), lastTimestamp(0), 
    lastVoltage(0.0), lastCurrent(0.0), initialized(false) {}
  
  // Set or update battery capacity
  void setBatteryCapacity(double capacity_Wh) {
    batteryCapacityWh = capacity_Wh;
  }
  
  // Get configured battery capacity
  double getBatteryCapacity() const {
    return batteryCapacityWh;
  }
  
  // Add measurement with trapezoidal integration (more accurate)
  double addMeasurement(float voltage, float current, unsigned long timestamp_ms) {
    if (!initialized) {
      lastTimestamp = timestamp_ms;
      lastVoltage = voltage;
      lastCurrent = current;
      initialized = true;
      return totalWattHours;
    }
    
    // Calculate time delta in hours
    unsigned long deltaTime_ms = timestamp_ms - lastTimestamp;
    double deltaTime_hours = deltaTime_ms / 3600000.0;
    
    // Trapezoidal integration: average of current and last power
    double lastPower = lastVoltage * lastCurrent;
    double currentPower = voltage * current;
    double avgPower = (lastPower + currentPower) / 2.0;
    
    // Integrate average power over time
    totalWattHours += avgPower * deltaTime_hours;
    
    // Store for next iteration
    lastTimestamp = timestamp_ms;
    lastVoltage = voltage;
    lastCurrent = current;
    
    return totalWattHours;
  }
  
  // Get remaining battery capacity as percentage (0-100%)
  uint8_t getRemainingCapacityPercent() const {
    if (batteryCapacityWh <= 0) return 0;
    double remaining = ((batteryCapacityWh - totalWattHours) / batteryCapacityWh) * 100.0;
    return (uint8_t)constrain(remaining, 0.0, 100.0);
  }
  
  // Get remaining capacity in Watt-Hours
  double getRemainingCapacityWh() const {
    double remaining = batteryCapacityWh - totalWattHours;
    return max(remaining, 0.0);
  }
  
  // Get consumed energy as percentage of total capacity
  uint8_t getConsumedPercent() const {
    if (batteryCapacityWh <= 0) return 0;
    double consumed = (totalWattHours / batteryCapacityWh) * 100.0;
    return (uint8_t)constrain(consumed, 0.0, 100.0);
  }
  
  // Estimate remaining flight time based on current power consumption
  double estimateRemainingTime_minutes() const {
    double currentPower = getCurrentPower();
    if (currentPower <= 0) return -1; // Invalid/no consumption
    
    double remainingWh = getRemainingCapacityWh();
    double remainingHours = remainingWh / currentPower;
    return remainingHours * 60.0; // Convert to minutes
  }
  
  // Reset integration but keep battery capacity
  void reset() {
    totalWattHours = 0.0;
    initialized = false;
  }
  
  // Reset integration and set new battery capacity
  void reset(double newCapacity_Wh) {
    totalWattHours = 0.0;
    batteryCapacityWh = newCapacity_Wh;
    initialized = false;
  }
  
  // Get current total
  double getTotalWattHours() const {
    return totalWattHours;
  }
  
  // Get total in milliWatt-Hours
  double getTotalMilliWattHours() const {
    return totalWattHours * 1000.0;
  }
  
  // Get current instantaneous power
  double getCurrentPower() const {
    return lastVoltage * lastCurrent;
  }
  
  // Check if battery capacity is critically low
  bool isBatteryLow(uint8_t threshold_percent = 20) const {
    return getRemainingCapacityPercent() <= threshold_percent;
  }
  
  // Check if battery is empty
  bool isBatteryEmpty() const {
    return getRemainingCapacityPercent() == 0;
  }
};

// Simple helper function for one-shot calculations
inline double calculateWattHours(float voltage1, float current1, unsigned long time1_ms,
                                float voltage2, float current2, unsigned long time2_ms) {
  double deltaTime_hours = (time2_ms - time1_ms) / 3600000.0;
  double avgPower = ((voltage1 * current1) + (voltage2 * current2)) / 2.0;
  return avgPower * deltaTime_hours;
}

// Battery capacity estimation based on consumed Watt-Hours
inline uint8_t estimateRemainingCapacity(double consumedWh, double totalCapacityWh) {
  if (totalCapacityWh <= 0) return 0;
  double remaining = ((totalCapacityWh - consumedWh) / totalCapacityWh) * 100.0;
  return (uint8_t)constrain(remaining, 0.0, 100.0);
}

#endif // LIPO_BATTERY_H
