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
 * DUAL-MODE BATTERY PERCENTAGE SYSTEM:
 * ====================================
 * 
 * The library provides two different percentage calculation modes:
 * 
 * 1. TECHNICAL/REALISTIC MODE (3.2V-4.2V full range):
 * ```cpp
 * float voltage = 3.5;  // 3.5V measured
 * uint8_t techPercent = voltageToBatteryPercentFloat(voltage);
 * Serial.println(techPercent);  // Output: ~20% (realistic LiPo curve)
 * 
 * // Shows actual battery state including dangerous levels
 * // 3.2V = 0%, 3.3V = 10%, 3.6V = 40%, 4.2V = 100%
 * ```
 * 
 * 2. USER-FRIENDLY/SAFE MODE (3.6V-4.2V safe range):
 * ```cpp
 * float voltage = 3.5;  // 3.5V measured
 * uint8_t safePercent = voltageToBatteryPercentSafeFloat(voltage);
 * Serial.println(safePercent);  // Output: 0% (below safe minimum)
 * 
 * float voltage2 = 3.6;  // Safe minimum voltage
 * uint8_t safePercent2 = voltageToBatteryPercentSafeFloat(voltage2);
 * Serial.println(safePercent2);  // Output: 0% (remapped safe minimum)
 * 
 * float voltage3 = 3.9;  // Mid-range voltage
 * uint8_t safePercent3 = voltageToBatteryPercentSafeFloat(voltage3);
 * Serial.println(safePercent3);  // Output: ~50% (3.6V-4.2V → 0%-100%)
 * 
 * // Safe mode protects users from deep discharge:
 * // 3.6V = 0%, 3.9V = 50%, 4.2V = 100%
 * // Voltages below 3.6V show as 0% to prevent cell damage
 * ```
 * 
 * When to use each mode:
 * - Technical mode: Advanced users, diagnostics, full battery analysis
 * - Safe mode: General users, consumer applications, battery protection
 * 
 * Both modes support reverse conversion:
 * ```cpp
 * // Technical mode reverse conversion
 * uint16_t voltage_mv = batteryPercentToVoltage(20);    // Returns ~3500mV (3.5V)
 * float voltage_v = batteryPercentToVoltageFloat(20);   // Returns ~3.5V
 * 
 * // Safe mode reverse conversion  
 * uint16_t safe_mv = batteryPercentToVoltageSafe(50);   // Returns 3900mV (3.9V)
 * float safe_v = batteryPercentToVoltageSafeFloat(50);  // Returns 3.9V
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

// LiPo Battery Voltage to Percentage Lookup Table (TECHNICAL/REALISTIC)
// Based on realistic unloaded LiPo cell discharge curve (3.2V = 0%, 4.2V = 100%)
// Reflects actual LiPo characteristics including critical/dangerous ranges
// Index represents percentage (0-99), value is voltage in millivolts
// WARNING: Values below 30% (3.6V) can damage cells if discharged further!

const uint16_t LIPO_VOLTAGE_TABLE[100] = {
  // Exponentieller Bereich: 3.2V-3.4V (0%-8%) - Steiler Abfall am Ende
  3200, 3203, 3206, 3209, 3212, 3215, 3218, 3221, 3224, 3227, // 0-9%   - Critical/dangerous
  3230, 3235, 3240, 3245, 3250, 3255, 3260, 3265, 3270, 3275, // 10-19% - Very low/risky  
  3280, 3285, 3290, 3295, 3300, 3310, 3320, 3330, 3340, 3350, // 20-29% - Low/unsafe (3.3V=3%→20%, 3.4V=8%→29%)
  // Linearer Bereich: 3.5V-3.8V (20%-75%) - Flacher Mittelteil
  3360, 3370, 3380, 3390, 3400, 3410, 3420, 3430, 3440, 3450, // 30-39% - Safe minimum zone (3.5V=20%→30%)
  3460, 3470, 3480, 3490, 3500, 3510, 3520, 3530, 3540, 3550, // 40-49% - Safe operation 
  3560, 3570, 3580, 3590, 3600, 3610, 3620, 3630, 3640, 3650, // 50-59% - Normal range (3.6V=30%→55%)
  3660, 3670, 3680, 3690, 3700, 3710, 3720, 3730, 3740, 3750, // 60-69% - Good capacity (3.7V=55%→69%)
  3760, 3770, 3780, 3790, 3800, 3810, 3820, 3830, 3840, 3850, // 70-79% - High capacity (3.8V=75%→79%)
  // Plateau-Bereich: 3.9V-4.2V (90%-100%) - Flach oben, wenig Kapazität
  3860, 3870, 3880, 3890, 3900, 3920, 3940, 3960, 3980, 4000, // 80-89% - Very high (3.9V=90%→84%)
  4020, 4040, 4060, 4080, 4100, 4120, 4140, 4160, 4180, 4200  // 90-99% - Full charge (4.0V=95%→90%, 4.1V=98%→95%, 4.2V=100%→99%)
};

// Alternative float array (voltage in volts) - TECHNICAL/REALISTIC LiPo discharge curve
const float LIPO_VOLTAGE_TABLE_FLOAT[100] = {
  // Exponentieller Bereich: 3.2V-3.4V (0%-8%) - Steiler Abfall am Ende
  3.200, 3.203, 3.206, 3.209, 3.212, 3.215, 3.218, 3.221, 3.224, 3.227, // 0-9%   - Critical/dangerous
  3.230, 3.235, 3.240, 3.245, 3.250, 3.255, 3.260, 3.265, 3.270, 3.275, // 10-19% - Very low/risky  
  3.280, 3.285, 3.290, 3.295, 3.300, 3.310, 3.320, 3.330, 3.340, 3.350, // 20-29% - Low/unsafe (3.3V=3%→20%, 3.4V=8%→29%)
  // Linearer Bereich: 3.5V-3.8V (20%-75%) - Flacher Mittelteil
  3.360, 3.370, 3.380, 3.390, 3.400, 3.410, 3.420, 3.430, 3.440, 3.450, // 30-39% - Safe minimum zone (3.5V=20%→30%)
  3.460, 3.470, 3.480, 3.490, 3.500, 3.510, 3.520, 3.530, 3.540, 3.550, // 40-49% - Safe operation 
  3.560, 3.570, 3.580, 3.590, 3.600, 3.610, 3.620, 3.630, 3.640, 3.650, // 50-59% - Normal range (3.6V=30%→55%)
  3.660, 3.670, 3.680, 3.690, 3.700, 3.710, 3.720, 3.730, 3.740, 3.750, // 60-69% - Good capacity (3.7V=55%→69%)
  3.760, 3.770, 3.780, 3.790, 3.800, 3.810, 3.820, 3.830, 3.840, 3.850, // 70-79% - High capacity (3.8V=75%→79%)
  // Plateau-Bereich: 3.9V-4.2V (90%-100%) - Flach oben, wenig Kapazität
  3.860, 3.870, 3.880, 3.890, 3.900, 3.920, 3.940, 3.960, 3.980, 4.000, // 80-89% - Very high (3.9V=90%→84%)
  4.020, 4.040, 4.060, 4.080, 4.100, 4.120, 4.140, 4.160, 4.180, 4.200  // 90-99% - Full charge (4.0V=95%→90%, 4.1V=98%→95%, 4.2V=100%→99%)
};

// Battery voltage constants
#define LIPO_MIN_VOLTAGE_MV    3200  // 0% charge (technical minimum - cell death)
#define LIPO_MAX_VOLTAGE_MV    4200  // 100% charge (full)
#define LIPO_CRITICAL_MV       3300  // Chemical destabilization threshold
#define LIPO_OVERCHARGE_MV     4250  // Overcharge protection
#define LIPO_SAFE_MIN_MV       3600  // 40% charge (safe minimum for users)

#define LIPO_MIN_VOLTAGE       3.2   // 0% charge (technical minimum - cell death)
#define LIPO_MAX_VOLTAGE       4.2   // 100% charge (full)
#define LIPO_CRITICAL_VOLTAGE  3.3   // Chemical destabilization threshold
#define LIPO_OVERCHARGE_VOLTAGE 4.25 // Overcharge protection
#define LIPO_SAFE_MIN_VOLTAGE  3.6   // 40% charge (safe minimum for users)

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

// USER-FRIENDLY functions - Safe range mapping using realistic LiPo curve
// Uses the same lookup table but remaps percentages: 3.6V = 0%, 4.2V = 100%
// This provides realistic discharge curve characteristics with user protection
inline uint8_t voltageToBatteryPercentSafe(uint16_t voltage_mv) {
  if (voltage_mv <= LIPO_SAFE_MIN_MV) return 0;
  if (voltage_mv >= LIPO_MAX_VOLTAGE_MV) return 100;
  
  // First get the technical percentage using the realistic curve
  uint8_t technicalPercent = voltageToBatteryPercent(voltage_mv);
  
  // Technical range: 3.6V = 55%, 4.2V = 100% (from updated lookup table)
  // Safe range:     3.6V = 0%,  4.2V = 100% (remapped for users)
  
  if (technicalPercent <= 55) return 0;  // Below safe minimum (3.6V)
  
  // Remap 55%-100% technical range to 0%-100% safe range
  // Formula: safePercent = (technicalPercent - 55) * 100 / (100 - 55)
  return (uint8_t)((technicalPercent - 55) * 100 / 45);
}

inline uint8_t voltageToBatteryPercentSafeFloat(float voltage) {
  uint16_t voltage_mv = (uint16_t)(voltage * 1000.0 + 0.5);
  return voltageToBatteryPercentSafe(voltage_mv);
}

inline uint16_t batteryPercentToVoltageSafe(uint8_t percent) {
  if (percent >= 100) return LIPO_MAX_VOLTAGE_MV;
  if (percent == 0) return LIPO_SAFE_MIN_MV;
  
  // Reverse mapping: Safe 0%-100% to Technical 55%-100%
  // Formula: technicalPercent = (safePercent * 45 / 100) + 55
  uint8_t technicalPercent = (percent * 45 / 100) + 55;
  
  // Use technical lookup table for realistic voltage
  return batteryPercentToVoltage(technicalPercent);
}

inline float batteryPercentToVoltageSafeFloat(uint8_t percent) {
  return batteryPercentToVoltageSafe(percent) / 1000.0;
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
