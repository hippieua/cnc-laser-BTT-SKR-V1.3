/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#define CONFIGURATION_ADV_H_VERSION 02010205

// @section develop





// @section homing

#define SENSORLESS_BACKOFF_MM  { 5, 5, 0 }  // (linear=mm, rotational=°) Backoff from endstops before sensorless homing
#define HOMING_BUMP_MM      { 0, 0, 0 }       // (linear=mm, rotational=°) Backoff from endstops after first bump
#define HOMING_BUMP_DIVISOR { 2, 2, 4 }       // Re-Bump Speed Divisor (Divides the Homing Feedrate)

#define HOMING_BACKOFF_POST_MM { 10, 10, 2 }  // (linear=mm, rotational=°) Backoff from endstops after homing
//#define XY_COUNTERPART_BACKOFF_MM 15         // (mm) Backoff X after homing Y, and vice-versa

//#define QUICK_HOME                          // If G28 contains XY do a diagonal move first
//#define HOME_Y_BEFORE_X                     // If G28 contains XY home Y before X
//#define HOME_Z_FIRST                        // Home Z first. Requires a real endstop (not a probe).
//#define CODEPENDENT_XY_HOMING               // If X/Y can't home without homing Y/X first


#define AXIS_RELATIVE_MODES { false, false, false }

// By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_I_STEP_PIN false
#define INVERT_J_STEP_PIN false
#define INVERT_K_STEP_PIN false
#define INVERT_U_STEP_PIN false
#define INVERT_V_STEP_PIN false
#define INVERT_W_STEP_PIN false
#define INVERT_E_STEP_PIN false

/**
 * Idle Stepper Shutdown
 * Enable DISABLE_IDLE_* to shut down axis steppers after an idle period.
 * The default timeout duration can be overridden with M18 and M84. Set to 0 for No Timeout.
 */
#define DEFAULT_STEPPER_TIMEOUT_SEC 120
#define DISABLE_IDLE_X
#define DISABLE_IDLE_Y
#define DISABLE_IDLE_Z    // Disable if the nozzle could fall onto your printed part!
#define DISABLE_IDLE_E    // Shut down all idle extruders

#define DEFAULT_MINIMUMFEEDRATE       0.0     // (mm/s) Minimum feedrate. Set with M205 S.
#define DEFAULT_MINTRAVELFEEDRATE     0.0     // (mm/s) Minimum travel feedrate. Set with M205 T.

#define DEFAULT_MINSEGMENTTIME        20000   // (µs) Set with M205 B.


// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/s)


// @section safety

#define USE_WATCHDOG
#if ENABLED(USE_WATCHDOG)
  //#define WATCHDOG_RESET_MANUAL
#endif


// @section extras

//
// G60/G61 Position Save and Return
//
//#define SAVED_POSITIONS 1         // Each saved position slot costs 12 bytes

//
// G2/G3 Arc Support
//
#define ARC_SUPPORT                   // Requires ~3226 bytes
#if ENABLED(ARC_SUPPORT)
  #define MIN_ARC_SEGMENT_MM      0.1 // (mm) Minimum length of each arc segment
  #define MAX_ARC_SEGMENT_MM      1.0 // (mm) Maximum length of each arc segment
  #define MIN_CIRCLE_SEGMENTS    72   // Minimum number of segments in a complete circle
  //#define ARC_SEGMENTS_PER_SEC 50   // Use the feedrate to choose the segment length
  #define N_ARC_CORRECTION       25   // Number of interpolated segments between corrections
  //#define ARC_P_CIRCLES             // Enable the 'P' parameter to specify complete circles
  //#define SF_ARC_FIX                // Enable only if using SkeinForge with "Arc Point" fillet procedure
#endif

// G5 Bézier Curve Support with XYZE destination and IJPQ offsets
//#define BEZIER_CURVE_SUPPORT        // Requires ~2666 bytes

#if EITHER(ARC_SUPPORT, BEZIER_CURVE_SUPPORT)
  //#define CNC_WORKSPACE_PLANES      // Allow G2/G3/G5 to operate in XY, ZX, or YZ planes
#endif



#define MIN_STEPS_PER_SEGMENT 6

#define MINIMUM_STEPPER_POST_DIR_DELAY 20
#define MINIMUM_STEPPER_PRE_DIR_DELAY 20
#define MINIMUM_STEPPER_PULSE 0
#define MAXIMUM_STEPPER_RATE 5000000

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section gcode

#define BLOCK_BUFFER_SIZE 16
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
#define TX_BUFFER_SIZE 0
#define SERIAL_OVERRUN_PROTECTION
#define PROPORTIONAL_FONT_RATIO 1.0

#if HAS_TRINAMIC_CONFIG || HAS_TMC26X
  #define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current
  #define INTERPOLATE      true

  #if AXIS_IS_TMC_CONFIG(X)
    #define X_CURRENT       1200        // (mA) RMS current. Multiply by 1.414 for peak current.
    #define X_CURRENT_HOME  800  // (mA) RMS current for homing. (Typically lower than *_CURRENT.)
    #define X_MICROSTEPS    16        // 0..256
    #define X_RSENSE        0.11     // Multiplied x1000 for TMC26X
    #define X_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Y)
    #define Y_CURRENT       1200
    #define Y_CURRENT_HOME  800
    #define Y_MICROSTEPS    16
    #define Y_RSENSE        0.11
    #define Y_CHAIN_POS     -1
  #endif


  #if HAS_STEALTHCHOP
    #define STEALTHCHOP_XY
  #endif

  #define CHOPPER_TIMING CHOPPER_DEFAULT_12V        // All axes (override below)
  #define MONITOR_DRIVER_STATUS
  #if ENABLED(MONITOR_DRIVER_STATUS)
    #define CURRENT_STEP_DOWN     50  // [mA]
    #define REPORT_CURRENT_CHANGE
    #define STOP_ON_ERROR
  #endif

  #define X_HYBRID_THRESHOLD     100  // [mm/s]
  #define Y_HYBRID_THRESHOLD     100

  #define SENSORLESS_HOMING // StallGuard capable drivers only

  #if EITHER(SENSORLESS_HOMING, SENSORLESS_PROBING)
    #define X_STALL_SENSITIVITY  35
    #define Y_STALL_SENSITIVITY  35
    #define IMPROVE_HOMING_RELIABILITY
  #endif

  #define SQUARE_WAVE_STEPPING
  #define TMC_DEBUG
  #define TMC_ADV() {  }

#endif // HAS_TRINAMIC_CONFIG || HAS_TMC26X

#define LASER_FEATURE
#define SPINDLE_LASER_USE_PWM                // Enable if your controller supports setting the speed/power
#define SPINDLE_LASER_PWM_PIN P2_07
#define SPINDLE_LASER_ENA_PIN P2_04
#define SPINDLE_LASER_ACTIVE_STATE   HIGH    // Set to "HIGH" if SPINDLE_LASER_ENA_PIN is active HIGH

#define SPINDLE_LASER_PWM_INVERT    true  // Set to "true" if the speed/power goes up when you want it to go slower
#define SPINDLE_LASER_FREQUENCY     2000   // (Hz) Spindle/laser frequency (only on supported HALs: AVR, ESP32, and LPC)
                                            // ESP32: If SPINDLE_LASER_PWM_PIN is onboard then <=78125Hz. For I2S expander
                                            //  the frequency determines the PWM resolution. 2500Hz = 0-100, 977Hz = 0-255, ...
                                            //  (250000 / SPINDLE_LASER_FREQUENCY) = max value.

//#define AIR_EVACUATION                     // Cutter Vacuum / Laser Blower motor control with G-codes M10-M11
#if ENABLED(AIR_EVACUATION)
  #define AIR_EVACUATION_ACTIVE       LOW    // Set to "HIGH" if the on/off function is active HIGH
  //#define AIR_EVACUATION_PIN        42     // Override the default Cutter Vacuum or Laser Blower pin
#endif

//#define AIR_ASSIST                         // Air Assist control with G-codes M8-M9
#if ENABLED(AIR_ASSIST)
  #define AIR_ASSIST_ACTIVE           LOW    // Active state on air assist pin
  //#define AIR_ASSIST_PIN            44     // Override the default Air Assist pin
#endif

//#define SPINDLE_SERVO                      // A servo converting an angle to spindle power
#ifdef SPINDLE_SERVO
  #define SPINDLE_SERVO_NR   0               // Index of servo used for spindle control
  #define SPINDLE_SERVO_MIN 10               // Minimum angle for servo spindle
#endif

  /**
   * Speed / Power can be set ('M3 S') and displayed in terms of:
   *  - PWM255  (S0 - S255)
   *  - PERCENT (S0 - S100)
   *  - RPM     (S0 - S50000)  Best for use with a spindle
   *  - SERVO   (S0 - S180)
   */
#define CUTTER_POWER_UNIT PWM255

/**
 * Relative Cutter Power
 * Normally, 'M3 O<power>' sets
 * OCR power is relative to the range SPEED_POWER_MIN...SPEED_POWER_MAX.
 * so input powers of 0...255 correspond to SPEED_POWER_MIN...SPEED_POWER_MAX
 * instead of normal range (0 to SPEED_POWER_MAX).
 * Best used with (e.g.) SuperPID router controller: S0 = 5,000 RPM and S255 = 30,000 RPM
 */
//#define CUTTER_POWER_RELATIVE              // Set speed proportional to [SPEED_POWER_MIN...SPEED_POWER_MAX]

#if ENABLED(SPINDLE_FEATURE)
  //#define SPINDLE_CHANGE_DIR               // Enable if your spindle controller can change spindle direction
  #define SPINDLE_CHANGE_DIR_STOP            // Enable if the spindle should stop before changing spin direction
  #define SPINDLE_INVERT_DIR          false  // Set to "true" if the spin direction is reversed

  #define SPINDLE_LASER_POWERUP_DELAY   5000 // (ms) Delay to allow the spindle/laser to come up to speed/power
  #define SPINDLE_LASER_POWERDOWN_DELAY 5000 // (ms) Delay to allow the spindle to stop

  /**
   * M3/M4 Power Equation
   *
   * Each tool uses different value ranges for speed / power control.
   * These parameters are used to convert between tool power units and PWM.
   *
   * Speed/Power = (PWMDC / 255 * 100 - SPEED_POWER_INTERCEPT) / SPEED_POWER_SLOPE
   * PWMDC = (spdpwr - SPEED_POWER_MIN) / (SPEED_POWER_MAX - SPEED_POWER_MIN) / SPEED_POWER_SLOPE
   */
  #if ENABLED(SPINDLE_LASER_USE_PWM)
    #define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage
    #define SPEED_POWER_MIN          5000    // (RPM)
    #define SPEED_POWER_MAX         30000    // (RPM) SuperPID router controller 0 - 30,000 RPM
    #define SPEED_POWER_STARTUP     25000    // (RPM) M3/M4 speed/power default (with no arguments)
  #endif

#else

  #if ENABLED(SPINDLE_LASER_USE_PWM)
    #define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage
    #define SPEED_POWER_MIN             0    // (%) 0-100
    #define SPEED_POWER_MAX           100    // (%) 0-100
    #define SPEED_POWER_STARTUP        80    // (%) M3/M4 speed/power default (with no arguments)
  #endif

  // Define the minimum and maximum test pulse time values for a laser test fire function
  #define LASER_TEST_PULSE_MIN           1   // (ms) Used with Laser Control Menu
  #define LASER_TEST_PULSE_MAX         999   // (ms) Caution: Menu may not show more than 3 characters

  #define SPINDLE_LASER_POWERUP_DELAY   50   // (ms) Delay to allow the spindle/laser to come up to speed/power
  #define SPINDLE_LASER_POWERDOWN_DELAY 50   // (ms) Delay to allow the spindle to stop


  //#define LASER_SAFETY_TIMEOUT_MS     1000   // (ms)

  /**
   * Any M3 or G1/2/3/5 command with the 'I' parameter enables continuous inline power mode.
   *
   * e.g., 'M3 I' enables continuous inline power which is processed by the planner.
   * Power is stored in move blocks and applied when blocks are processed by the Stepper ISR.
   *
   * 'M4 I' sets dynamic mode which uses the current feedrate to calculate a laser power OCR value.
   *
   * Any move in dynamic mode will use the current feedrate to calculate the laser power.
   * Feed rates are set by the F parameter of a move command e.g. G1 X0 Y10 F6000
   * Laser power would be calculated by bit shifting off 8 LSB's. In binary this is div 256.
   * The calculation gives us ocr values from 0 to 255, values over F65535 will be set as 255 .
   * More refined power control such as compensation for accel/decel will be addressed in future releases.
   *
   * M5 I clears inline mode and set power to 0, M5 sets the power output to 0 but leaves inline mode on.
   */

  /**
   * Enable M3 commands for laser mode inline power planner syncing.
   * This feature enables any M3 S-value to be injected into the block buffers while in
   * CUTTER_MODE_CONTINUOUS. The option allows M3 laser power to be committed without waiting
   * for a planner synchronization
   */
  //#define LASER_POWER_SYNC

  /**
   * Scale the laser's power in proportion to the movement rate.
   *
   * - Sets the entry power proportional to the entry speed over the nominal speed.
   * - Ramps the power up every N steps to approximate the speed trapezoid.
   * - Due to the limited power resolution this is only approximate.
   */
  //#define LASER_POWER_TRAP



  //
  // Laser Coolant Flow Meter
  //
  //#define LASER_COOLANT_FLOW_METER
  #if ENABLED(LASER_COOLANT_FLOW_METER)
    #define FLOWMETER_PIN         20  // Requires an external interrupt-enabled pin (e.g., RAMPS 2,3,18,19,20,21)
    #define FLOWMETER_PPL       5880  // (pulses/liter) Flow meter pulses-per-liter on the input pin
    #define FLOWMETER_INTERVAL  1000  // (ms) Flow rate calculation interval in milliseconds
    #define FLOWMETER_SAFETY          // Prevent running the laser without the minimum flow rate set below
    #if ENABLED(FLOWMETER_SAFETY)
      #define FLOWMETER_MIN_LITERS_PER_MINUTE 1.5 // (liters/min) Minimum flow required when enabled
    #endif
  #endif
#endif


/**
 * Synchronous Laser Control with M106/M107
 *`
 * Marlin normally applies M106/M107 fan speeds at a time "soon after" processing
 * a planner block. This is too inaccurate for a PWM/TTL laser attached to the fan
 * header (as with some add-on laser kits). Enable this option to set fan/laser
 * speeds with much more exact timing for improved print fidelity.
 *
 * NOTE: This option sacrifices some cooling fan speed options.
 */
//#define LASER_SYNCHRONOUS_M106_M107

/**
 * Coolant Control
 *
 * Add the M7, M8, and M9 commands to turn mist or flood coolant on and off.
 *
 * Note: COOLANT_MIST_PIN and/or COOLANT_FLOOD_PIN must also be defined.
 */
//#define COOLANT_CONTROL
#if ENABLED(COOLANT_CONTROL)
  #define COOLANT_MIST                // Enable if mist coolant is present
  #define COOLANT_FLOOD               // Enable if flood coolant is present
  #define COOLANT_MIST_INVERT  false  // Set "true" if the on/off function is reversed
  #define COOLANT_FLOOD_INVERT false  // Set "true" if the on/off function is reversed
#endif

// @section filament width


#define EXTENDED_CAPABILITIES_REPORT
#if ENABLED(EXTENDED_CAPABILITIES_REPORT)
  //#define M115_GEOMETRY_REPORT
#endif

#if DISABLED(NO_VOLUMETRICS)
  #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
    #define DEFAULT_VOLUMETRIC_EXTRUDER_LIMIT  0.00     // (mm^3/sec)
  #endif
#endif

#define FASTER_GCODE_PARSER
#if ENABLED(FASTER_GCODE_PARSER)
  //#define GCODE_QUOTED_STRINGS  // Support for quoted string parameters
#endif

#define GCODE_MOTION_MODES  // Remember the motion mode (G0 G1 G2 G3 G5 G38.X) and apply for X Y Z E F, etc.

//#define STARTUP_COMMANDS "M17 Z"

//#define DIRECT_PIN_CONTROL