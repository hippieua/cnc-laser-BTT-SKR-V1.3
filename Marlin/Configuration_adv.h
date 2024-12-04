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


#define SENSORLESS_BACKOFF_MM  { 5, 5, 0 }    // (linear=mm, rotational=°) Backoff from endstops before sensorless homing
#define HOMING_BUMP_MM      { 0, 0, 0 }       // (linear=mm, rotational=°) Backoff from endstops after first bump
#define HOMING_BUMP_DIVISOR { 2, 2, 4 }       // Re-Bump Speed Divisor (Divides the Homing Feedrate)

#define HOMING_BACKOFF_POST_MM { 10, 10, 2 }  // (linear=mm, rotational=°) Backoff from endstops after homing

#define AXIS_RELATIVE_MODES { false, false, false }

#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false

#define DEFAULT_STEPPER_TIMEOUT_SEC 120
#define DISABLE_IDLE_X
#define DISABLE_IDLE_Y
#define DISABLE_IDLE_Z    // Disable if the nozzle could fall onto your printed part!

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


#define ARC_SUPPORT                   // Requires ~3226 bytes
#if ENABLED(ARC_SUPPORT)
  #define MIN_ARC_SEGMENT_MM      0.1 // (mm) Minimum length of each arc segment
  #define MAX_ARC_SEGMENT_MM      1.0 // (mm) Maximum length of each arc segment
  #define MIN_CIRCLE_SEGMENTS    72   // Minimum number of segments in a complete circle
  #define N_ARC_CORRECTION       25   // Number of interpolated segments between corrections
#endif


#define MIN_STEPS_PER_SEGMENT 6

#define MINIMUM_STEPPER_POST_DIR_DELAY 20
#define MINIMUM_STEPPER_PRE_DIR_DELAY 20
#define MINIMUM_STEPPER_PULSE 0
#define MAXIMUM_STEPPER_RATE 5000000

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
    #define X_CURRENT_HOME  800         // (mA) RMS current for homing. (Typically lower than *_CURRENT.)
    #define X_MICROSTEPS    16          // 0..256
    #define X_RSENSE        0.11        // Multiplied x1000 for TMC26X
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
#define SPINDLE_LASER_ACTIVE_STATE   HIGH     // Set to "HIGH" if SPINDLE_LASER_ENA_PIN is active HIGH
#define SPINDLE_LASER_PWM_INVERT    true      // Set to "true" if the speed/power goes up when you want it to go slower
#define SPINDLE_LASER_FREQUENCY     2000      // (Hz) Spindle/laser frequency (only on supported HALs: AVR, ESP32, and LPC)


#define AIR_EVACUATION                     // Cutter Vacuum / Laser Blower motor control with G-codes M10-M11
#define AIR_EVACUATION_ACTIVE       LOW    // Set to "HIGH" if the on/off function is active HIGH
#define AIR_EVACUATION_PIN        P2_05    // Override the default Cutter Vacuum or Laser Blower pin

#define AIR_ASSIST                         // Air Assist control with G-codes M8-M9
#define AIR_ASSIST_ACTIVE           LOW    // Active state on air assist pin
#define AIR_ASSIST_PIN            P2_03     // Override the default Air Assist pin

#define CUTTER_POWER_UNIT PWM255


#define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage
#define SPEED_POWER_MIN             0    // (%) 0-100
#define SPEED_POWER_MAX           100    // (%) 0-100
#define SPEED_POWER_STARTUP        80    // (%) M3/M4 speed/power default (with no arguments)

// Define the minimum and maximum test pulse time values for a laser test fire function
#define LASER_TEST_PULSE_MIN           1   // (ms) Used with Laser Control Menu
#define LASER_TEST_PULSE_MAX         999   // (ms) Caution: Menu may not show more than 3 characters

#define SPINDLE_LASER_POWERUP_DELAY   50   // (ms) Delay to allow the spindle/laser to come up to speed/power
#define SPINDLE_LASER_POWERDOWN_DELAY 50   // (ms) Delay to allow the spindle to stop


//#define LASER_SAFETY_TIMEOUT_MS     1000   // (ms)

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



#define EXTENDED_CAPABILITIES_REPORT
#if ENABLED(EXTENDED_CAPABILITIES_REPORT)
  //#define M115_GEOMETRY_REPORT
#endif

#define FASTER_GCODE_PARSER
#define GCODE_MOTION_MODES  // Remember the motion mode (G0 G1 G2 G3 G5 G38.X) and apply for X Y Z E F, etc.

//#define STARTUP_COMMANDS "M17 Z"
//#define DIRECT_PIN_CONTROL