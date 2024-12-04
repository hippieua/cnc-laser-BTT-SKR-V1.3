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

#define CONFIGURATION_H_VERSION 02010205

#define STRING_CONFIG_H_AUTHOR "(hippie, skr 1.3)" // Who made the changes.
#define SHOW_BOOTSCREEN

#define MOTHERBOARD BOARD_BTT_SKR_V1_3

#define SERIAL_PORT -1
#define BAUDRATE 115200

#define X_DRIVER_TYPE  TMC2209
#define Y_DRIVER_TYPE  TMC2209
#define Z_DRIVER_TYPE  A4988

#define EXTRUDERS 0

#define COREXY

// @section endstops

#define USE_XMIN_PLUG
#define USE_YMIN_PLUG
#define USE_ZMIN_PLUG


// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MIN_PROBE_ENDSTOP_INVERTING false // Set to true to invert the logic of the probe.

#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400 }
#define DEFAULT_MAX_FEEDRATE          { 900, 900, 5 }

#define DEFAULT_MAX_ACCELERATION      { 1500, 1500, 100 }

#define DEFAULT_ACCELERATION          1500    // X, Y, Z and E acceleration for printing moves
#define DEFAULT_RETRACT_ACCELERATION  1500    // E acceleration for retracts
#define DEFAULT_TRAVEL_ACCELERATION   1500    // X, Y, Z acceleration for travel (non printing) moves


#define DEFAULT_EJERK    5.0  // May be used by Linear Advance
#define JUNCTION_DEVIATION_MM 0.013 // (mm) Distance from real junction edge
#define JD_HANDLE_SMALL_SEGMENTS    // Use curvature estimation instead of just the junction angle

#define NOZZLE_TO_PROBE_OFFSET { 10, 10, 0 }
#define PROBING_MARGIN 10
#define XY_PROBE_FEEDRATE (133*60)

// @section stepper drivers

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{ 0:'Low', 1:'High' }
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0

#define DISABLE_OTHER_EXTRUDERS   // Keep only the active extruder enabled

#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false


// @section homing
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1


// @section geometry
#define X_BED_SIZE 235
#define Y_BED_SIZE 250

// Travel limits (linear=mm, rotational=°) after homing, corresponding to endstop positions.
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 200


#define MIN_SOFTWARE_ENDSTOPS
#define MIN_SOFTWARE_ENDSTOP_X
#define MIN_SOFTWARE_ENDSTOP_Y
#define MAX_SOFTWARE_ENDSTOPS
#define MAX_SOFTWARE_ENDSTOP_X
#define MAX_SOFTWARE_ENDSTOP_Y


#define HOMING_FEEDRATE_MM_M { (20*60), (20*60), (4*60) }
#define VALIDATE_HOMING_ENDSTOPS

#define EEPROM_CHITCHAT       // Give feedback on EEPROM commands. Disable to save flash.
#define EEPROM_BOOT_SILENT    // Keep M503 quiet and only give errors during first load

//#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages
//#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.

#define SOFT_PWM_SCALE 0
