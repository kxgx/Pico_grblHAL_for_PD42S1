/*
  pico_pd42s1_map.h - driver code for RP2040 ARM processors
  Custom board map for Pi Pico2 W with custom pin assignments

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io
  Copyright (c) 2021 Volksolive

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// Disable I2C
#undef I2C_ENABLE
#define I2C_ENABLE            0

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

// Currently configured for 4-axis (X, Y, Z, A)
// A axis is ganged to Y axis (Y dual motor configuration)
#if N_AXIS == 4
#define M3_AVAILABLE
#define M3_STEP_PIN             4         // GP4 - A axis stepper pulse
#define M3_DIRECTION_PIN        3         // GP3 - A axis stepper direction
#define M3_LIMIT_PIN            2         // GP2 - A axis limit switch
#endif

#if I2C_STROBE_ENABLE && MOTOR_FAULT_ENABLE
#error "Motor fault input and I2C strobe input (keypad plugin) cannot be enabled at the same time."
#endif

// Define step pulse output pins.
// Using GPIO_OUTPUT mode - Non-consecutive pins: GP19/GP8/GP12/GP4
// X=GP19, Y=GP8, Z=GP12, A=GP4
#define STEP_PORT               GPIO_OUTPUT
#define X_STEP_PIN              19        // GP19
#define Y_STEP_PIN              8         // GP8
#define Z_STEP_PIN              12        // GP12
#define A_STEP_PIN              4         // GP4

// Define step direction output pins.
// X=GP20, Y=GP9, Z=GP11, A=GP3
#define DIRECTION_PORT          GPIO_OUTPUT
#define X_DIRECTION_PIN         20        // GP20
#define Y_DIRECTION_PIN         9         // GP9
#define Z_DIRECTION_PIN         11        // GP11
#define A_DIRECTION_PIN         3         // GP3
#define DIRECTION_OUTMODE       GPIO_MAP     // Use map mode for non-consecutive pins

// Define stepper driver enable/disable output pin.
// X=GP18, Y=GP7, Z=GP13, A=GP5
#define ENABLE_PORT             GPIO_OUTPUT
#define X_ENABLE_PIN            18        // GP18
#define X_ENABLE_PORT           GPIO_OUTPUT
#define Y_ENABLE_PIN            7         // GP7
#define Y_ENABLE_PORT           GPIO_OUTPUT
#define Z_ENABLE_PIN            13        // GP13
#define Z_ENABLE_PORT           GPIO_OUTPUT
#define A_ENABLE_PIN            5         // GP5
#define A_ENABLE_PORT           GPIO_OUTPUT
#define STEPPERS_ENABLE_PIN     8         // Common enable pin (if needed)
#define ENABLE_ACTIVE_LOW       Off

// Define homing/hard limit switch input pins.
// X=GP21, Y=GP14, Z=GP10, A=GP2
#define LIMIT_PORT              GPIO_INPUT
#define X_LIMIT_PIN             21        // GP21
#define Y_LIMIT_PIN             14        // GP14
#define Z_LIMIT_PIN             10        // GP10
#define A_LIMIT_PIN             2         // GP2
#define LIMIT_INMODE            GPIO_MAP

// Auxiliary outputs
// GP1 is assigned to Laser PWM
#if I2C_ENABLE
#define I2C_PORT                1
#define I2C_SDA                 26        // GP26/ADC0
#define I2C_SCL                 27        // GP27/ADC1
#else
// #define AUXOUTPUT1_PORT      GPIO_OUTPUT
// #define AUXOUTPUT1_PIN       26
// #define AUXOUTPUT2_PORT      GPIO_OUTPUT
// #define AUXOUTPUT2_PIN       27
#endif

// Laser PWM output on GP1
#define AUXOUTPUT0_PORT         GPIO_OUTPUT
#define AUXOUTPUT0_PIN          1         // GP1 - Laser PWM

// Spindle outputs (not assigned in current configuration)
// Laser uses AUXOUTPUT0 (GP1) for PWM

// Define flood and mist coolant enable output pins.
// Note: Not used per pin definition file, define to valid pins to avoid compilation errors
#define COOLANT_FLOOD_PORT      GPIO_OUTPUT
#define COOLANT_FLOOD_PIN       0         // Dummy pin (not used)
#define COOLANT_MIST_PORT       GPIO_OUTPUT
#define COOLANT_MIST_PIN        0         // Dummy pin (not used)

// #define NEOPIXELS_PIN        27
// #define NEOPIXELS_NUM        5
//Define user-control controls (cycle start, reset, feed hold) input pins.

// Define driver spindle pins
// Laser PWM is on GP1 (AUXOUTPUT0)
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_PORT            GPIO_OUTPUT
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN  // GP1 - Laser PWM
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
// #define SPINDLE_DIRECTION_PIN   AUXOUTPUT4_PIN  // Not assigned
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA   
// #define SPINDLE_ENABLE_PIN      AUXOUTPUT5_PIN  // Not assigned
#endif

// Define flood and mist coolant enable output pins.
// Coolant not used in this configuration - pins already defined above

// Define auxiliary I/O inputs
// Only I2C is enabled, no other auxiliary inputs per pin definition file
// #define AUXINPUT0_PIN        22  // Not assigned
// #define AUXINPUT1_PIN        21  // Used by X_LIMIT
// #define AUXINPUT2_PIN        28  // GP28/ADC2 - Probe (DISABLED - not in pin definition)
// #define AUXINPUT3_PIN        16  // GP16 - Reset/EStop (DISABLED - not in pin definition)
// #define AUXINPUT4_PIN        17  // GP17 - Feed hold (DISABLED - not in pin definition)
// #define AUXINPUT5_PIN        6   // GP6 - Cycle start (DISABLED - not in pin definition)

// Define user-control controls (cycle start, reset, feed hold) input pins.
// DISABLED - Not in pin definition file
// #if CONTROL_ENABLE & CONTROL_HALT
// #define RESET_PIN            AUXINPUT3_PIN  // GP16
// #endif
// #if CONTROL_ENABLE & CONTROL_FEED_HOLD
// #define FEED_HOLD_PIN        AUXINPUT4_PIN  // GP17
// #endif
// #if CONTROL_ENABLE & CONTROL_CYCLE_START
// #define CYCLE_START_PIN      AUXINPUT5_PIN  // GP6
// #endif

// #if PROBE_ENABLE
// #define PROBE_PIN            AUXINPUT2_PIN  // GP28/ADC2
// #endif

// #define SAFETY_DOOR_PIN      AUXINPUT1_PIN  // Disabled: pin not assigned

// #define I2C_STROBE_PIN       AUXINPUT0_PIN  // Disabled: pin not assigned
// #elif MOTOR_FAULT_ENABLE
// #define MOTOR_FAULT_PIN      AUXINPUT0_PIN  // Disabled: pin not assigned
