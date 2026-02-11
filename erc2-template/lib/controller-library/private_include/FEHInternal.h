/**
 * @file FEHInternal.h
 * @author Brian Jia <jia.659@osu.edu>
 * @brief Internal helper functions and declarations for the FEH library
 *
 * This header contains private functions and external references used internally
 * by the FEH library. These functions are not part of the public API and should
 * not be called directly by user code.
 *
 * Key functionalities:
 * - Hardware access (LCD, touchscreen, SD card)
 * - Motor pin mapping
 * - Error handling and fatal error management
 * - Robot kill/safety mechanisms
 * - Hardware fault detection
 * - Battery monitoring
 */

#ifndef FEHINTERNAL_H
#define FEHINTERNAL_H

#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include <SdFat.h>

//=============================================================================
// EXTERNAL HARDWARE INTERFACE OBJECTS
//=============================================================================

/// @brief ILI9341 LCD display controller instance
extern Adafruit_ILI9341 ILI9341;

/// @brief FT6206 capacitive touchscreen controller instance
extern Adafruit_FT6206 FT6206;

/// @brief SD card FAT filesystem interface
extern SdFat FAT;

//=============================================================================
// MOTOR PIN MAPPING
//=============================================================================

/**
 * @brief Get the PWM pin for a specific motor
 *
 * Retrieves the Arduino pin number used for PWM control of the specified motor.
 * Motor indices are 0-based.
 *
 * @param motorIndex Motor number (0-3)
 * @return Arduino pin number for motor PWM
 */
uint8_t _getMotorPwmPin(uint8_t motorIndex);

/**
 * @brief Get the direction pin for a specific motor
 *
 * Retrieves the Arduino pin number used for direction control of the specified motor.
 * Motor indices are 0-based.
 *
 * @param motorIndex Motor number (0-3)
 * @return Arduino pin number for motor direction
 */
uint8_t _getMotorDirectionPin(uint8_t motorIndex);

//=============================================================================
// ERROR HANDLING AND DISPLAY
//=============================================================================

/**
 * @brief Prepare LCD for displaying an error message
 *
 * Initializes the LCD display with error formatting (red background, white text,
 * large "ERROR!" heading). Called before displaying fatal error messages.
 *
 * @note Safe to call multiple times - reinitializes LCD if needed
 */
void _lcdErrorPrelude();

/**
 * @brief Check if a value is within a specified range
 *
 * Validates that a value falls within the specified inclusive range [min, max].
 * If the value is out of range, prints an error message to Serial but does not halt.
 *
 * @param funcName Name of the calling function (for error message)
 * @param valName Name of the parameter being checked (for error message)
 * @param val Value to check
 * @param min Minimum allowed value (inclusive)
 * @param max Maximum allowed value (inclusive)
 * @return true if value is in range, false otherwise
 */
bool _checkRange(const char *funcName, const char *valName, int val, int min, int max);

/**
 * @brief Check if a value is within range and kill robot if not
 *
 * Similar to _checkRange() but treats range violations as fatal errors.
 * If the value is out of range, displays error on LCD and kills the robot.
 *
 * @param funcName Name of the calling function (for error message)
 * @param valName Name of the parameter being checked (for error message)
 * @param val Value to check
 * @param min Minimum allowed value (inclusive)
 * @param max Maximum allowed value (inclusive)
 * @return true if value is in range, false otherwise (robot killed if false)
 */
bool _checkRangeFatal(const char *funcName, const char *valName, int val, int min, int max);

/**
 * @brief Trigger a fatal error with a custom message
 *
 * Displays the error message on the LCD with red background and kills the robot.
 * This function does not return - the robot will be stopped and locked in a
 * halted state until power cycled.
 *
 * @param msg Error message to display on LCD and serial console
 *
 * @note This function ensures Arduino is fully initialized before displaying error
 * @note Safe to call from global constructors (before main() runs)
 */
void _fatalError(const char *msg);

/**
 * @brief Trigger a fatal error with default message
 *
 * Calls _fatalError() with a generic "Unspecified fatal error" message.
 * Useful for error conditions where a specific message isn't available.
 */
void _fatalError();

//=============================================================================
// ROBOT KILL / SAFETY MECHANISMS
//=============================================================================

/**
 * @brief Kill the robot without LCD output
 *
 * Immediately stops all motors, disables interrupts, and optionally plays a
 * warning tone. Does not display anything on the LCD. Used when the LCD is
 * unavailable or when called before error display.
 *
 * @param loop If true, loop forever after kill (default: true)
 * @param tone If true, play warning tone before kill (default: true)
 *
 * @note Motors are put to sleep and all PWM outputs are stopped
 * @note All interrupts are disabled to prevent further code execution
 */
void _killNoScreen(bool loop = true, bool tone = true);

/**
 * @brief Kill the robot with a specific reason
 *
 * Stops all motors, displays "KILLED" message on LCD with the specified reason,
 * and enters an infinite loop. The robot must be power cycled to recover.
 *
 * @param reason Description of why the robot was killed (displayed on LCD)
 *
 * @note This is the primary robot kill function - displays full status
 * @note Used for RCS kills, hardware faults, and safety violations
 */
void _kill(const char *reason);

/**
 * @brief Kill the robot with default message
 *
 * Calls _kill() with a generic "Unspecified kill" message.
 */
void _kill();

/**
 * @brief ESP32 kill signal interrupt handler
 *
 * Called when the ESP32 pulls the BOOT_SEL pin low, indicating an RCS kill
 * command has been received. Verifies the shield is powered on before killing
 * to distinguish between legitimate kills and power-down events.
 *
 * @note Attached to BOOT_SEL pin falling edge interrupt
 * @note If shield is off, performs software reset instead of kill
 */
void _kill_esp();

//=============================================================================
// HARDWARE MONITORING
//=============================================================================

/**
 * @brief Read battery voltage
 *
 * Measures the battery voltage using the ADC. The voltage is read through a
 * voltage divider (divide by 3) and scaled appropriately.
 *
 * @return Battery voltage in volts (typically 10-13V when powered on)
 *
 * @note Uses 12-bit ADC with 5V reference
 * @note Below LOW_BATTERY_THRESHOLD indicates low battery or power off
 */
float _batteryVoltage();

/**
 * @brief Check for I2C bus fault
 *
 * Reads the I2C_nFAULT pin to detect I2C bus errors. The TCA9555 I/O expander
 * pulls this pin low when an I2C fault is detected.
 *
 * @return true if I2C fault is present, false if bus is healthy
 *
 * @note Fault can indicate shield is powered off or I2C bus error
 */
bool _I2CFault();

/**
 * @brief Check for I/O expander fault
 *
 * Reads the IO_nFAULT pin to detect I/O expander errors. This pin is pulled
 * low when the I/O expander encounters a fault condition.
 *
 * @return true if I/O fault is present, false if I/O is healthy
 *
 * @note Fault can indicate shield is powered off or hardware failure
 */
bool _IOFault();

#endif // FEHINTERNAL_H