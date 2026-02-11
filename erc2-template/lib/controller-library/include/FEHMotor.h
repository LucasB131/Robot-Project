/**
 * FEHMotor.h
 */

#ifndef FEHMOTOR_H
#define FEHMOTOR_H

#include <stdint.h>

/**
 * @brief Access to the motor ports
 *
 * @author Brian Jia <jia.659@osu.edu>
 */
class FEHMotor
{
public:
    /**
     * @brief Motor port values to be used when declaring an FEHMotor
     *
     * Motor port values to be used when declaring an FEHMotor
     */
    typedef enum
    {
        Motor0 = 0,
        Motor1,
        Motor2,
        Motor3
    } FEHMotorPort;

    /**
     * @brief Declare a new FEHMotor object
     *
     * Reccommended max voltages for provided DC motors:<br/>
     * - Acroname = 12.0<br/>
     * - Hacked Futaba = 5.0<br/>
     * - Hacked FITEC = 5.0<br/>
     * - Igwan = 9.0<br/>
     * - Vex Motor = 7.2<br/>
     * - GMH-34 = 7.2
     *
     * @param motorport Motor port number used to power motor
     * @param max_voltage Maximum voltage allows to power motor
     */
    FEHMotor(FEHMotorPort motorPort, float maxVoltage);

    /**
     * @brief Stop powering a motor
     *
     * Stop powering a motor.<br/>
     * Note: This is the same as using SetPercent(0), so it will not apply physical brake force to the motor. Momentum may still affect your movement.
     *
     */
    void Stop();

    /**
     * @brief Set motor percent for your motor
     *
     * Motor percent must be greater than -100 and less than 100.
     *
     * @param percent Percent power to motor
     */
    void SetPercent(int8_t percent);

    /// @brief Stop all motors
    static void StopAll();

    /// @brief Set the sleep state of all motors
    static void SetAllSleep(bool sleep);

private:
    uint8_t _powerScalingFactor;
    FEHMotorPort _motorPort;
};

#endif // FEHMOTOR_H