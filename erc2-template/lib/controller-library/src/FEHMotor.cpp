/**
 * FEHMotor.cpp
 *
 * FEH motor library.
 *
 * Direct register access is used because PWM frequency cannot be controlled through the Arduino API,
 * and high PWM frequencies create switching losses in the motor drivers.
 *
 * @author Brian Jia <jia.659@osu.edu>
 */

#include <FEH.h>
#include "../private_include/FEHInternal.h"
#include <Arduino.h>

/* If set to true, motor PWM is slowed down so it can be measured in software by a unit test. */
/* Used by test/test_motors_and_servos. */
bool debug_motorSlowPwm = false;


uint8_t _getMotorPwmPin(uint8_t motorIndex)
{
    return pgm_read_byte(MOTOR_PWM_PINS + motorIndex);
}

uint8_t _getMotorDirectionPin(uint8_t motorIndex)
{
    return pgm_read_byte(MOTOR_DIRECTION_PINS + motorIndex);
}

FEHMotor::FEHMotor(FEHMotor::FEHMotorPort motorPort, float maxVoltage)
    : _motorPort(motorPort)
{
    if (!_checkRangeFatal("FEHMotor::FEHMotor", "motorPort", motorPort, 0, 3))
    {
        return;
    }

    // Voltage Range: 1.0 - 12.0 V
    if (maxVoltage > 12)
    {
        maxVoltage = 12.0;
    }
    else if (maxVoltage <= 1)
    {
        maxVoltage = 1.0;
    }

    /* This is used to scale the passed-in percentage in FEHMotor::SetPercent() */
    _powerScalingFactor = (uint8_t)((maxVoltage / 12.0) * 255.0);

    /*
     * Initialize Timers 3 and 5 which we use for motor PWM.
     * Both these timers are 16-bit Timer/Counters
     */

    /* Set Waveform Generation Mode (WGM) */
    // set WGM0 and WGM2 to 1 for fast 8-bit PWM mode
    uint8_t wgm_tccr_a = bit(WGM00) | bit(WGM02);

    /* Enable non-inverting 8-bit PWM for Timer 3 and Timer 5 */
    // COMnA1 = 1, COMnA0 = 0 for "Clear OCnA/OCnB/OCnC on compare match (set output to low level)"
    // -> basically means if OCR value is 100 the PWM is high 100/255 of the time.
    TCCR3A = bit(COM3C1) | wgm_tccr_a;
    TCCR5A = bit(COM5A1) | bit(COM5B1) | bit(COM5C1) | wgm_tccr_a;

    /* Set timer clocks to clk_I/O / 1 (No prescaling) */
    // Produces a 31.25 kHz PWM signal, verified with oscilloscope.
    // We probably want to keep it this high to keep it outside of human hearing range.
    TCCR3B = bit(CS30);
    TCCR5B = bit(CS50);

    switch (motorPort)
    {
    case Motor0: // D3, PE5, OC3C
        /* Set duty cycle to 0 */
        OCR3C = 0;
        break;

    case Motor1: // D45, PL4, OC5B
        OCR5B = 0;
        break;

    case Motor2: // D44, PL5, OC5C
        OCR5C = 0;
        break;

    case Motor3: // D46, PL3, OC5A
        OCR5A = 0;
        break;

    default:
        _fatalError();
    }
}

void FEHMotor::SetPercent(int8_t percent)
{
    /*
     * Motor driver: TI DRV8874
     *
     * Motor drivers are wired in PH/EN control mode,
     * which is a speed and direction type of interface.
     *
     * Speed is PWM controlled through the EN pin.
     * Direction is controlled through the PH pin.
     */

    /* Slow PWM is a debugging option so that pulse lengths can be measured during unit testing. */
    if (debug_motorSlowPwm)
    {
        /* Set timer clocks to clk_I/O / 1024 */
        // Produces a ~30.52 Hz PWM signal.
        TCCR3B = bit(CS32) | bit(CS30);
        TCCR5B = bit(CS52) | bit(CS50);
    }
    else
    {
        /* Set timer clocks to clk_I/O / 1 (No prescaling) */
        // Produces a 31.25 kHz PWM signal, verified with oscilloscope.
        // We probably want to keep it this high to keep it outside of human hearing range.
        TCCR3B = bit(CS30);
        TCCR5B = bit(CS50);
    }

    // TODO: Do we want to change PWM frequency? Is currently 31 kHz. Absolute max on chip side is 100 kHz.

    /* Clamp motor power to [-100, 100]. */
    /* Replicates Proteus behavior */
    if (percent < -100)
    {
        percent = -100;
    }
    else if (percent > 100)
    {
        percent = 100;
    }

    /* Scale for 8-bit PWM, and based on power scaling factor */
    int pwm = (abs(percent) * _powerScalingFactor) / 100;

    /* Determine direction based on if percentage is positive or negative */
    /* Invert direction so LED indicator appears correct*/
    bool direction = percent < 0;

    uint8_t directionPin = _getMotorDirectionPin(_motorPort);
    /* Set direction pin as output*/
    pinMode(directionPin, OUTPUT);
    /* Write to direction pin */
    digitalWrite(directionPin, direction);
    /* Set PWM pin as output */
    pinMode(_getMotorPwmPin(_motorPort), OUTPUT);

    switch (_motorPort)
    {
    case Motor0: // D3, PE5, OC3C
        /* Set pulse width */
        OCR3C = pwm;
        break;
    case Motor1: // D45, PL4, OC5B
        OCR5B = pwm;
        break;
    case Motor2: // D44, PL5, OC5C
        OCR5C = pwm;
        break;
    case Motor3: // D46, PL3, OC5A
        OCR5A = pwm;
        break;
    }
}

void FEHMotor::Stop()
{
    this->SetPercent(0);
}


void FEHMotor::StopAll()
{
    for (uint8_t motorIndex = 0; motorIndex < 4; motorIndex++)
    {
        // Declare and stop all
        FEHMotor motor((FEHMotor::FEHMotorPort)motorIndex, 12.0); // 12V is the max voltage for all motors
        motor.Stop();
    }
}

void FEHMotor::SetAllSleep(bool sleep)
{
    pinMode(MOTOR_nSLEEP_PIN, OUTPUT);
    digitalWrite(MOTOR_nSLEEP_PIN, !sleep);
}


