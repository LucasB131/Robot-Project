/*
 * test_motors.cpp
 *
 * Unit tests for motors.
 * Also tests that Servo.h and Tone.h doesn't clobber motor timer settings.
 *
 * Authors:
 * - Brian Jia <jia.659@osu.edu>
 */

#include <unity.h>
#include <Arduino.h>
#include <FEH.h>
#include "../private_include/FEHInternal.h"

extern bool debug_motorSlowPwm;

#define nMSLEEP_PIN 27

const volatile uint8_t *MOTOR_PWM_PIN[] = {&PINE, &PINL, &PINL, &PINL};
const uint8_t MOTOR_PWM_MASK[] = {bit(5), bit(4), bit(5), bit(3)};

FEHMotor motors[] = {
    FEHMotor(FEHMotor::FEHMotorPort::Motor0, 12),
    FEHMotor(FEHMotor::FEHMotorPort::Motor1, 12),
    FEHMotor(FEHMotor::FEHMotorPort::Motor2, 12),
    FEHMotor(FEHMotor::FEHMotorPort::Motor3, 12),
};

void test_motor_dir(int motor_num)
{
    int dir_pin = _getMotorDirectionPin(motor_num);
    FEHMotor &m = motors[motor_num];
    for (int i = -100; i <= 100; i++)
    {
        m.SetPercent(i);
        /* DO NOT TEST FOR THE CASE THAT i == 0 */
        /* BECAUSE THE PWM SIGNAL WILL BE OFF IN THAT CASE ANYWAY */
        if (i > 0)
        {
            TEST_ASSERT_TRUE(digitalRead(dir_pin));
        }
        else if (i < 0)
        {
            TEST_ASSERT_FALSE(digitalRead(dir_pin));
        }
    }
}

void test_motor_dir_pin_initialization(int motor_num)
{
    /* Clobber the direction pin modes and then test motor direction */
    pinMode(_getMotorDirectionPin(0), INPUT);
    pinMode(_getMotorDirectionPin(1), INPUT);
    pinMode(_getMotorDirectionPin(2), INPUT);
    pinMode(_getMotorDirectionPin(3), INPUT);
    test_motor_dir(motor_num);
}

bool read_motor_pwm(int motor_num)
{
    if (motor_num > 3)
    {
        TEST_ABORT();
    }

    return (bool)(*MOTOR_PWM_PIN[motor_num] & MOTOR_PWM_MASK[motor_num]);
}

float measure_motor_pwm_high_pct(int motor_num)
{
    /* Wait for PWM to go low */
    while (read_motor_pwm(motor_num))
    {
    }

    /* Wait for PWM to go high */
    while (!read_motor_pwm(motor_num))
    {
    }
    uint32_t high_micros = micros();

    /* Wait for PWM to go low */
    while (read_motor_pwm(motor_num))
    {
    }

    uint32_t low_micros = micros();

    /* Wait for PWM to go high again */
    while (!read_motor_pwm(motor_num))
    {
    }

    uint32_t high_again_micros = micros();

    uint32_t cycle_duration_micros = high_again_micros - high_micros;
    uint32_t high_duration_micros = low_micros - high_micros;

    return ((float)high_duration_micros * 100.0) / cycle_duration_micros;
}

void check_motor_pwm(int motor_num, int pct)
{
    /* Wait for at least a PWM cycle to pass (~28ms at the test PWM speed), so the PWM signal fully updates */
    delay(50);

    int pwm_pct = abs(pct);
    if (pwm_pct == 0)
    {
        /* If it's 0%, make sure output is 0 for a while */
        for (long i = 0; i < 100000; i++)
        {
            TEST_ASSERT_FALSE(read_motor_pwm(motor_num));
            delayMicroseconds(1);
        }
    }
    else if (pwm_pct == 100)
    {
        /* If it's 100%, make sure output is 1 for a while */
        for (long i = 0; i < 100000; i++)
        {
            TEST_ASSERT_TRUE(read_motor_pwm(motor_num));
            delayMicroseconds(1);
        }
    }
    else
    {
        /* If it's 1-99%, measure the PWM high percentage and assert */
        float high_pct = measure_motor_pwm_high_pct(motor_num);
        TEST_ASSERT_FLOAT_WITHIN(0.5, pwm_pct, high_pct);
    }
}

void test_motor_pwm(int motor_num, int pct)
{
    FEHMotor &m = motors[motor_num];
    m.SetPercent(pct);
    check_motor_pwm(motor_num, pct);
}

void test_motor_pwm_withMaxVoltage(int motor_num, float maxVoltage, int pct)
{
    FEHMotor m((FEHMotor::FEHMotorPort)motor_num, maxVoltage);
    m.SetPercent(pct);
    check_motor_pwm(motor_num, pct * (maxVoltage / 12.0));
}

void test_motor_pwm_pin_initialization(int motor_num, int pct)
{
    /* Clobber the PWM pin modes and then test motor PWM */
    pinMode(_getMotorPwmPin(0), INPUT);
    pinMode(_getMotorPwmPin(1), INPUT);
    pinMode(_getMotorPwmPin(2), INPUT);
    pinMode(_getMotorPwmPin(3), INPUT);
    test_motor_pwm(motor_num, pct);
}

void test_motor_pwm_101pct(int motor_num)
{
    FEHMotor &m = motors[motor_num];
    /* set to 50 so it's obvious if something goes wrong */
    m.SetPercent(50);
    /* should be clamped to 100 */
    m.SetPercent(101);
    check_motor_pwm(motor_num, 100);
}

void test_motor_pwm_neg101pct(int motor_num)
{
    FEHMotor &m = motors[motor_num];
    /* set to -50 so it's obvious if something goes wrong */
    m.SetPercent(-50);
    /* should be clamped to -100 */
    m.SetPercent(-101);
    check_motor_pwm(motor_num, -100);
}

void test_m0_dir() { test_motor_dir(0); }
void test_m1_dir() { test_motor_dir(1); }
void test_m2_dir() { test_motor_dir(2); }
void test_m3_dir() { test_motor_dir(3); }
void test_m0_dir_pin_initialization() { test_motor_dir_pin_initialization(0); }
void test_m1_dir_pin_initialization() { test_motor_dir_pin_initialization(1); }
void test_m2_dir_pin_initialization() { test_motor_dir_pin_initialization(2); }
void test_m3_dir_pin_initialization() { test_motor_dir_pin_initialization(3); }
void test_m0_pwm_maxVoltage12_0pct() { test_motor_pwm_withMaxVoltage(0, 12.0, 0); }
void test_m1_pwm_maxVoltage12_0pct() { test_motor_pwm_withMaxVoltage(1, 12.0, 0); }
void test_m2_pwm_maxVoltage12_0pct() { test_motor_pwm_withMaxVoltage(2, 12.0, 0); }
void test_m3_pwm_maxVoltage12_0pct() { test_motor_pwm_withMaxVoltage(3, 12.0, 0); }
void test_m0_pwm_maxVoltage12_25pct() { test_motor_pwm_withMaxVoltage(0, 12.0, 25); }
void test_m1_pwm_maxVoltage12_25pct() { test_motor_pwm_withMaxVoltage(1, 12.0, 25); }
void test_m2_pwm_maxVoltage12_25pct() { test_motor_pwm_withMaxVoltage(2, 12.0, 25); }
void test_m3_pwm_maxVoltage12_25pct() { test_motor_pwm_withMaxVoltage(3, 12.0, 25); }
void test_m0_pwm_maxVoltage12_50pct() { test_motor_pwm_withMaxVoltage(0, 12.0, 50); }
void test_m1_pwm_maxVoltage12_50pct() { test_motor_pwm_withMaxVoltage(1, 12.0, 50); }
void test_m2_pwm_maxVoltage12_50pct() { test_motor_pwm_withMaxVoltage(2, 12.0, 50); }
void test_m3_pwm_maxVoltage12_50pct() { test_motor_pwm_withMaxVoltage(3, 12.0, 50); }
void test_m0_pwm_maxVoltage12_75pct() { test_motor_pwm_withMaxVoltage(0, 12.0, 75); }
void test_m1_pwm_maxVoltage12_75pct() { test_motor_pwm_withMaxVoltage(1, 12.0, 75); }
void test_m2_pwm_maxVoltage12_75pct() { test_motor_pwm_withMaxVoltage(2, 12.0, 75); }
void test_m3_pwm_maxVoltage12_75pct() { test_motor_pwm_withMaxVoltage(3, 12.0, 75); }
void test_m0_pwm_maxVoltage12_100pct() { test_motor_pwm_withMaxVoltage(0, 12.0, 100); }
void test_m1_pwm_maxVoltage12_100pct() { test_motor_pwm_withMaxVoltage(1, 12.0, 100); }
void test_m2_pwm_maxVoltage12_100pct() { test_motor_pwm_withMaxVoltage(2, 12.0, 100); }
void test_m3_pwm_maxVoltage12_100pct() { test_motor_pwm_withMaxVoltage(3, 12.0, 100); }
void test_m0_pwm_maxVoltage4_0pct() { test_motor_pwm_withMaxVoltage(0, 4.0, 0); }
void test_m1_pwm_maxVoltage4_0pct() { test_motor_pwm_withMaxVoltage(1, 4.0, 0); }
void test_m2_pwm_maxVoltage4_0pct() { test_motor_pwm_withMaxVoltage(2, 4.0, 0); }
void test_m3_pwm_maxVoltage4_0pct() { test_motor_pwm_withMaxVoltage(3, 4.0, 0); }
void test_m0_pwm_maxVoltage4_25pct() { test_motor_pwm_withMaxVoltage(0, 4.0, 25); }
void test_m1_pwm_maxVoltage4_25pct() { test_motor_pwm_withMaxVoltage(1, 4.0, 25); }
void test_m2_pwm_maxVoltage4_25pct() { test_motor_pwm_withMaxVoltage(2, 4.0, 25); }
void test_m3_pwm_maxVoltage4_25pct() { test_motor_pwm_withMaxVoltage(3, 4.0, 25); }
void test_m0_pwm_maxVoltage4_50pct() { test_motor_pwm_withMaxVoltage(0, 4.0, 50); }
void test_m1_pwm_maxVoltage4_50pct() { test_motor_pwm_withMaxVoltage(1, 4.0, 50); }
void test_m2_pwm_maxVoltage4_50pct() { test_motor_pwm_withMaxVoltage(2, 4.0, 50); }
void test_m3_pwm_maxVoltage4_50pct() { test_motor_pwm_withMaxVoltage(3, 4.0, 50); }
void test_m0_pwm_maxVoltage4_75pct() { test_motor_pwm_withMaxVoltage(0, 4.0, 75); }
void test_m1_pwm_maxVoltage4_75pct() { test_motor_pwm_withMaxVoltage(1, 4.0, 75); }
void test_m2_pwm_maxVoltage4_75pct() { test_motor_pwm_withMaxVoltage(2, 4.0, 75); }
void test_m3_pwm_maxVoltage4_75pct() { test_motor_pwm_withMaxVoltage(3, 4.0, 75); }
void test_m0_pwm_maxVoltage4_100pct() { test_motor_pwm_withMaxVoltage(0, 4.0, 100); }
void test_m1_pwm_maxVoltage4_100pct() { test_motor_pwm_withMaxVoltage(1, 4.0, 100); }
void test_m2_pwm_maxVoltage4_100pct() { test_motor_pwm_withMaxVoltage(2, 4.0, 100); }
void test_m3_pwm_maxVoltage4_100pct() { test_motor_pwm_withMaxVoltage(3, 4.0, 100); }
void test_m0_pwm_maxVoltage12_neg0pct() { test_motor_pwm_withMaxVoltage(0, 12.0, -0); }
void test_m1_pwm_maxVoltage12_neg0pct() { test_motor_pwm_withMaxVoltage(1, 12.0, -0); }
void test_m2_pwm_maxVoltage12_neg0pct() { test_motor_pwm_withMaxVoltage(2, 12.0, -0); }
void test_m3_pwm_maxVoltage12_neg0pct() { test_motor_pwm_withMaxVoltage(3, 12.0, -0); }
void test_m0_pwm_maxVoltage12_neg25pct() { test_motor_pwm_withMaxVoltage(0, 12.0, -25); }
void test_m1_pwm_maxVoltage12_neg25pct() { test_motor_pwm_withMaxVoltage(1, 12.0, -25); }
void test_m2_pwm_maxVoltage12_neg25pct() { test_motor_pwm_withMaxVoltage(2, 12.0, -25); }
void test_m3_pwm_maxVoltage12_neg25pct() { test_motor_pwm_withMaxVoltage(3, 12.0, -25); }
void test_m0_pwm_maxVoltage12_neg50pct() { test_motor_pwm_withMaxVoltage(0, 12.0, -50); }
void test_m1_pwm_maxVoltage12_neg50pct() { test_motor_pwm_withMaxVoltage(1, 12.0, -50); }
void test_m2_pwm_maxVoltage12_neg50pct() { test_motor_pwm_withMaxVoltage(2, 12.0, -50); }
void test_m3_pwm_maxVoltage12_neg50pct() { test_motor_pwm_withMaxVoltage(3, 12.0, -50); }
void test_m0_pwm_maxVoltage12_neg75pct() { test_motor_pwm_withMaxVoltage(0, 12.0, -75); }
void test_m1_pwm_maxVoltage12_neg75pct() { test_motor_pwm_withMaxVoltage(1, 12.0, -75); }
void test_m2_pwm_maxVoltage12_neg75pct() { test_motor_pwm_withMaxVoltage(2, 12.0, -75); }
void test_m3_pwm_maxVoltage12_neg75pct() { test_motor_pwm_withMaxVoltage(3, 12.0, -75); }
void test_m0_pwm_maxVoltage12_neg100pct() { test_motor_pwm_withMaxVoltage(0, 12.0, -100); }
void test_m1_pwm_maxVoltage12_neg100pct() { test_motor_pwm_withMaxVoltage(1, 12.0, -100); }
void test_m2_pwm_maxVoltage12_neg100pct() { test_motor_pwm_withMaxVoltage(2, 12.0, -100); }
void test_m3_pwm_maxVoltage12_neg100pct() { test_motor_pwm_withMaxVoltage(3, 12.0, -100); }
void test_m0_pwm_maxVoltage4_neg0pct() { test_motor_pwm_withMaxVoltage(0, 4.0, -0); }
void test_m1_pwm_maxVoltage4_neg0pct() { test_motor_pwm_withMaxVoltage(1, 4.0, -0); }
void test_m2_pwm_maxVoltage4_neg0pct() { test_motor_pwm_withMaxVoltage(2, 4.0, -0); }
void test_m3_pwm_maxVoltage4_neg0pct() { test_motor_pwm_withMaxVoltage(3, 4.0, -0); }
void test_m0_pwm_maxVoltage4_neg25pct() { test_motor_pwm_withMaxVoltage(0, 4.0, -25); }
void test_m1_pwm_maxVoltage4_neg25pct() { test_motor_pwm_withMaxVoltage(1, 4.0, -25); }
void test_m2_pwm_maxVoltage4_neg25pct() { test_motor_pwm_withMaxVoltage(2, 4.0, -25); }
void test_m3_pwm_maxVoltage4_neg25pct() { test_motor_pwm_withMaxVoltage(3, 4.0, -25); }
void test_m0_pwm_maxVoltage4_neg50pct() { test_motor_pwm_withMaxVoltage(0, 4.0, -50); }
void test_m1_pwm_maxVoltage4_neg50pct() { test_motor_pwm_withMaxVoltage(1, 4.0, -50); }
void test_m2_pwm_maxVoltage4_neg50pct() { test_motor_pwm_withMaxVoltage(2, 4.0, -50); }
void test_m3_pwm_maxVoltage4_neg50pct() { test_motor_pwm_withMaxVoltage(3, 4.0, -50); }
void test_m0_pwm_maxVoltage4_neg75pct() { test_motor_pwm_withMaxVoltage(0, 4.0, -75); }
void test_m1_pwm_maxVoltage4_neg75pct() { test_motor_pwm_withMaxVoltage(1, 4.0, -75); }
void test_m2_pwm_maxVoltage4_neg75pct() { test_motor_pwm_withMaxVoltage(2, 4.0, -75); }
void test_m3_pwm_maxVoltage4_neg75pct() { test_motor_pwm_withMaxVoltage(3, 4.0, -75); }
void test_m0_pwm_maxVoltage4_neg100pct() { test_motor_pwm_withMaxVoltage(0, 4.0, -100); }
void test_m1_pwm_maxVoltage4_neg100pct() { test_motor_pwm_withMaxVoltage(1, 4.0, -100); }
void test_m2_pwm_maxVoltage4_neg100pct() { test_motor_pwm_withMaxVoltage(2, 4.0, -100); }
void test_m3_pwm_maxVoltage4_neg100pct() { test_motor_pwm_withMaxVoltage(3, 4.0, -100); }
void test_m0_pwm_101pct() { test_motor_pwm_101pct(0); }
void test_m1_pwm_101pct() { test_motor_pwm_101pct(1); }
void test_m2_pwm_101pct() { test_motor_pwm_101pct(2); }
void test_m3_pwm_101pct() { test_motor_pwm_101pct(3); }
void test_m0_pwm_neg101pct() { test_motor_pwm_neg101pct(0); };
void test_m1_pwm_neg101pct() { test_motor_pwm_neg101pct(1); };
void test_m2_pwm_neg101pct() { test_motor_pwm_neg101pct(2); };
void test_m3_pwm_neg101pct() { test_motor_pwm_neg101pct(3); };
void test_m0_pwm_pin_initialization() { test_motor_pwm_pin_initialization(0, 25); }
void test_m1_pwm_pin_initialization() { test_motor_pwm_pin_initialization(1, 25); }
void test_m2_pwm_pin_initialization() { test_motor_pwm_pin_initialization(2, 25); }
void test_m3_pwm_pin_initialization() { test_motor_pwm_pin_initialization(3, 25); }
void test_is_nMSLEEP_asserted_to_wake_up_motors() { TEST_ASSERT_TRUE(digitalRead(MOTOR_nSLEEP_PIN)); }

void test_that_servos_dont_conflict_with_motors()
{
    test_motor_pwm(0, 0);
    test_motor_pwm(1, 25);
    test_motor_pwm(2, 50);
    test_motor_pwm(3, 75);

    /* Initialize servos.*/
    /* A timer conflict might occur here. */
    FEHServo servos[] = {
        FEHServo(FEHServo::FEHServoPort::Servo0),
        FEHServo(FEHServo::FEHServoPort::Servo1),
        FEHServo(FEHServo::FEHServoPort::Servo2),
        FEHServo(FEHServo::FEHServoPort::Servo3),
        FEHServo(FEHServo::FEHServoPort::Servo4),
        FEHServo(FEHServo::FEHServoPort::Servo5),
        FEHServo(FEHServo::FEHServoPort::Servo6),
        FEHServo(FEHServo::FEHServoPort::Servo7),
    };

    /* Set servos willy nilly. */
    /* A timer conflict might also occur here. */
    for (int i = 0; i < 8; i++)
    {
        servos[i].SetDegree(i * 20);
    }

    /* Check that the motor PWMs are still set to what they should be, and that the Servo.h library hasn't overwritten the motor timer settings. */
    check_motor_pwm(0, 0);
    check_motor_pwm(1, 25);
    check_motor_pwm(2, 50);
    check_motor_pwm(3, 75);
}

void test_that_buzzer_doesnt_conflict_with_motors()
{
    test_motor_pwm(0, 0);
    test_motor_pwm(1, 25);
    test_motor_pwm(2, 50);
    test_motor_pwm(3, 75);

    /* Beeeeeeep!!!! */
    Buzzer.Beep();

    /* Wait for beep to stop, Buzzer.Beep() beeps for 500ms */
    delay(600);

    /* Check that the motor PWMs are still set to what they should be, and that Buzzer hasn't overwritten the motor timer settings. */
    check_motor_pwm(0, 0);
    check_motor_pwm(1, 25);
    check_motor_pwm(2, 50);
    check_motor_pwm(3, 75);
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    
    /* Turn on slow PWM so we can measure the motor PWM outputs in software */
    debug_motorSlowPwm = true;

    UNITY_BEGIN();

    /* nMSLEEP is asserted true in the FEHMotor constructor so this should be true */
    /* This depends on globally declared FEHMotors above to initialize properly, */
    /* so this should also test if global FEHMotor objects initialize correctly. */
    RUN_TEST(test_is_nMSLEEP_asserted_to_wake_up_motors);

    /* Test that the direction pin is being set appropriately based on the sign of the power percentage passed in */
    RUN_TEST(test_m0_dir);
    RUN_TEST(test_m1_dir);
    RUN_TEST(test_m2_dir);
    RUN_TEST(test_m3_dir);

    /* Test that FEHMotor::SetPercent() sets the corresponding direction pin to OUTPUT, in case a student willy-nilly calls pinMode() on a motor pin */
    RUN_TEST(test_m0_dir_pin_initialization);
    RUN_TEST(test_m1_dir_pin_initialization);
    RUN_TEST(test_m2_dir_pin_initialization);
    RUN_TEST(test_m3_dir_pin_initialization);

    /* Tests that measure the PWM output for accuracy, at different maxVoltage levels */
    RUN_TEST(test_m0_pwm_maxVoltage12_0pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_0pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_0pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_0pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_25pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_25pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_25pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_25pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_50pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_50pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_50pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_50pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_75pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_75pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_75pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_75pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_100pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_100pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_100pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_100pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_0pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_0pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_0pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_0pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_25pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_25pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_25pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_25pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_50pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_50pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_50pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_50pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_75pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_75pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_75pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_75pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_100pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_100pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_100pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_100pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_neg0pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_neg0pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_neg0pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_neg0pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_neg25pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_neg25pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_neg25pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_neg25pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_neg50pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_neg50pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_neg50pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_neg50pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_neg75pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_neg75pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_neg75pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_neg75pct);
    RUN_TEST(test_m0_pwm_maxVoltage12_neg100pct);
    RUN_TEST(test_m1_pwm_maxVoltage12_neg100pct);
    RUN_TEST(test_m2_pwm_maxVoltage12_neg100pct);
    RUN_TEST(test_m3_pwm_maxVoltage12_neg100pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_neg0pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_neg0pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_neg0pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_neg0pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_neg25pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_neg25pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_neg25pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_neg25pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_neg50pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_neg50pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_neg50pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_neg50pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_neg75pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_neg75pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_neg75pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_neg75pct);
    RUN_TEST(test_m0_pwm_maxVoltage4_neg100pct);
    RUN_TEST(test_m1_pwm_maxVoltage4_neg100pct);
    RUN_TEST(test_m2_pwm_maxVoltage4_neg100pct);
    RUN_TEST(test_m3_pwm_maxVoltage4_neg100pct);

    /* Make sure things are ok even if students are a little silly */
    RUN_TEST(test_m0_pwm_101pct);
    RUN_TEST(test_m1_pwm_101pct);
    RUN_TEST(test_m2_pwm_101pct);
    RUN_TEST(test_m3_pwm_101pct);
    RUN_TEST(test_m0_pwm_neg101pct);
    RUN_TEST(test_m1_pwm_neg101pct);
    RUN_TEST(test_m2_pwm_neg101pct);
    RUN_TEST(test_m3_pwm_neg101pct);

    /* Test that FEHMotor::SetPercent() sets the corresponding PWM pin to OUTPUT, in case a student willy-nilly calls pinMode() on a motor pin */
    RUN_TEST(test_m0_pwm_pin_initialization);
    RUN_TEST(test_m1_pwm_pin_initialization);
    RUN_TEST(test_m2_pwm_pin_initialization);
    RUN_TEST(test_m3_pwm_pin_initialization);

    /* Test that the Servo.h library doesn't try to use the same timers as the FEH.h motor library */
    RUN_TEST(test_that_servos_dont_conflict_with_motors);
    /* TODO: Test that the Buzzer API doesn't try to use the same timers as the motors */
    RUN_TEST(test_that_buzzer_doesnt_conflict_with_motors);

    UNITY_END();
}

void loop()
{
}