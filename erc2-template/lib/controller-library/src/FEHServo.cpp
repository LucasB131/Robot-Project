/**
 * FEHServo.cpp
 *
 * FEH servo library.
 *
 * A thin, student-friendly wrapper around the stock Arduino Servo library.
 *
 * Note that this library is not being pulled from PlatformIO,
 * rather the library is copied into the project (lib/Servo) and ServoTimers.h is modified to set
 * the servo control timer to Timer 1. This is necessary because other timers are allocated
 * to specific functionality (e.g. Timer 3 and Timer 5 for motor PWM.)
 *
 * @author Brian Jia <jia.659@osu.edu>
 */

#include <FEH.h>
#include "../private_include/FEHInternal.h"
#include "../private_include/Servo/Servo.h"
#include <Arduino.h>

/* Servo library objects */
static Servo servos[NUM_SERVOS];
/* Mapping of controller servo port numbers to Arduino pins */
const static PROGMEM uint8_t SERVO_NUM_TO_PIN[NUM_SERVOS] = {12, 11, 10, 9, 8, 7, 6, 5};

FEHServo::FEHServo(FEHServoPort servoPort)
    : _servoPort(servoPort), _servoMin(MIN_PULSE_WIDTH), _servoMax(MAX_PULSE_WIDTH)
{
    if (!_checkRange("servoInit", "servoNum", servoPort, 0, Servo7))
    {
        return;
    }

    // Servo should not be attached during constructor, as it may occur outside of the main program flow.
    // First call to any other method will attach the servo.
}

void FEHServo::SetDegree(int16_t degree)
{
    if (!_checkRange("SetDegree", "degree", degree, 0, 180))
    {
        return;
    }

    /* Ensure servo is attached, Off() may have detached it. */
    if (!servos[_servoPort].attached())
    {
        servos[_servoPort].attach(pgm_read_byte(SERVO_NUM_TO_PIN + _servoPort), _servoMin, _servoMax);
    }

    servos[_servoPort].write(degree);
}

void FEHServo::TouchCalibrate()
{

    /* Ensure servo is attached, Off() may have detached it. */
    if (!servos[_servoPort].attached())
    {
        servos[_servoPort].attach(pgm_read_byte(SERVO_NUM_TO_PIN + _servoPort), _servoMin, _servoMax);
    }
    
    // Modified from Proteus source code to be slightly more readable
    int servo_min = MIN_PULSE_WIDTH, servo_max = MAX_PULSE_WIDTH;
    int x = 0, y = 0;

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    FEHIcon::Icon VAL[2];
    char val_labels[2][20] = {"Current Minimum", ""};
    FEHIcon::DrawIconArray(VAL, 2, 1, 41, 160, 1, 1, val_labels, YELLOW, WHITE);

    FEHIcon::Icon MOVE[2];
    char move_labels[2][20] = {"Backward", "Forward"};
    FEHIcon::DrawIconArray(MOVE, 1, 2, 80, 40, 1, 1, move_labels, RED, WHITE);

    FEHIcon::Icon SET[1];
    char set_label[1][20] = {"SET MIN"};
    FEHIcon::DrawIconArray(SET, 1, 1, 201, 2, 1, 1, set_label, BLUE, WHITE);

    LCD.SetTextCursor(0, 0);
    LCD.WriteLine("Use icons to select min.");
    LCD.WriteLine("Press "
                  "SET MIN"
                  " when ready.");

    servos[_servoPort].write(servo_min);

    while (!SET[0].Pressed(x, y, 0))
    {
        VAL[1].ChangeLabelInt(_servoMin);
        if (LCD.Touch(&x, &y))
        {
            if (MOVE[0].Pressed(x, y, 0))
            {
                while (MOVE[0].Pressed(x, y, 1))
                {
                    servo_min--;
                    if (servo_min < MIN_PULSE_WIDTH)
                        servo_min = MIN_PULSE_WIDTH;
                    servos[_servoPort].write(servo_min);
                    VAL[1].ChangeLabelInt(servo_min);
                }
                MOVE[0].Deselect();
            }
            if (MOVE[1].Pressed(x, y, 0))
            {
                while (MOVE[1].Pressed(x, y, 1))
                {
                    servo_min++;
                    if (servo_min > MAX_PULSE_WIDTH)
                        servo_min = MAX_PULSE_WIDTH;
                    servos[_servoPort].write(servo_min);
                    VAL[1].ChangeLabelInt(servo_min);
                }
                MOVE[1].Deselect();
            }
        }
    }
    SET[0].WhilePressed(x, y);
    SET[0].Deselect();

    LCD.Clear(BLACK);

    VAL[0].ChangeLabelString("Current Maximum");
    VAL[0].Draw();
    VAL[1].Draw();

    FEHIcon::DrawIconArray(MOVE, 1, 2, 80, 40, 1, 1, move_labels, RED, WHITE);

    SET[0].ChangeLabelString("SET MAX");
    SET[0].Draw();

    LCD.SetTextCursor(0, 0);
    LCD.WriteLine("Use icons to select max.");
    LCD.WriteLine("Press "
                  "SET MAX"
                  " when ready.");

    servos[_servoPort].write(servo_max);

    while (!SET[0].Pressed(x, y, 0))
    {
        VAL[1].ChangeLabelInt(servo_max);
        if (LCD.Touch(&x, &y))
        {
            if (MOVE[0].Pressed(x, y, 0))
            {
                while (MOVE[0].Pressed(x, y, 1))
                {
                    servo_max--;
                    if (servo_max < MIN_PULSE_WIDTH)
                        servo_max = MIN_PULSE_WIDTH;
                    servos[_servoPort].write(servo_max);
                    VAL[1].ChangeLabelInt(servo_max);
                }
                MOVE[0].Deselect();
            }
            if (MOVE[1].Pressed(x, y, 0))
            {
                while (MOVE[1].Pressed(x, y, 1))
                {
                    servo_max++;
                    if (servo_max > MAX_PULSE_WIDTH)
                        servo_max = MAX_PULSE_WIDTH;
                    servos[_servoPort].write(servo_max);
                    VAL[1].ChangeLabelInt(servo_max);
                }
                MOVE[1].Deselect();
            }
        }
    }
    SET[0].WhilePressed(x, y);
    SET[0].Deselect();

    // Set the smaller value to min and larger value to max
    if (servo_min > servo_max)
    {
        unsigned short temp_min;
        temp_min = servo_min;
        servo_min = servo_max;
        servo_max = temp_min;
    }

    LCD.Clear(BLACK);

    FEHIcon::Icon OUT[4];
    char out_labels[4][20] = {"SERVO MIN", "SERVO MAX", "", ""};
    FEHIcon::DrawIconArray(OUT, 2, 2, 80, 120, 20, 20, out_labels, BLACK, WHITE);

    FEHIcon::Icon EXIT[1];
    char exit_label[1][20] = {"EXIT"};
    FEHIcon::DrawIconArray(EXIT, 1, 1, 121, 40, 20, 20, exit_label, RED, WHITE);

    OUT[2].ChangeLabelInt(servo_min);
    OUT[2].Draw();
    OUT[3].ChangeLabelInt(servo_max);
    OUT[3].Draw();

    while (!EXIT[0].Pressed(x, y, 0))
    {
        LCD.Touch(&x, &y);
    }
    EXIT[0].WhilePressed(x, y);
    LCD.Clear(BLACK);


    // Detach the servo to save power
    servos[_servoPort].detach();


}

void FEHServo::Off()
{
    servos[_servoPort].detach();
}

void FEHServo::SetMax(int16_t max)
{
    _servoMax = max;
    /* Only reattach if the servo is already attached */
    if (servos[_servoPort].attached())
    {
        servos[_servoPort].attach(pgm_read_byte(SERVO_NUM_TO_PIN + _servoPort), _servoMin, _servoMax);
    }
}

void FEHServo::SetMin(int16_t min)
{
    _servoMin = min;
    /* Only reattach if the servo is already attached */
    if (servos[_servoPort].attached())
    {
        servos[_servoPort].attach(pgm_read_byte(SERVO_NUM_TO_PIN + _servoPort), _servoMin, _servoMax);
    }
}


// Destructor, detaches the servo
FEHServo::~FEHServo()
{
   Off();
}