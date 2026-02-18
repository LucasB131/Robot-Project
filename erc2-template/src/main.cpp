#include <FEH.h>
#include <Arduino.h>

AnalogInputPin sensor(FEHIO::Pin0);

FEHMotor left_motor(FEHMotor::Motor0, 9.0);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);

DigitalInputPin back_left_bump_switch(FEHIO::Pin12);
DigitalInputPin back_right_bump_switch(FEHIO::Pin13);
DigitalInputPin front_right_bump_switch(FEHIO::Pin14);
DigitalInputPin front_left_bump_switch(FEHIO::Pin15);

bool start = false;
bool second = false;
bool third = false;
bool fourth = false;
bool fifth = false;

void ERCMain()
{
    while (true)
    {

        bool back_left = back_left_bump_switch.Value();
        bool back_right = back_right_bump_switch.Value();
        bool front_right = front_right_bump_switch.Value();
        bool front_left = front_left_bump_switch.Value();

        if (back_left)
        {
            start = true;
        }

        if (start)
        {
            left_motor.SetPercent(40);
            right_motor.SetPercent(40);

            if (front_right || front_left)
            {
                second = true;
                start = false;
            }
        }

        else if (second)
        {
            left_motor.SetPercent(30);
            right_motor.SetPercent(-40);

            if (back_right || back_left)
            {
                second = false;
                third = true;
            }
        }

        else if (third)
        {
            left_motor.SetPercent(40);
            right_motor.SetPercent(40);

            if (front_left || front_right)
            {
                third = false;
                fourth = true;
            }
        }

        else if (fourth)
        {
            left_motor.SetPercent(-40);
            right_motor.SetPercent(30);

            if (back_right || back_left)
            {
                fourth = false;
                fifth = true;
            }
        }

        else if (fifth)
        {
            left_motor.SetPercent(40);
            right_motor.SetPercent(40);

            if (front_left || front_right)
            {
                fifth = false;

                left_motor.Stop();
                right_motor.Stop();
            }
        }
    }
}
