#include <FEH.h>
#include <Arduino.h>

AnalogInputPin sensor(FEHIO::Pin0);

FEHMotor left_motor(FEHMotor::Motor0, 9.0);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);

DigitalInputPin back_left_bump_switch(FEHIO::Pin12);
DigitalInputPin back_right_bump_switch(FEHIO::Pin13);
DigitalInputPin front_right_bump_switch(FEHIO::Pin14);
DigitalInputPin front_left_bump_switch(FEHIO::Pin15);

bool startit = true;
bool start = false;
bool second = false;
bool third = false;
bool fourth = false;
bool fifth = false;
bool sixth = false;

void ERCMain()
{
    while (true)
    {

        bool back_left = back_left_bump_switch.Value();
        bool back_right = back_right_bump_switch.Value();
        bool front_right = front_right_bump_switch.Value();
        bool front_left = front_left_bump_switch.Value();

        float forward_speed = 30;
        float backward_speed = -20;
        float turn_speed = 20;

        if (startit) {
            if (!back_left) 
            {
                start = true;
                startit = false;
            }
        }

        if (start) 
        {
            left_motor.SetPercent(forward_speed);
            right_motor.SetPercent(forward_speed);

            if (!front_right && !front_left)
            {
                start = false;
                second = true;
            }
        }

        else if (second) 
        {
            left_motor.SetPercent(backward_speed);
            left_motor.SetPercent(backward_speed);
            Sleep(1.0);

            left_motor.SetPercent(turn_speed);
            right_motor.SetPercent(-(turn_speed));
            Sleep(1.0);
            
            second = false;
            third = true;
        }

        else if (third)
        {
            left_motor.SetPercent(backward_speed);
            right_motor.SetPercent(backward_speed);

            if (!front_left && !front_right)
            {
                third = false;
                fourth = true;
            }
        }

        else if (fourth)
        {
            left_motor.SetPercent(forward_speed);
            right_motor.SetPercent(forward_speed);

            if (!back_right && !back_left)
            {
                fourth = false;
                fifth = true;
            }
        }

        else if (fifth)
        {
            left_motor.SetPercent(backward_speed);
            right_motor.SetPercent(backward_speed);
            Sleep(1.0);

            left_motor.SetPercent(-(turn_speed));
            right_motor.SetPercent(turn_speed);
            Sleep(1.0);

            left_motor.SetPercent(backward_speed);
            right_motor.SetPercent(backward_speed);

            if (!back_left && !back_right)
            {
                fifth = false;
                sixth = true;
            }
        }

        else if (sixth) 
        {
            left_motor.SetPercent(forward_speed);
            right_motor.SetPercent(forward_speed);

            left_motor.Stop();
            right_motor.Stop();
        }
    }
}
