#include <FEH.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <Arduino.h>

// exact pin locations to change
FEHMotor left_motor(FEHMotor::Motor1, 9.0);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);

AnalogInputPin right_opto(FEHIO::Pin11);
AnalogInputPin middle_opto(FEHIO::Pin12);
AnalogInputPin left_opto(FEHIO::Pin13);

DigitalEncoder right_encoder(FEHIO::Pin14);
DigitalEncoder left_encoder(FEHIO::Pin15);

// Global Constants
#define PI 3.141592653
#define SENSOR_THRESHOLD 2.5

#define WHEEL_RADIUS 1.0 // tbd
#define IGWAN_TRANASITIONS 318.

void ERCMain()
{
    TestGUI();
}

// Follows the line
// (to change?): until all three sensors are off
void followLine() {
    bool left, middle, right; // false is off, true is on
    float goFasterSpeed = 30, goSpeed = 20, slowSpeed = 12;

    while (true) {
        left = left_opto.Value() > SENSOR_THRESHOLD;
        middle = middle_opto.Value() > SENSOR_THRESHOLD;
        right = right_opto.Value() > SENSOR_THRESHOLD;

        if (left && middle && !right) {
            // right is off, turn left
            left_motor.SetPercent(goSpeed);
            right_motor.SetPercent(goFasterSpeed);
        }   
        if (left && !middle && !right) {
            // middle and right are off, turn left
            left_motor.SetPercent(slowSpeed);
            right_motor.SetPercent(goFasterSpeed);
        }
        if (!left && middle && right) {
            // left is off, turn right
            left_motor.SetPercent(goFasterSpeed);
            right_motor.SetPercent(goSpeed);
        }
        if (!left && !middle && right) {
            // left and middle are off, turn right
            left_motor.SetPercent(goFasterSpeed);
            right_motor.SetPercent(slowSpeed);
        }
        if (!left && middle && !right) {
            // left and right are off, go straight
            left_motor.SetPercent(goSpeed);
            right_motor.SetPercent(goSpeed);
        }
        if (!left && !middle && !right) {
            // all are off: stop
            left_motor.SetPercent(0);
            right_motor.SetPercent(0);
        }
    }
}

// Goes forward an exact distance
void goForward(float distance) {
    // s = 2(pi)rn/N // n = sN/2(pi)r
    int countsLimit = (distance*IGWAN_TRANASITIONS)/(2*PI*WHEEL_RADIUS);
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    while(((left_encoder.Counts() + right_encoder.Counts()) / 2.) < countsLimit);
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}