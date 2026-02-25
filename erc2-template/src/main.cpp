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
#define ROBOT_RADIUS 1.0 // tbd
#define IGWAN_TRANSITIONS 318.

#define GO_SPEED 20
#define BACK_SPEED -20

void ERCMain()
{
    TestGUI();
}

// Follows the line until all three sensors are on
void followLine() {
    bool left, middle, right; // false is off, true is on
    float goFasterSpeed = 40, goSpeed = 20, slowSpeed = 12;

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
            // all are off: error
            // unsure if this condition is necessary to code
            left_motor.SetPercent(0);
            right_motor.SetPercent(0);
        }
        if (left && middle && right) {
            // all on: success and finish
            left_motor.SetPercent(0);
            right_motor.SetPercent(0);
        }
    }
}

// Goes forward or backward an exact distance
void goDistance(bool forward, float distance) {
    // s = 2(pi)rn/N // n = sN/2(pi)r
    int countsLimit = (distance*IGWAN_TRANSITIONS)/(2*PI*WHEEL_RADIUS);
    if (forward) {
        left_motor.SetPercent(GO_SPEED);
        right_motor.SetPercent(GO_SPEED);
    } else {
        left_motor.SetPercent(BACK_SPEED);
        right_motor.SetPercent(BACK_SPEED);
    }
    while(((left_encoder.Counts() + right_encoder.Counts()) / 2.) < countsLimit);
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}

// Turns certain degrees clockwise or counter-clockwise
void turnAngle(bool CW, int degrees) {
    // convert degrees to arc length
    float theta = degrees * (PI/180.);
    float distance = ROBOT_RADIUS*theta;
    int countsLimit = (distance*IGWAN_TRANSITIONS)/(2*PI*WHEEL_RADIUS);
    // move that distance in opposite wheel directions
    if (CW) {
        left_motor.SetPercent(GO_SPEED);
        right_motor.SetPercent(BACK_SPEED);
    } else {
        left_motor.SetPercent(BACK_SPEED);
        right_motor.SetPercent(GO_SPEED);
    }
    while(((left_encoder.Counts() + right_encoder.Counts()) / 2.) < countsLimit);
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}
