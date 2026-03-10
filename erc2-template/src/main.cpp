#include <FEH.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <Arduino.h>
#include <FEHRCS.h>
#include <FEHMotor.h>

// function declarations
void followLine();
void goDistance(bool forward, float distance);
void turnAngle(bool CW, int degrees);

// exact pin locations to change
FEHMotor left_motor(FEHMotor::Motor2, 9.0);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);

AnalogInputPin cds(FEHIO::Pin0);

AnalogInputPin right_opto(FEHIO::Pin11);
AnalogInputPin middle_opto(FEHIO::Pin12);
AnalogInputPin left_opto(FEHIO::Pin13);

DigitalEncoder right_encoder(FEHIO::Pin12);
DigitalEncoder left_encoder(FEHIO::Pin14);

// Global Constants
#define PI 3.141592653
#define SENSOR_THRESHOLD 2.5

#define WHEEL_RADIUS 1.25 // inches
#define ROBOT_RADIUS 8.10/2.0 // inches
#define IGWAN_TRANSITIONS 318.

#define GO_SPEED_RIGHT 40
#define GO_SPEED_LEFT 40
#define BACK_SPEED_RIGHT -40
#define BACK_SPEED_LEFT -40

#define STARTLIGHT_THRESHOLD 3.0
#define RED_THRESHOLD 0.6

void ERCMain() {
    right_motor.SetPercent(GO_SPEED_RIGHT);
    left_motor.SetPercent(GO_SPEED_LEFT);
}


void Milestone3()
{
    // Wait for CDS value to read start light activation
    while (cds.Value() > STARTLIGHT_THRESHOLD) {};

    // Go into and out of the button (figure out exact distances)
    goDistance(true, 4.0);
    goDistance(false, 4.0);

    // Turn x degrees to face the line up the ramp (figure out exact angle to turn)
    turnAngle(false, 135);

    // Go x distance to reach the line
    goDistance(true, 30.0);

    // Follow the line to the humidifer buttons
    followLine();

    // Move a little forward x inches to read the CDS cell
    goDistance(true, 4.0);

    // Read CDS cell (consider getting the red filter) (no filter: blue ~0.25 red ~0.99)
    bool red = false;
    if (cds.Value() < RED_THRESHOLD) {
        bool red = true;
    }
    
    // Hit button according to which light turned on
    if (red) {
        // turn and go to hit red
    } else {
        // turn and go to hit blue
    }
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
        left_motor.SetPercent(GO_SPEED_LEFT);
        right_motor.SetPercent(GO_SPEED_RIGHT);
    } else {
        left_motor.SetPercent(BACK_SPEED_LEFT);
        right_motor.SetPercent(BACK_SPEED_RIGHT);
    }
    while(((left_encoder.Counts() + right_encoder.Counts()) / 2.) < countsLimit);
    left_motor.SetPercent(0);
    right_motor.SetPercent(0);
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
        left_motor.SetPercent(GO_SPEED_LEFT);
        right_motor.SetPercent(BACK_SPEED_RIGHT);
    } else {
        left_motor.SetPercent(BACK_SPEED_LEFT);
        right_motor.SetPercent(GO_SPEED_RIGHT);
    }
    while(((left_encoder.Counts() + right_encoder.Counts()) / 2.) < countsLimit);
    left_motor.SetPercent(0);
    right_motor.SetPercent(0);
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}
