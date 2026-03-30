#include <FEH.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEHUtility.h>
#include <Arduino.h>
#include <FEHRCS.h>
#include <FEHMotor.h>

// function declarations
void followLine(double secs);
void goDistance(bool forward, float distance);
void turnAngle(bool CW, int degrees);
void Milestone2();
void Milestone3();

// exact pin locations to change
FEHMotor left_motor(FEHMotor::Motor2, 9.0);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);

FEHServo vertical(FEHServo::Servo1);
FEHServo horizontal(FEHServo::Servo0);

AnalogInputPin cds(FEHIO::Pin8);

AnalogInputPin right_opto(FEHIO::Pin12);
AnalogInputPin middle_opto(FEHIO::Pin13);
AnalogInputPin left_opto(FEHIO::Pin14);

DigitalEncoder right_encoder(FEHIO::Pin9);
DigitalEncoder left_encoder(FEHIO::Pin10);

// Global Constants
#define PI 3.141592653

#define WHEEL_RADIUS 1.25 // inches
#define ROBOT_RADIUS 8.10/2.0 // inches
#define IGWAN_TRANSITIONS 318.

#define GO_SPEED_RIGHT 35
#define GO_SPEED_LEFT 35
#define BACK_SPEED_RIGHT -35
#define BACK_SPEED_LEFT -35

#define OPTO_THRESHOLD 4.2
#define STARTLIGHT_THRESHOLD 2.5
#define RED_THRESHOLD 0.9

void ERCMain() {
    TestGUI();
}

void Milestone3() 
{
    // Wait for CDS value to read start light activation
    while (cds.Value() > STARTLIGHT_THRESHOLD) {};

    vertical.SetDegree(20);
    horizontal.SetDegree(180);
    // Go into and out of the button (figure out exact distances)
    goDistance(false, 3.0);
    Sleep(0.2);
    goDistance(true, 4.0);
    Sleep(0.2);

    // Turn x degrees to face the line up the ramp (figure out exact angle to turn) and go
    turnAngle(true, 55);
    Sleep(0.2);
    goDistance(true, 34.0);
    Sleep(0.2);
    
    // turn parallel to window and go
    turnAngle(false, 90);
    goDistance(true, 13.5);

    // arm wrap around
    goDistance(false, 11.0);
    horizontal.SetDegree(0);
    goDistance(true, 11.0);
    horizontal.SetDegree(180);
    
    // pull window closed
    goDistance(false, 20.0);
}

void Milestone2()
{
    // Wait for CDS value to read start light activation
    while (cds.Value() > STARTLIGHT_THRESHOLD) {};

    // Go into and out of the button (figure out exact distances)
    goDistance(false, 3.0);
    Sleep(0.2);
    goDistance(true, 4.0);
    Sleep(0.2);

    // Turn x degrees to face the line up the ramp (figure out exact angle to turn)
    turnAngle(true, 50);
    Sleep(0.2);

    // Go x distance to reach the line
    goDistance(true, 31.0);
    Sleep(0.2);

    // Follow the line to the humidifer buttons
    followLine(5.0);
    turnAngle(false, 15);

    // Move a little forward x inches to read the CDS cell
    goDistance(true, 7.0);
    Sleep(0.2);

    // Read CDS cell (consider getting the red filter) (no filter: blue ~0.25 red ~0.99)
    bool red = cds.Value() < RED_THRESHOLD;
    
    // Hit button according to which light turned on
    if (red) {
        LCD.Write("red");
        // turn and go to hit red on right
        turnAngle(true, 90);
        Sleep(0.2);
        goDistance(true, 3.0);
        Sleep(0.2);
        turnAngle(false, 90);
        Sleep(0.2);

        goDistance(true, 7.0);

        // back into wall
        goDistance(false, 33.0);
        goDistance(true, 3.5);
        turnAngle(false, 90);
        goDistance(true, 42.0);
        right_motor.SetPercent(20);
    } else {
        LCD.Write("blue");
        // turn and go to hit blue on left
        turnAngle(false, 90);
        Sleep(0.2);
        goDistance(true, 3.0);
        Sleep(0.2);
        turnAngle(true, 90);
        Sleep(0.2);
        goDistance(true, 7.0);

        // back into wall
        goDistance(false, 33.0);
        goDistance(true, 2.0);
        turnAngle(false, 90);
        goDistance(true, 42.0);
        right_motor.SetPercent(20);
    }
}

// Follows the line until all three sensors are on and until x seconds have passed
void followLine(double secs) {
    bool left, middle, right, lastLeft; // false is off, true is on
    float goFasterSpeed = 20, goSpeed = 10, slowSpeed = -20;
    double initialTime = TimeNow(), elapsedTime;

    bool reached = false;
    while (!reached) {
        left = left_opto.Value() > OPTO_THRESHOLD;
        middle = middle_opto.Value() > OPTO_THRESHOLD;
        right = right_opto.Value() > OPTO_THRESHOLD;

        elapsedTime = TimeNow() - initialTime;

        if (left && middle && !right) {
            // right is off, turn left
            left_motor.SetPercent(goSpeed);
            right_motor.SetPercent(goFasterSpeed);
        }   
        if (left && !middle && !right) {
            // middle and right are off, turn left
            left_motor.SetPercent(slowSpeed);
            right_motor.SetPercent(goFasterSpeed);
            lastLeft = true;
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
            lastLeft = false;
        }
        if (!left && middle && !right) {
            // left and right are off, go straight
            left_motor.SetPercent(goSpeed);
            right_motor.SetPercent(goSpeed);
        }
        if (!left && !middle && !right) {
            // all are off: error
            if (lastLeft) {
                left_motor.SetPercent(slowSpeed);
                right_motor.SetPercent(goFasterSpeed);
            } else {
                right_motor.SetPercent(slowSpeed);
                left_motor.SetPercent(goFasterSpeed);
            }
        }
        if (left && middle && right && (elapsedTime > secs)) {
            // all on: success and finish
            left_motor.SetPercent(0);
            right_motor.SetPercent(0);
            reached = true;
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

/* * * * * * * * */
/* RCS Functions */
/* * * * * * * * */

// RCS Pulse Values
#define RCS_WAIT_TIME_IN_SEC 0.35
#define PULSE_TIME 0.1
#define PULSE_POWER 50

// Orientation of AruCo Code
#define PLUS 0
#define MINUS 1

/*
 * Pulse forward or backward a short distance using time
 */
void pulse_forward(bool forward, float seconds) 
{
    // Set both motors to desired percent
    if (forward) {
        right_motor.SetPercent(GO_SPEED_RIGHT);
        left_motor.SetPercent(GO_SPEED_LEFT);
    } else {
        right_motor.SetPercent(GO_SPEED_RIGHT);
        left_motor.SetPercent(GO_SPEED_LEFT);
    }

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/*
 * Pulse a short rotation using time
 */
void pulse_rotation(bool cw, float seconds)
{
    // Set both motors to desired percent
    if (cw) {
        right_motor.SetPercent(BACK_SPEED_RIGHT);
        left_motor.SetPercent(GO_SPEED_LEFT);
    } else {
        right_motor.SetPercent(GO_SPEED_RIGHT);
        left_motor.SetPercent(BACK_SPEED_LEFT);
    }

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

/* 
 * Use RCS to move to the desired x_coordinate based on the orientation of the AruCo code
 */
void check_x(float x_coordinate, int orientation)
{
    RCSPose* pose = RCS.RequestPosition();

    // Check if receiving proper RCS coordinates and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++) {
        if(pose->x >= 0 && (pose->x < x_coordinate - 1 || pose->x > x_coordinate + 1))
        {
            if (pose->x > x_coordinate + 1)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(false, PULSE_TIME);
            }
            else if(pose->x < x_coordinate - 1)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(true, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
}
}


/* 
 * Use RCS to move to the desired y_coordinate based on the orientation of the QR code
 */
void check_y(float y_coordinate, int orientation)
{
    RCSPose* pose = RCS.RequestPosition();

    for (int i = 0; i < 10; i++) {
        while(pose->y >= 0 && (pose->y < y_coordinate - 1 || pose->y > y_coordinate + 1))
        {
            if(pose->y > y_coordinate + 1)
            {
                pulse_forward(false, PULSE_TIME);
            }
            else if(pose->y < y_coordinate - 1)
            {
                pulse_forward(true, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);
            
            pose = RCS.RequestPosition();
        }
    }   
}

/* 
 * Use RCS to move to the desired heading
 */
void check_heading(float heading)
{
    RCSPose* pose = RCS.RequestPosition();

    for (int i = 0; i < 10; i++) {
        while(pose->heading >= heading + 1 || pose->heading > heading - 1)
        {
            if(pose->heading > heading + 1)
            {
                pulse_rotation(true, PULSE_TIME);
            }
            else if(pose->heading < heading - 1)
            {
                pulse_forward(false, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);
            
            pose = RCS.RequestPosition();
        }
    }   
}