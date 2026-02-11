/**
 * FEHUtility.hpp
 *
 * Header-only utility functions
 *
 * @author Adam Exley <exley.8@osu.edu>
 */

#ifndef FEHUTILITY_H
#define FEHUTILITY_H

#include <FEHTestGUI.h>

// Millisecond sleeps
void Sleep(unsigned long ms);
void Sleep(int ms);

// Second sleeps
void Sleep(double s);
void Sleep(float s);

// Microsecond sleeps
void SleepMicroseconds(unsigned long us);
void SleepMicroseconds(int us);

// Battery voltage
float BatteryVoltage();

// Just for backwards compatibility
class DummyBattery
{
public:
    float Voltage();
};

extern DummyBattery Battery;




// Wrapper for millis() but in seconds
float TimeNow();

// Random number generation
int RandInt(int min, int max);
float RandFloat(float min = 0.0f, float max = 1.0f);

#endif // FEHUTILITY_H