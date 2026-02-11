#include <FEH.h>
#include <Arduino.h>
#include "../private_include/FEHInternal.h"

// Millisecond sleeps
void Sleep(unsigned long ms) { delay(ms); }
void Sleep(int ms) { delay(ms); }

// Second sleeps
void Sleep(double s) { delay(s * 1000); }
void Sleep(float s) { delay(s * 1000); }

// Microsecond sleeps
void SleepMicroseconds(unsigned long us) { delayMicroseconds(us); }
void SleepMicroseconds(int us) { delayMicroseconds(us); }

// Battery voltage
float BatteryVoltage() { return _batteryVoltage(); }

// Wrapper for millis() but in seconds
float TimeNow() { return millis() / 1000.0; }

// Random number generation
int RandInt(int min, int max)
{
    return random(min, max);
}

float RandFloat(float min, float max)
{
    return rand() / (float)RAND_MAX * (max - min) + min;
}


float DummyBattery::Voltage(){
    return BatteryVoltage();
}

// Battery singleton
DummyBattery Battery;