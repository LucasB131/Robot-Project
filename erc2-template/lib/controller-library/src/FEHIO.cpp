/**
 * FEHIO.cpp
 *
 * FEH I/O library.
 *
 * @authors Brian Jia, Adam Exley, Olivia Maynard
 */

#include <FEH.h>
#include "../private_include/FEHInternal.h"
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

DigitalInputPin::DigitalInputPin(FEHIO::FEHIOPin pin, bool usePullup)
{
    if ((uint8_t)pin > 15)
    {
        _fatalError("DigitalInputPin:\npin out of range");
    }

    _arduinoPin = pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pin);
    pinMode(_arduinoPin, usePullup ? INPUT_PULLUP : INPUT);
}

bool DigitalInputPin::Value()
{
    return digitalRead(_arduinoPin);
}

DigitalOutputPin::DigitalOutputPin(FEHIO::FEHIOPin pin)
{
    if ((uint8_t)pin > 15)
    {
        _fatalError("DigitalOutputPin: \npin out of range");
    }
   _arduinoPin = pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pin);
   pinMode(_arduinoPin, OUTPUT);
}

void DigitalOutputPin::Write(bool value)
{
    digitalWrite(_arduinoPin, value);
}

bool DigitalOutputPin::Status()
{ 
    return digitalRead(_arduinoPin);
}

void DigitalOutputPin::Toggle()
{
    bool toggled = !digitalRead(_arduinoPin);
    digitalWrite(_arduinoPin, toggled);
}

AnalogInputPin::AnalogInputPin(FEHIO::FEHIOPin pin, bool usePullup)
{
    if ((uint8_t)pin > 15)
    {
        _fatalError("AnalogInputPin:\npin out of range");
    }

    if (!pgm_read_byte(FEHIOPIN_VALID_ANALOG_PINS + pin))
    {
        char msg[128];
        snprintf(
            msg, 128,
            "AnalogInputPin:\n"
            "\n"
            "Attemped to use\n"
            "non-analog pin %d.\n"
            "\n"
            "Valid analog pins are:\n"
            "0-14.\n",
            pin);

        _fatalError(msg);
    }

    _arduinoPin = pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pin);
    pinMode(_arduinoPin, usePullup ? INPUT_PULLUP : INPUT);
}

float AnalogInputPin::Value()
{
    /* Arduino ADC is 10-bit by default */
    return analogRead(_arduinoPin) * (5.0 / 1023.0);
}

DigitalQuadratureEncoder::DigitalQuadratureEncoder(FEHIO::FEHIOPin pinA, FEHIO::FEHIOPin pinB)
{
    if ((uint8_t)pinA > 15 || (uint8_t)pinB > 15)
    {
        _fatalError("DigitalQuadratureEncoder:\npin out of range");
    }

    // Throw a fatal error if the pins are not valid interrupt pins
    if (!pgm_read_byte(FEHIOPIN_VALID_INTERRUPT_PINS + pinA) ||
        !pgm_read_byte(FEHIOPIN_VALID_INTERRUPT_PINS + pinB))
    {
        char msg[128];
        snprintf(
            msg, 128,
            "DigitalQuadratureEncoder:\n"
            "\n"
            "Attemped to use\n"
            "non-interrupt pin %d or %d.\n"
            "\n"
            "Valid interrupt pins are:\n"
            "8-14.\n",
            pinA,
            pinB);

        _fatalError(msg);
    }

    pinMode(pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pinA), INPUT_PULLUP);
    pinMode(pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pinB), INPUT_PULLUP);

    _encoder = new Encoder(
        pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pinA),
        pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pinB));

    _encoder->write(0);
}

int DigitalQuadratureEncoder::Counts()
{
    return _encoder->read();
}

void DigitalQuadratureEncoder::ResetCounts()
{
    _encoder->write(0);
}

// User-facing "Counts" variables
volatile unsigned long digital_encoder_counts[7] = {0};
bool digital_encoder_enabled[7] = {false};

// ISR variables
uint8_t _encoder_isr_mask = 0;
volatile uint8_t _portK_last_state = 0;
uint8_t _pinchange;

DigitalEncoder::DigitalEncoder(FEHIO::FEHIOPin pin)
{
    if ((uint8_t)pin > 15)
    {
        _fatalError("DigitalEncoder:\npin out of range");
    }

    // Throw a fatal error if the pin is not a valid interrupt pin
    if (!pgm_read_byte(FEHIOPIN_VALID_INTERRUPT_PINS + pin))
    {
        char msg[128];
        snprintf(
            msg, 128,
            "DigitalEncoder:\n"
            "\n"
            "Attemped to use\n"
            "non-interrupt pin %d.\n"
            "\n"
            "Valid interrupt pins are:\n"
            "8-14.\n",
            pin);

        _fatalError(msg);
    }

    // Get the pin number on port K
    _portKpin = pin - 8;

    // Enable pullup
    _arduinoPin = pgm_read_byte(FEHIOPIN_TO_ARDUINOPIN + pin);
    pinMode(_arduinoPin, INPUT_PULLUP);

    // All digital encoders are on port K
    // This corresponds to PCINT2 (PCINT16:23)
    // Disable global interrupts
    cli();

    // set the mask for the pin change interrupt
    _encoder_isr_mask |= (1 << _portKpin);

    // Enable pin change interrupt
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << _portKpin);

    // Get state of port K pins
    _portK_last_state = PINK;

    // Enable global interrupts
    sei();
}

// ISR for all digital encoders
ISR(PCINT2_vect)
{
    // Find the pins that have changed within the mask
    _pinchange = (PINK ^ _portK_last_state) & _encoder_isr_mask;

    // store current state of port K
    _portK_last_state = PINK;

    // Add to the count if the pin has changed
    for (uint8_t i = 0; i < 8; i++)
    {
        if (_pinchange & (1 << i))
        {
            digital_encoder_counts[i]++;
        }
    }
}

int DigitalEncoder::Counts()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return digital_encoder_counts[_portKpin];
    }
}

void DigitalEncoder::ResetCounts()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        digital_encoder_counts[_portKpin] = 0;
    }
}