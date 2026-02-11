/**
 * FEHBuzzer.cpp
 *
 * FEH buzzer library.
 *
 * Most of this is ripped from the Tone.h library, with unnecessary code removed.
 *
 * It has also been modified to use the Timer 4 scheduler to keep track of when to stop
 * the timer, instead of keeping track of time inside the buzzer timer ISR.
 *
 * Tone.h Attribution
 * Author: Brett Hagman <bhagman@wiring.org.co>
 * Contributor: Fotis Papadopoulos <fpapadopou@gmail.com>
 * License: MIT
 *
 * @author Brian Jia <jia.659@osu.edu>
 */

#include <FEH.h>
#include "../private_include/scheduler.h"
#include <Arduino.h>

#define BUZZER_PORT PORTG
#define BUZZER_DDR DDRG
#define BUZZER_MASK bit(5)

/* Callback for scheduler */
static void buzzerOff()
{
    Buzzer.Off();
}

void FEHBuzzer::Tone(int frequency)
{
    cancelEvents(buzzerOff);

    /* Set up Timer 2 */
    // 8 bit timer
    TCCR2A = bit(WGM21);
    TCCR2B = bit(CS20);

    uint8_t prescalerBits = 0b001;
    uint32_t ocr = 0;

    // Set the pinMode as OUTPUT
    BUZZER_DDR |= BUZZER_MASK;

    ocr = F_CPU / frequency / 2 - 1;
    prescalerBits = 0b001; // ck/1: same for both timers
    if (ocr > 255)
    {
        ocr = F_CPU / frequency / 2 / 8 - 1;
        prescalerBits = 0b010; // ck/8: same for both timers

        if (ocr > 255)
        {
            ocr = F_CPU / frequency / 2 / 32 - 1;
            prescalerBits = 0b011;
        }

        if (ocr > 255)
        {
            ocr = F_CPU / frequency / 2 / 64 - 1;
            prescalerBits = 0b100;

            if (ocr > 255)
            {
                ocr = F_CPU / frequency / 2 / 128 - 1;
                prescalerBits = 0b101;
            }

            if (ocr > 255)
            {
                ocr = F_CPU / frequency / 2 / 256 - 1;
                prescalerBits = 0b110;
                if (ocr > 255)
                {
                    // can't do any better than /1024
                    ocr = F_CPU / frequency / 2 / 1024 - 1;
                    prescalerBits = 0b111;
                }
            }
        }
    }

    TCCR2B = (TCCR2B & 0b11111000) | prescalerBits;

    OCR2A = ocr;
    TIMSK2 = bit(OCIE2A);
}

void FEHBuzzer::Beep()
{
    this->Tone(1000, 500);
}

void FEHBuzzer::Buzz()
{
    this->Tone(1000);
}

void FEHBuzzer::Buzz(int milliseconds)
{
    this->Tone(1000, milliseconds);
}

void FEHBuzzer::Tone(int frequency, int milliseconds)
{
    /* Scheduler doesn't support more than 4194 milliseconds or 65535 ticks */
    if (milliseconds > 4194)
    {
        milliseconds = 4194;
    }
    /* Handle the other silly goofy case */
    else if (milliseconds < 0)
    {
        return;
    }

    /* Tone() cancels all buzzerOff events so schedule AFTER */
    this->Tone(frequency);
    scheduleEvent(buzzerOff, schedulerMsToTicks(milliseconds));
}

void FEHBuzzer::Off()
{
    cancelEvents(buzzerOff);

    /* Disable interrupts for that timer*/
    TIMSK2 &= ~(1 << OCIE2A);
    /* Turn off the buzzer pin */
    BUZZER_PORT &= ~(BUZZER_MASK);
}

/* ISR for Buzzer Timer */
ISR(TIMER2_COMPA_vect)
{
    /* Toggle the buzzer timer pin */
    BUZZER_PORT ^= BUZZER_MASK;
}