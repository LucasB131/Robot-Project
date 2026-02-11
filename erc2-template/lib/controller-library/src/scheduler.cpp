/**
 * scheduler.cpp
 *
 * Internal Scheduler API.
 *
 * An internal Scheduler API for developer convenience.
 * Uses Timer 4 to facilitate scheduling automated health checks, buzzer beeps, and anything else requiring relatively infrequent (< 20Hz) dynamic scheduling.
 *
 * Events are scheduled in 16000000 Hz / 1024 cycle prescaler = 64 microsecond ticks.
 * Because Timer 4 is 16-bit, events can be scheduled a maximum of 65535 ticks or 4194 ms in advance.
 *
 * I do not promise that this scheduler is perfectly accurate, just that it is good enough.
 * Please don't use it for anything where cycle-accuracy is needed.
 * Also, Timer 4 is extremely low on the ISR priority list so be aware.
 *
 * @author Brian Jia <jia.659@osu.edu>
 */

#include <Arduino.h>
#define SCHEDULER_MAX_EVENTS 8

struct EventData
{
    void (*callback)();
    uint16_t ticks;
};

/* Queue of scheduled events, represented as a sorted list in first-to-last order. */
static EventData eventData[SCHEDULER_MAX_EVENTS];
static volatile uint8_t numEventsPending = 0;

static void schedulerTimerDisable()
{
    /* Stop Timer 4 so we can schedule. */
    /* Writing 0 to TCCR4B clears the CS40, CS41, CS42 bits to disconnect */
    /* the timer from its clock. */
    TCCR4A = 0;
    TCCR4B = 0;

    /* Disable Timer 4 interrupts */
    TIMSK4 = 0;

    /* Clear Timer 4 interrupt flags TIFR4 */
    /* INTERRUPT FLAGS ARE WRITE 1 TO ACKNOWLEDGE */
    TIFR4 = 0xFF;
}

/* Sets up Timer 4 to go off once the next event is due. */
static void schedulerTimerSetup()
{
    if (numEventsPending > 0)
    {
        /* There are pending events, let's set the timer back up. */

        /* Make the timer (and its ISR) go off when the next event is due. */
        OCR4A = eventData[0].ticks;

        /* Enable Timer 4 output compare A match interrupt so the ISR is run */
        /* when the timer hits TOP as specified in OCR4A. */
        TIMSK4 = bit(OCIE4A);

        /* Set Waveform Generation Mode (WGM) to CTC (Clear Timer on Compare) mode */
        /* and to use OCR4A as the TOP (maximum counter value.) */
        /* Pretty much means the timer will count up to the value specified in OCR4A */
        /* and reset to 0 once it hits that value. */
        /* Set timer clock to clk_I/O / 1024 (prescaler divides system clock by 1024) */
        /* which gives us our 64 microsecond tick interval. */
        TCCR4A = 0;
        TCCR4B = bit(WGM42) | bit(CS42) | bit(CS40);

        TCNT4 = 0;
    }
}

bool scheduleEvent(void (*callback)(), uint16_t ticks)
{
    /* Compensate setup time by subtracting a ticks */
    if (ticks >= 1)
    {
        ticks -= 1;
    }

    /* You can't schedule 0-tick events. */
    if (ticks == 0)
    {
        ticks = 1;
    }

    /* Prevent UB in case too many events are added */
    if (numEventsPending >= SCHEDULER_MAX_EVENTS)
    {
        return false;
    }

    /* Disable the timer so we can schedule. */
    schedulerTimerDisable();

    /* Subtract time elapsed from other events */
    int8_t i = 0;
    while (i < numEventsPending)
    {
        /*
         * ALWAYS CHECK FOR UNDERFLOW SITUATIONS LIKE THIS IN A SCHEDULER.
         * Theoretically, this should never happen because the scheduler IRQ should go off
         * before TCNT4 goes past any event's tick count, but this is embedded so S*** HAPPENS.
         *
         * Adding these checks fixed some underflow bug that was causing scheduler events to
         * wrap around to max ticks.
         *
         * - Brian
         */
        if (eventData[i].ticks < TCNT4)
        {
            eventData[i].ticks = 0;
        }
        else
        {
            eventData[i].ticks -= TCNT4;
        }
        i++;
    }

    /* Add event to event list. */
    /* This is just your standard "insert item into sorted array" algorithm. */
    EventData e = {callback, ticks};
    i = numEventsPending - 1;
    while (i >= 0 && eventData[i].ticks > ticks)
    {
        eventData[i + 1] = eventData[i];
        i--;
    }
    eventData[i + 1] = e;

    /* Increment event queue size. */
    numEventsPending++;

    /* Set up the timer to go off once the next event is due. */
    schedulerTimerSetup();

    return true;
}

/* Interrupt service routine (ISR) for Timer 4 output compare A match. */
/* Removes events from event list and dispatches them. */
ISR(TIMER4_COMPA_vect)
{
    /* Subtract time elapsed from all events */
    int i = 0;
    while (i < numEventsPending)
    {
        if (eventData[i].ticks < OCR4A)
        {
            eventData[i].ticks = 0;
        }
        else
        {
            eventData[i].ticks -= OCR4A;
        }
        i++;
    }

    /* Remove and dispatch all due events from queue. */
    while (eventData[0].ticks == 0 && numEventsPending > 0)
    {
        EventData e = eventData[0];

        /* Remove first event from list by shifting all other events to the left. */
        int8_t i = 1;
        while (i < numEventsPending)
        {
            eventData[i - 1] = eventData[i];
            i++;
        }

        /* Decrement event queue size. */
        numEventsPending--;

        /* Dispatch the event */
        e.callback();
    }

    /* Set up the timer again to go off once the next event is due. */
    schedulerTimerSetup();
}

/* Cancels all events that have a given callback. */
void cancelEvents(void (*callback)())
{
    /* Disable the timer so we can modify the event list. */
    schedulerTimerDisable();

    /* Subtract time elapsed from other events */
    int8_t i = 0;
    while (i < numEventsPending)
    {
        if (eventData[i].ticks < TCNT4)
        {
            eventData[i].ticks = 0;
        }
        else
        {
            eventData[i].ticks -= TCNT4;
        }
        i++;
    }

    /* Remove all events with given callback from queue. */
    i = 0;
    while (i < numEventsPending)
    {
        /* If the event at index i has the given callback, remove it from the queue. */
        /* After removing the event, if  another event with the given callback takes its place, */
        /* remove it too, without incrementing i. */
        while (i < numEventsPending && eventData[i].callback == callback)
        {
            /* Remove event from list by shifting events to the right to the left. */
            int8_t j = i + 1;
            while (j < numEventsPending)
            {
                eventData[j - 1] = eventData[j];
                j++;
            }

            /* Decrement event queue size. */
            numEventsPending--;
        }

        i++;
    }

    /* Set up the timer again to go off once the next event is due. */
    schedulerTimerSetup();
}

uint16_t schedulerMsToTicks(int milliseconds)
{
    // 64 micros per tick
    return ((uint32_t)milliseconds * 1000L) / 64L;
}