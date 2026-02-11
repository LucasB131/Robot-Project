/**
 * scheduler.h
 * 
 * @author Brian Jia <jia.659@osu.edu>
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#define SCHEDULER_MS_TO_TICKS(ms) ((uint32_t)ms * 1000L / 64L)

bool scheduleEvent(void (*callback)(), uint16_t ticksInFuture);
void cancelEvents(void (*callback)());
uint16_t schedulerMsToTicks(int milliseconds);

#endif // SCHEDULER_H