/*
 * test_scheduler.cpp
 *
 * Unit tests for the Timer 4 scheduler.
 *
 * Authors:
 * - Brian Jia <jia.659@osu.edu>
 */

#include <Arduino.h>
#include <unity.h>
#include <FEH.h>
#include "../private_include/scheduler.h"

#define ACCEPTABLE_MICROS_ERROR 64
#define ACCEPTABLE_MICROS_ERROR_BOTH 128

int32_t ticks_to_micros(int32_t ticks)
{
    // 64 micros per tick
    return ticks * 64L;
}

int32_t micros_to_ticks(int32_t micros)
{
    // 64 micros per tick
    return micros / 64L;
}

static volatile int32_t time_start = 0;
static volatile int32_t time_elapsed1 = 0;
static volatile int32_t time_elapsed2 = 0;
static volatile int32_t time_elapsed3 = 0;
static volatile bool completed1 = false;
static volatile bool completed2 = false;
static volatile bool completed3 = false;

void setUp(void)
{
    time_start = micros();
    time_elapsed1 = 0;
    time_elapsed2 = 0;
    time_elapsed3 = 0;
    completed1 = false;
    completed2 = false;
    completed3 = false;
}

void tearDown(void)
{

}

void callback1(void)
{
    time_elapsed1 = micros() - time_start;
    completed1 = true;
}

void callback2(void)
{
    time_elapsed2 = micros() - time_start;
    completed2 = true;
}

void callback3(void)
{
    time_elapsed3 = micros() - time_start;
    completed3 = true;
}

void callback_schedules_2_in_5000_ticks(void)
{
    scheduleEvent(callback2, 5000);
}

void wait1(void)
{
    while (!completed1)
    {
    }
}

void wait_both(void)
{
    while (!completed1 || !completed2)
    {
    }
}

void test_1_event(uint16_t ticks)
{
    scheduleEvent(callback1, ticks);
    wait1();
    TEST_ASSERT_INT32_WITHIN(ACCEPTABLE_MICROS_ERROR, ticks_to_micros(ticks), time_elapsed1);
}

void assert_both(int32_t ticks1, int32_t ticks2)
{
    wait_both();
    TEST_ASSERT_INT32_WITHIN(ACCEPTABLE_MICROS_ERROR_BOTH, ticks_to_micros(ticks1), time_elapsed1);
    TEST_ASSERT_INT32_WITHIN(ACCEPTABLE_MICROS_ERROR_BOTH, ticks_to_micros(ticks2), time_elapsed2);
}

void test_1_event_0_ticks(void)
{
    scheduleEvent(callback1, 0);
    wait1();
    /* Expect some delay when we schedule 0-tick */
    TEST_ASSERT_INT32_WITHIN(ACCEPTABLE_MICROS_ERROR, 100, time_elapsed1);
}

void test_1_event_1_tick(void)
{
    test_1_event(1);
}

void test_1_event_1000_ticks(void)
{
    test_1_event(7813);
}

void test_1_event_65535_ticks(void)
{
    test_1_event(65535);
}

void test_2_events_0_tick(void)
{
    scheduleEvent(callback1, 0);
    scheduleEvent(callback2, 0);
    /* Expect some delay when we schedule 0-tick */
    assert_both(1, 1);
}

void test_2_events_1_tick(void)
{
    scheduleEvent(callback1, 1);
    scheduleEvent(callback2, 1);
    assert_both(1, 1);
}

void test_2_events_1000_tick(void)
{
    scheduleEvent(callback1, 1000);
    scheduleEvent(callback2, 1000);
    assert_both(1000, 1000);
}

void test_2_events_different_ticks_1(void)
{
    scheduleEvent(callback1, 1000);
    scheduleEvent(callback2, 1500);
    assert_both(1000, 1500);
}

void test_2_events_different_ticks_2(void)
{
    scheduleEvent(callback1, 1500);
    scheduleEvent(callback2, 1000);
    assert_both(1500, 1000);
}

void test_2_events_65535_ticks(void)
{
    scheduleEvent(callback1, 65535);
    scheduleEvent(callback2, 65535);
    assert_both(65535, 65535);
}

void test_2_events_one_delayed_start(void)
{
    scheduleEvent(callback1, 15000);
    /* Adds up to 5000 ticks in total */
    delay(320);
    scheduleEvent(callback2, 5000);
    /* Expect some delay for this more complicated one */
    assert_both(15001, 10000);
}

void test_2_events_nested_scheduling(void)
{
    scheduleEvent(callback1, 10000);
    scheduleEvent(callback_schedules_2_in_5000_ticks, 5000);
    /* Expect some delay for this more complicated one */
    assert_both(10001, 10001);
}

void test_cancel_1_event(void)
{
    scheduleEvent(callback1, schedulerMsToTicks(100));
    delay(50);
    cancelEvents(callback1);
    delay(100);
    TEST_ASSERT_FALSE(completed1);
}

void test_cancel_1_event_with_1_not_cancelled_1(void)
{
    scheduleEvent(callback1, schedulerMsToTicks(100));
    scheduleEvent(callback2, schedulerMsToTicks(100));
    delay(50);
    cancelEvents(callback1);
    delay(100);
    TEST_ASSERT_FALSE(completed1);
    TEST_ASSERT_TRUE(completed2);
}

void test_cancel_1_event_with_1_not_cancelled_2(void)
{
    scheduleEvent(callback1, schedulerMsToTicks(100));
    scheduleEvent(callback2, schedulerMsToTicks(100));
    delay(50);
    cancelEvents(callback2);
    delay(100);
    TEST_ASSERT_TRUE(completed1);
    TEST_ASSERT_FALSE(completed2);
}

void test_cancel_1_event_with_2_not_cancelled(void)
{
    scheduleEvent(callback1, schedulerMsToTicks(100));
    scheduleEvent(callback2, schedulerMsToTicks(150));
    scheduleEvent(callback3, schedulerMsToTicks(200));
    delay(50);
    cancelEvents(callback2);
    delay(250);
    TEST_ASSERT_TRUE(completed1);
    TEST_ASSERT_FALSE(completed2);
    TEST_ASSERT_TRUE(completed3);
}

void test_cancel_2_events(void)
{
    scheduleEvent(callback1, schedulerMsToTicks(100));
    scheduleEvent(callback2, schedulerMsToTicks(100));
    delay(50);
    cancelEvents(callback1);
    cancelEvents(callback2);
    delay(100);
    TEST_ASSERT_FALSE(completed1);
    TEST_ASSERT_FALSE(completed2);
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN(); // IMPORTANT LINE!

    RUN_TEST(test_1_event_0_ticks);
    RUN_TEST(test_1_event_1_tick);
    RUN_TEST(test_1_event_1000_ticks);
    RUN_TEST(test_1_event_65535_ticks);
    RUN_TEST(test_2_events_0_tick);
    RUN_TEST(test_2_events_1_tick);
    RUN_TEST(test_2_events_1000_tick);
    RUN_TEST(test_2_events_different_ticks_1);
    RUN_TEST(test_2_events_different_ticks_2);
    RUN_TEST(test_2_events_65535_ticks);
    RUN_TEST(test_2_events_one_delayed_start);
    RUN_TEST(test_2_events_nested_scheduling);

    RUN_TEST(test_cancel_1_event);
    RUN_TEST(test_cancel_1_event_with_1_not_cancelled_1);
    RUN_TEST(test_cancel_1_event_with_1_not_cancelled_2);
    RUN_TEST(test_cancel_1_event_with_2_not_cancelled);
    RUN_TEST(test_cancel_2_events);
}

void loop()
{
    UNITY_END();
}