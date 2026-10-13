#include "pid.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
    PID_t a, b;
    PID_Init(&a, 2.0f, 0.0f, 0.0f, 10.0f);
    assert(PID_Calculate(&a, 3.0f) == 6.0f);
    assert(a.error == 3.0f && a.last_error == 3.0f && a.output == 6.0f);
    assert(PID_Calculate(&a, 100.0f) == 10.0f);
    assert(PID_Calculate(&a, -100.0f) == -10.0f);
    PID_Init(&b, 0.0f, 0.0f, 3.0f, 100.0f);
    assert(PID_Calculate(&b, 4.0f) == 12.0f);
    assert(PID_Calculate(&b, 1.0f) == -9.0f);
    assert(a.output == -10.0f && a.last_error == -100.0f);
    puts("PASS discrete P/D, signed output limits and independent controller states");

    PID_Init(&a, 2.0f, 1.0f, 0.0f, 10.0f);
    for (unsigned i = 0; i < 1000; ++i) { assert(PID_Calculate(&a, 10.0f) == 10.0f); }
    assert(a.integral == 0.0f);
    PID_Init(&a, 0.0f, 1.0f, 0.0f, 10.0f);
    assert(PID_Calculate(&a, 4.0f) == 4.0f);
    assert(PID_Calculate(&a, 4.0f) == 8.0f);
    assert(PID_Calculate(&a, 4.0f) == 10.0f);
    assert(PID_Calculate(&a, -2.0f) == 8.0f);
    PID_Init(&a, 1.0f, 2.0f, 0.0f, 100.0f);
    assert(a.error == 0.0f && a.integral == 0.0f && a.last_error == 0.0f);
    assert(PID_Calculate(&a, 3.0f) == 9.0f);
    puts("PASS integral clamp, saturation anti-windup, unwind and reinitialization");

    assert(PID_Calculate(&a, NAN) == 0.0f && a.integral == 0.0f && a.error == 0.0f);
    assert(PID_Calculate(&a, INFINITY) == 0.0f);
    PID_Init(&a, FLT_MAX, 0.0f, 0.0f, 100.0f);
    assert(PID_Calculate(&a, 2.0f) == 0.0f && a.last_error == 0.0f);
    PID_Init(&a, NAN, 1.0f, 1.0f, 100.0f);
    assert(PID_Calculate(&a, 2.0f) == 0.0f);
    PID_Init(&a, 1.0f, 1.0f, 1.0f, -10.0f);
    assert(PID_Calculate(&a, 2.0f) == 0.0f);
    PID_Init(NULL, 1.0f, 1.0f, 1.0f, 10.0f);
    assert(PID_Calculate(NULL, 2.0f) == 0.0f);
    puts("PASS invalid inputs, arithmetic overflow and null guards");

    PID_Position timed;
    PID_Position_Init(&timed, 0.0f, 2.0f, 0.0f, 100.0f, 100.0f);
    PID_Init(&a, 0.0f, 0.02f, 0.0f, 100.0f);
    for (unsigned i = 0; i < 100; ++i)
    {
        assert(fabsf(PID_Calculate(&a, 3.0f) -
            PID_Position_Calculate(&timed, 3.0f, 0.01f)) < 0.0001f);
    }
    PID_Position_Init(&timed, 0.0f, 0.0f, 1.0f, 100.0f, 1000.0f);
    assert(PID_Position_Calculate(&timed, 3.0f, 0.01f) == 0.0f);
    assert(fabsf(PID_Position_Calculate(&timed, 4.0f, 0.01f) - 100.0f) < 0.001f);
    assert(PID_Position_Calculate(&timed, 4.0f, 0.0f) == 0.0f);
    assert(timed.integral == 0.0f && !timed.has_previous_error);
    puts("PASS sample-gain conversion and existing time-based PID behavior");
    return 0;
}
