#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#ifndef _MSC_VER
#include <time.h>
#endif

#ifndef _MSC_VER
/* edited version of timersub for timespec */
#define tsub(a, b, result) \
do { \
(result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
(result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec; \
if ((result)->tv_nsec < 0) { \
--(result)->tv_sec; \
(result)->tv_nsec += 1000000000; \
} \
} while (0)
#endif

struct timer
{
    timer();

    bool start();
    bool stop();
    void reset();

    double interval_S();

    void init();

#ifdef _MSC_VER
    __int64 m_interval;
    __int64 m_frequency;
    __int64 m_startCount;
    __int64 m_stopCount;
    __int64 m_adjustCount;
    bool m_btimerSupported;
    bool m_btimerRunning;
#else
    struct timespec start_t;
    struct timespec stop_t;
#endif
};

#endif
