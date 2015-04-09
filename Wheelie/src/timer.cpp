#include <cmath>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include "timer.hpp"

#ifdef _MSC_VER
timer::timer()
{
    this->init();
}

inline bool timer::start()
{
    bool bSuccess = false;

    if(!m_btimerRunning && m_btimerSupported)
    {
        m_startCount = 0;
        m_stopCount = 0;
        m_interval = 0;

        if(::QueryPerformanceCounter((LARGE_INTEGER*)&m_startCount))
        {
            m_btimerRunning = true;
            bSuccess = true;
        }
    }

    return bSuccess;
}

inline bool timer::stop()
{
    bool bSuccess = false;

    if(m_btimerRunning && m_btimerSupported)
    {
        if(::QueryPerformanceCounter((LARGE_INTEGER*)&m_stopCount))
        {
            m_btimerRunning = false;
            bSuccess = true;
        }
    }

    return bSuccess;
}

void timer::reset()
{
    this->init();
}

double timer::interval_S()
{
    return ((double)(m_stopCount - m_startCount) - m_adjustCount) / (double)m_frequency;
}

void timer::init()
{
    m_frequency = 0;
    m_adjustCount = 0;
    m_btimerSupported = false;
    m_btimerRunning = false;

    if(::QueryPerformanceFrequency((LARGE_INTEGER*)&m_frequency))
    {
        m_btimerSupported = true;

        // Measure the 'Stop' function call overhead
        const int iNumSamples = 10;
        __int64 samples[iNumSamples];
        __int64 countTot = 0;
        double dAvCount = 0.0;
        double dAvDeviance = 0.0;

        for(int i = 0; i < iNumSamples; i++)
        {
            this->start();
            this->stop();

            samples[i] = m_stopCount - m_startCount;
            countTot += samples[i];
        }

        dAvCount = (double)countTot / (double)iNumSamples;

        // Get the average deviance
        for(int i = 0; i < iNumSamples; i++)
        {
            dAvDeviance += fabs(((double)samples[i]) - dAvCount);
        }

        // Average deviance only required for debug
        dAvDeviance /= iNumSamples;
        m_adjustCount = (__int64)dAvCount;
    }
}

#else
timer::timer()
{
}

bool timer::start()
{
    clock_gettime(CLOCK_REALTIME, &start_t);
    return true;
}

bool timer::stop()
{
    clock_gettime(CLOCK_REALTIME, &stop_t);
    return true;
}

void timer::reset()
{
}

double timer::interval_S()
{
    timespec td;
    tsub(&stop_t, &start_t, &td);
    return ((unsigned long)td.tv_sec  + 1e-9*td.tv_nsec);
}

void timer::init()
{
}
#endif
