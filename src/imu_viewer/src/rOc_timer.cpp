#include <rOc_timer.h>



// Constructor of the class, initialize timer at zero
rOc_timer::rOc_timer()
{    
    restart();
}



// Restart the timer (at zero)
void rOc_timer::restart()
{
    // Store current initial time
    gettimeofday(&initialTime, NULL);
}



// Return the ellapsed time in microseconds
uint64_t rOc_timer::ellapsedMicroseconds()
{
    // Get current time
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    // Compute ellapsed seconds and microseconds
    int64_t useconds    =currentTime.tv_usec-initialTime.tv_usec;
    uint64_t seconds    =currentTime.tv_sec-initialTime.tv_sec;

    // Check for particular case
    if (useconds<0)
        return 1000000+useconds+(seconds-1)*1000000;

    // Return ellapsed miscroseconds
    return seconds*1000000+useconds;
}



// Return the ellapsed time in milliseconds
uint64_t rOc_timer::ellapsedMilliseconds()
{
    // Milliseconds = microseconds/1000
    return ellapsedMicroseconds()/1000;
}



// Return the ellapsed time in seconds
uint64_t rOc_timer::ellapsedSeconds()
{
    // Get current time
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    // Compute ellapsed seconds and microseconds
    int64_t useconds    =currentTime.tv_usec-initialTime.tv_usec;
    uint64_t seconds    =currentTime.tv_sec-initialTime.tv_sec;

    // Check for particular case
    if (useconds<0) return seconds-1;

    // Return ellapsed seconds
    return seconds;
}

