#ifndef ROC_TIMER_H
#define ROC_TIMER_H

#include <inttypes.h>
#include <sys/time.h>
#include <iostream>




/*!
 * \brief The rOc_timer class   This timer class is for measuring ellapsed timeval between two instants
 */
class rOc_timer
{
public:

    /*!
     * \brief rOc_timer         Constructor of the class, initialize the timer and start counting time
     */
    rOc_timer();


    /*!
     * \brief restart           Restart the time at zero
     */
    void                    restart();


    /*!
     * \brief ellapsedMicroseconds      Return the ellapsed time in microseconds since last initialization
     *                                  Last initialization may be call of the constructor or the restart function
     * \return                          Elapsed time in microseconds
     */
    uint64_t                ellapsedMicroseconds();


    /*!
     * \brief ellapsedMilliseconds      Return the ellapsed time in milliseconds since last initialization
     *                                  Last initialization may be call of the constructor or the restart function
     * \return                          Elapsed time in milliseconds
     */
    uint64_t                ellapsedMilliseconds();


    /*!
     * \brief ellapsedSeconds           Return the ellapsed time in seconds since last initialization
     *                                  Last initialization may be call of the constructor or the restart function
     * \return                          Elapsed time in seconds
     */
    uint64_t                ellapsedSeconds();



private:

    // Time at initialization
    struct timeval          initialTime;

};

#endif // ROC_TIMER_H
