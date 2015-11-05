// Note, this function requires compiler option "-lrt" to be set when compiling with gcc

#include "xsens_time.h"
#include <sys/timeb.h>
#include <stdlib.h>
#include "pstdint.h"
#include <stdio.h>

#ifdef _WIN32
#	include <windows.h>
#	ifndef _WCHAR_T_DEFINED
#		include <wchar.h>
#	endif
#else
#	include <unistd.h>
#   include <sys/time.h>
#	include <pthread.h>
#	include <string.h>
#endif
#include <math.h>
#include <float.h>

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Other  functions ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __APPLE__
/*! \brief An implementation of linux' clock_gettime()

   clock_gettime() is not available on Apple/Darwin platforms. This function helps
   maintaining compatibility with Linux code.
 */
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
static int clock_gettime(int clk_id, struct timespec *tp)
{
	struct timeval now;

	int rv = gettimeofday(&now, NULL);
	if (rv != 0)
		return rv;

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief A platform-independent clock.
*/
uint32_t getTimeOfDay(tm* date_, time_t* secs_)
{
#ifdef _WIN32
	static int64_t startTimePerfCount;
	static int64_t startTimeSysTime;
	static int64_t perfCountFreq = 0;

	FILETIME now;
	GetSystemTimeAsFileTime(&now);
	int64_t t = (int64_t) (((((uint64_t) now.dwHighDateTime << 32) | now.dwLowDateTime)/10000) - 11644473600000);

	LARGE_INTEGER pc;
	if (QueryPerformanceCounter(&pc))
	{
		if (!perfCountFreq)
		{
			LARGE_INTEGER tmp;
			QueryPerformanceFrequency(&tmp);
			perfCountFreq = tmp.QuadPart;
			startTimePerfCount = pc.QuadPart;
			startTimeSysTime = t;
		}

		int64_t tNow = startTimeSysTime + (1000*(pc.QuadPart - startTimePerfCount))/perfCountFreq;
		if (t > tNow || (tNow-t) > 16)
		{
			startTimePerfCount = pc.QuadPart;
			startTimeSysTime = t;
		}
		else
			t = tNow;
	}

	__time64_t tin = t/1000;
	if (date_ != NULL)
		_localtime64_s(date_,&tin);
	if (secs_ != NULL)
		*secs_ = (time_t) tin;
	return (uint32_t) (t % (XSENS_SEC_PER_DAY*1000));
#else
	timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp); // compile with -lrt

	if (date_ != NULL)
		localtime_r(&tp.tv_sec,date_);

	if (secs_ != NULL)
		secs_[0] = tp.tv_sec;

	// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
	return (1000 * (tp.tv_sec % XSENS_SEC_PER_DAY)) + (tp.tv_nsec/1000000);
#endif
}

int64_t getDateTime(tm *date)
{
#ifdef _WIN32
	__time64_t t;
	_time64(&t);
	if(date != 0)
		_localtime64_s(date, &t);
	return (int64_t)t;
#else
	time_t t;
	time(&t);
	if(date != 0)
	{
		tm *result = localtime(&t);
		memcpy(date, result, sizeof(tm));
	}
	return (int64_t)t;
#endif
}

void getDateAsString(uint8_t *dest, const tm* date)
{
	tm dt;
	
	if(date != 0)
		dt = *date;	
	else
		getDateTime(&dt);
	
	int year = dt.tm_year + 1900;
	int month = dt.tm_mon + 1;
	char temp[9];
	sprintf(temp, "%04d%02d%02d", year, month, dt.tm_mday);
	memcpy(dest, temp, 8);
}

void getTimeAsString(uint8_t *dest, const tm* date)
{
	tm dt;

	if(date != 0)
		dt = *date;
	else
		getDateTime(&dt);

	char temp[9];
	sprintf(temp, "%02d%02d%02d%02d", dt.tm_hour, dt.tm_min, dt.tm_sec, 0);
	memcpy(dest, temp, 8);
}

/*! \brief Sleep for \a ms milliseconds

  A platform independent sleep routine to sleep for at least \a ms milliseconds.

  On linux we are using pthread_cond_t, which implies that the timedwait() is a cancellation point.
  This function might return sooner than expected in some cases.
*/
void msleep(uint32_t ms)
{
#ifdef _WIN32
	Sleep(ms);
#elif defined(__APPLE__)
	/* untested yet */
	clock_t end = clock() + (CLOCKS_PER_SEC/1000) * ms;
	clock_t diff;

	while ((diff = end - clock()) > 0)
	{
		diff = (1000 * diff) / CLOCKS_PER_SEC;
		if (diff > 1000)
			sleep(diff / 1000);
		else
			usleep(diff * 1000);
	}
#else
	pthread_cond_t c;
	pthread_condattr_t ca;
	pthread_mutex_t m;

	pthread_condattr_init(&ca);

	pthread_mutex_init(&m, NULL);

	clockid_t clockid =  CLOCK_MONOTONIC;
	// using the monotonic clock -- The real time clock is affected by external processes such as ntp and influence the delay too much.
	int ret = pthread_condattr_setclock(&ca, clockid);
	// if this one failed (which should not happen), get the currently used clock instead
	if (ret)
		pthread_condattr_getclock(&ca, &clockid);

	pthread_cond_init(&c, &ca);

	timespec ts;

	clock_gettime(clockid, &ts);

	int64_t s = ms/1000;
	ts.tv_sec += s;

	ms -= s*1000;
	int64_t ns = ts.tv_nsec + (ms * 1000000);

	ts.tv_sec += ns / 1000000000L;
	ts.tv_nsec = ns % 1000000000L;

	pthread_mutex_lock(&m);
	pthread_cond_timedwait(&c, &m, &ts);

	pthread_mutex_unlock(&m);
	pthread_cond_destroy(&c);
	pthread_condattr_destroy(&ca);
	pthread_mutex_destroy(&m);
#endif
}

/*! \brief A single time of receive / time of send pair
*/
struct TimeSyncHistoryUnit {
	double tS, tR, tCR;
};

/*! \brief Private data for the TimeSync class
*/
class TimeSyncPrivate {
public:
	int m_historySize;		//!< The number of items stored in the history
	int m_quickSkewWindow;	//!< The number of steps to apply the quick skew estimation algorithm
	double m_a;				//!< Fuzzy factor
	double m_C;				//!< Stability factor (lower is more stable)

	TimeStamp m_tS0;		//!< Linearization point for S (external) values
	TimeStamp m_tR0;		//!< Linearization point for R (local) values

	double m_initialSkew;	//!< Initial skew to apply. When 0, the skew will be estimated automatically.
	double m_skew;			//!< Last known computed skew

	double m_outlierThreshold;	//!< Dynamic threshold for detecting outliers in the update function
	double m_outlierDelta;		//!< Used to move m_outlierThreshold up or down (lower value is less movement)
	double m_correction;		//!< Last known correction value computed from the history
	double m_tMc;				//!< Linearization point for the xsens::TimeSync::localTime and xsens::TimeSync::externalTime functions
	double m_tC;				//!< Last known computed time value, used in update and time conversion functions

	int m_historyMaxIndex;		//!< Index of the maximum tCR value in the history
	int m_historyCount;			//!< Items currently in the history
	int m_historyIndex;			//!< Index of the next item to write in the history (in a full history this is also the oldest item index)

	TimeSyncHistoryUnit* m_history;		//!< The time sync history buffer, implemented as a circular buffer

	bool m_initialized;			//!< Whether an initial estimate has been made and the normal time sync estimation can take place

	void addHistoryItem(const TimeSyncHistoryUnit& tu);
	double historyMax() const;
	int lastIndex() const;
	int firstIndex() const;
};

/*! \brief Add an item to the history, updating the m_historyMaxIndex value if necessary
*/
void TimeSyncPrivate::addHistoryItem(const TimeSyncHistoryUnit& tu)
{
	m_history[m_historyIndex] = tu;
	if (m_historyCount < m_historySize)
		++m_historyCount;

	bool redetermine = (m_historyIndex == m_historyMaxIndex);

	if (redetermine)
	{
		// determine new maximum from scratch
		m_historyMaxIndex = 0;
		double maxValue = m_history[0].tCR;
		for (int i = 1; i < m_historyCount; ++i)
		{
			if (m_history[i].tCR > maxValue)
			{
				m_historyMaxIndex = i;
				maxValue = m_history[i].tCR;
			}
		}
	}
	else
	{
		// only compare with current max
		if (tu.tCR > m_history[m_historyMaxIndex].tCR)
			m_historyMaxIndex = m_historyIndex;
	}
	m_historyIndex = (m_historyIndex+1) % m_historySize;
}

/*! \brief Index of the last (most recent) item in the history
*/
int TimeSyncPrivate::lastIndex() const
{
	return m_historyIndex ? m_historyIndex-1 : m_historySize-1;
}

/*! \brief Maximum tCR value in the history
*/
double TimeSyncPrivate::historyMax() const
{
	return m_history[m_historyMaxIndex].tCR;
}

/*! \brief Index of the first (oldest) item in the history
*/
int TimeSyncPrivate::firstIndex() const
{
	return (m_historyCount==m_historySize)?m_historyIndex:0;
}

/*! \class TimeSync
	\brief Class for synchronizing two clocks
*/

/*! \brief Constructor */
TimeSync::TimeSync(double skew, int historySize)
: d(new TimeSyncPrivate)
{
	d->m_historySize = historySize;
	d->m_quickSkewWindow = historySize / 2;

	d->m_a = 0.005;
	d->m_C = 0.01;
	d->m_initialSkew = skew;
	d->m_outlierDelta = 0.01;

	d->m_history = new TimeSyncHistoryUnit[historySize];

	reset();
}

/*! \brief Destructor */
TimeSync::~TimeSync()
{
	delete[] d->m_history;
	delete d;
}

/*! \brief Reset the time sync algorithm
*/
void TimeSync::reset()
{
	d->m_tS0 = 0;
	d->m_tR0 = 0;
	d->m_outlierThreshold = 2;
	d->m_correction = 0;
	d->m_tMc = 0;
	d->m_tC = 0;
	d->m_skew = 1;

	d->m_initialized = false;
	d->m_historyCount = 0;
	d->m_historyIndex = 0;
	d->m_historyMaxIndex = 0;
}

/*! \brief Set the initial skew value. Since using this value requires a reset, the function automatically resets the TimeSync.
*/
void TimeSync::setInitialSkew(double skew)
{
	d->m_initialSkew = skew;
	reset();
}

/*! \brief Returns whether the TimeSync has passed the initial stage of initialization
*/
int TimeSync::isInitialized() const
{
	return d->m_initialized;
}

/*! \brief Update the time synchronization with new information

	The computations can be done in double precision, since doubles have a worst-case
	precision in the 'normal' range of 15 decimals, which fits about 32000 years while
	retaining ms precision.
	\param local The local time. This time value may have noise on it caused by a delay.
	\param external The external time. This time must be exactly as supplied by the time source.
*/
void TimeSync::update(TimeStamp local, TimeStamp external)
{
	if (!d->m_historyCount)
	{
		d->m_tR0 = local;
		d->m_tS0 = external;
	}
	double tR = (double) (local - d->m_tR0);
	double tS = (double) (external - d->m_tS0);

	if (d->m_initialized)
	{
		double tSold = d->m_history[d->firstIndex()].tS;
		double tRold = d->m_history[d->firstIndex()].tR;
		//double tSprev = d->m_history[d->lastIndex()].tS;

		double newSkew = (tR-tRold)/(tS-tSold);
		double b = d->m_a;

		if (d->m_historyCount <= d->m_quickSkewWindow)
		{
			b = 0.1;
			if (d->m_initialSkew != 0)
			{
				d->m_skew = d->m_initialSkew;
				newSkew = d->m_initialSkew;
			}
			else
				d->m_skew = d->m_skew*0.75 + newSkew*0.25;
		}
		
		double innovation = fabs(1.0-newSkew/d->m_skew);
		if (innovation < d->m_outlierThreshold)
		{
			d->m_skew = (1.0-d->m_C)*d->m_skew + d->m_C*newSkew;

			// 'b' is used to determine the UPDATE
			d->m_tC += d->m_skew*(tS-d->m_history[d->lastIndex()].tS) - b*d->historyMax();

			TimeSyncHistoryUnit tu = { tS, tR, d->m_tC-tR };
			d->addHistoryItem(tu);

			// 'a' is used to determine the correction value to use when applying timesync
			d->m_correction = d->m_a*d->historyMax();
			d->m_tMc = tS;

			double ota = d->m_outlierThreshold * (1.0-d->m_outlierDelta);
			double otb = innovation*(1+d->m_outlierDelta);
			d->m_outlierThreshold = (ota>otb)?ota:otb;
		}
		else
			d->m_outlierThreshold *= 1.0+d->m_outlierDelta;
	}
	else
	{
		// not initialized
		TimeSyncHistoryUnit tu = { tS, tR, 0 };
		d->addHistoryItem(tu);

		if (d->m_historyCount > 1)
		{
			double dtR = tR - d->m_history[0].tR;
			double dtS = tS - d->m_history[0].tS;
			
			// check if first values are usable or if we should reset
			if (dtS < 1 || dtR < 1)
				reset();
			else
			{
				if (d->m_initialSkew != 0)
					d->m_skew = d->m_initialSkew;
				else
					d->m_skew = dtR / dtS;
				d->m_initialized = true;
			}
		}
		d->m_tC = tR;
		d->m_tMc = tS;
	}
}

/*! \brief Returns the local time computed from the supplied reference time
*/
TimeStamp TimeSync::localTime(TimeStamp tMi)
{
	return d->m_tR0 + (TimeStamp) (d->m_tC + d->m_skew*((double) (tMi - d->m_tS0) - d->m_tMc) - d->m_correction);
}

/*! \brief Returns the reference time computed from the supplied local time
*/
TimeStamp TimeSync::externalTime(TimeStamp tLi)
{
	return d->m_tS0 + (TimeStamp) (d->m_tMc + ((double) (tLi - d->m_tR0) - d->m_tC + d->m_correction)/d->m_skew);
}

TimeStamp timeStampNow()
{
	TimeStamp ms;
	time_t s;
	ms = (TimeStamp) getTimeOfDay(NULL,&s);
	ms = (ms % 1000) + (((TimeStamp)s)*1000);

	return ms;
}

/*! \brief Stabilize the clock
	\details Repeatedly call timeStampNow for 16-32 ms to stabilize the clock
*/
void initializeTime()
{
	TimeStamp start = timeStampNow();
	while (timeStampNow() - start < 32) {}
}


//////////////////////////////////////////////////////////////////////////////////////////
// MillisecondTimer implementation

/*! \brief Constructor
*/
MillisecondTimer::MillisecondTimer()
{
	restart();
}

/*! \brief Set the timer back to 0
*/
void MillisecondTimer::restart()
{
	m_tstart = xsens::getTimeOfDay();
}

/*! \brief Returns the number of milliseconds that has elapsed since the constructor or the restart function were called.
Note that the timer wraps around after 24 hours
*/
uint32_t MillisecondTimer::millisecondsElapsed()
{
	uint32_t tnow = getTimeOfDay();

	if (tnow >= m_tstart)
		return (tnow - m_tstart);
	else // TimeOfDay has wrapped around:
		return (tnow - m_tstart + XSENS_MS_PER_DAY);
}

}	// end of xsens namespace
