#include "time_profile.h"

int timeval_subtract(struct timeval *time_elapsed, struct timeval *time_end, struct timeval *time_start)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (time_end->tv_usec < time_start->tv_usec) {
    int nsec = (time_start->tv_usec - time_end->tv_usec) / 1000000 + 1;
    time_start->tv_usec -= 1000000 * nsec;
    time_start->tv_sec += nsec;
  }
  if (time_end->tv_usec - time_start->tv_usec > 1000000) {
    int nsec = (time_start->tv_usec - time_end->tv_usec) / 1000000;
    time_start->tv_usec += 1000000 * nsec;
    time_start->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  time_elapsed->tv_sec = time_end->tv_sec - time_start->tv_sec;
  time_elapsed->tv_usec = time_end->tv_usec - time_start->tv_usec;

  /* Return 1 if result is negative. */
  return time_end->tv_sec < time_start->tv_sec;
}


void show_time(char *name, struct timeval begin)
{
  struct timeval end, total_time;
  gettimeofday(&end, NULL);
  timeval_subtract(&total_time, &end, &begin);

  double p = total_time.tv_sec + total_time.tv_usec*1.0e-6;
  printf("%s %lfs\t%lf FPS\n", name, p, 1.0 / p);
}
