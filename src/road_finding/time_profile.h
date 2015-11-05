#ifndef TIME_PROFILE_H
#define TIME_PROFILE_H

#include <sys/time.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

int timeval_subtract(struct timeval *time_elapsed, struct timeval *time_end, struct timeval *time_start);
void show_time(char *name, struct timeval begin);

#ifdef __cplusplus
}
#endif

#endif
