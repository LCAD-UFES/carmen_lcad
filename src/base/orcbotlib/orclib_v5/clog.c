 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <stdio.h>
#include <sys/time.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "clog.h"

int clog_log_level = 4;

static int clog_init_flag = 0;
static int clog_stdout_flag = 1;
static int clog_logfile_fd = -1;

static struct timeval clog_start_tv;

static char *LOGLEVELS[] = {"FATAL", "ERROR", "WARN", "INFO", "VERBOSE", "DEBUG"};

char *clog_mallocf(const char *fmt, ...)
{
	int sz = 256;
	char *buf = NULL;
	va_list ap;
	int len;

_revprintf:
        buf = (char*) realloc(buf,sz);
	va_start(ap, fmt);                                         
	len = vsnprintf(buf, sz, fmt, ap);
	if (len < 0) { 
		sz*=2; 
		goto _revprintf; 
	}
	if (len >= sz) {
		sz = len+1; 
		goto _revprintf; 
	}
	va_end(ap);                                                

	return buf;
}

void clog_log_message(const char *source, int level, const char *file, int line, const char *msg, int freemsg)
{
	if (level >= clog_log_level)
		return;

	if (!clog_init_flag) {
		clog_init();
		clog_log_message("CLOG", 2, __FILE__, __LINE__, "clog_init() not called at application startup", 0);
	}

	struct timeval _tv;

	gettimeofday(&_tv, NULL);
	long t = _tv.tv_sec*1000 + _tv.tv_usec/1000 - clog_start_tv.tv_sec*1000 + clog_start_tv.tv_usec/1000;

	char *msgf = clog_mallocf("%8s %8s %6li.%03li %10s:%-4i   %s\n", source, LOGLEVELS[level],  t/1000, t%1000, file, line, msg);
	int len = strlen(msgf);

	if (clog_stdout_flag)
		write(STDOUT_FILENO, msgf, len);

	if (clog_logfile_fd >= 0)
		write(clog_logfile_fd, msgf, len);

	free(msgf);

	if (freemsg)
		free((char*) msg);
}
    
void clog_init()
{
	gettimeofday(&clog_start_tv, NULL);
	clog_init_flag = 1;
}

void clog_set_level(int level)
{
	clog_log_level = level;
}

void clog_set_logfile(int fd)
{
	clog_logfile_fd = fd;
}
