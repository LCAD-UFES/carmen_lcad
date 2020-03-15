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

#ifndef _LOG_H
#define _LOG_H

#include "clog.h"

#define LOG_SOURCE "LIBORC"

#define LOG_MESSAGE(level, fmt, ...) if (level < clog_log_level) \
		clog_log_message(LOG_SOURCE, level, __FILE__, __LINE__, clog_mallocf(fmt, ##__VA_ARGS__), 1);

#define LOG_FATAL(fmt, ...)    LOG_MESSAGE(0, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)    LOG_MESSAGE(1, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)     LOG_MESSAGE(2, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)     LOG_MESSAGE(3, fmt, ##__VA_ARGS__)
#define LOG_VERBOSE(fmt, ...)  LOG_MESSAGE(4, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)    LOG_MESSAGE(5, fmt, ##__VA_ARGS__)

#endif
