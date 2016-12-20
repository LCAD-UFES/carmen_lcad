/*
 * Copyright (c) 2015 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

#include "sickldmrs.h"

/**
 * return a string representing scan flags
 */
static char *
scan_flags(uint16_t flags)
{
	static char buffer[256];
	
	snprintf(buffer, sizeof(buffer), "\"%s,%s,%s,%s,%s\"",
	    flags & SLDMRS_SCAN_FLAG_GROUND_DETECTION ? 
	    "ground detection" : "no ground detection",
	    flags & SLDMRS_SCAN_FLAG_DIRT_DETECTION ?
	    "dirt detection" : "no dirt detection",
	    flags & SLDMRS_SCAN_FLAG_RAIN_DETECTION ?
	    "rain detection" : "no rain detection",
	    flags & SLDMRS_SCAN_FLAG_TRANS_DETECTION ?
	    "transparency detection" : "no transparency detection",
	    flags & SLDMRS_SCAN_FLAG_HORIZONTAL_OFFSET ?
	    "horizontal offset" : "no horizontal offset");
	return buffer;
}

/**
 * return a string representing point flags 
 */
static char *
point_flags(uint8_t flags)
{
	static char buffer[256];

	snprintf(buffer, sizeof(buffer), "\"%s,%s,%s,%s\"",
	    flags & SLDMRS_POINT_FLAG_TRANSPARENT ? "transparent" : "opaque",
	    flags & SLDMRS_POINT_FLAG_NOISE ? "atmospheric noise" : "no noise",
	    flags & SLDMRS_POINT_FLAG_GROUND ? "ground" : "data",
	    flags & SLDMRS_POINT_FLAG_DIRT ? "dirt" : "clean");;
	return buffer;
}

/**
 * print to stdout some informations about a scan
 */
void
sickldmrs_print_scan(struct sickldmrs_scan *scan)
{
	int i;

	printf("\nscan no %d", scan->scan_number);
	printf(" %d points\n%s\n", scan->scan_points, scan_flags(scan->flags));
	printf(" %.3lf..%.3lf\n", scan->start_angle, scan->end_angle);

	printf(" %lu.%09lus..%lu.%09lus\n",
	    scan->scan_start_time.tv_sec, scan->scan_start_time.tv_nsec,
	    scan->scan_end_time.tv_sec, scan->scan_end_time.tv_nsec);
	printf(" position %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
	    scan->mount_yaw, scan->mount_pitch, scan->mount_roll,
	    scan->mount_x, scan->mount_y, scan->mount_z);

	for (i = 0; i < scan->scan_points; i++) {
		printf("  %6d %1d %1d %s %.5f %.3f %.3f\n",
		    i, scan->points[i].layer, scan->points[i].echo,
		    point_flags(scan->points[i].flags),
		    scan->points[i].horizontal_angle,
		    scan->points[i].radial_distance,
		    scan->points[i].pulse_width);
	}
}

/**
 * print to stdout some informations about a scan
 */
void
sickldmrs_print_scan2(struct sickldmrs_scan *scan)
{
	printf("\nscan no %d", scan->scan_number);
	printf(" %d points\n%s\n", scan->scan_points, scan_flags(scan->flags));
	printf(" %.3lf..%.3lf\n", scan->start_angle, scan->end_angle);

	printf(" %lu.%09lus..%lu.%09lus\n",
	    scan->scan_start_time.tv_sec, scan->scan_start_time.tv_nsec,
	    scan->scan_end_time.tv_sec, scan->scan_end_time.tv_nsec);
	printf(" position %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
	    scan->mount_yaw, scan->mount_pitch, scan->mount_roll,
	    scan->mount_x, scan->mount_y, scan->mount_z);
}

