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

#include <err.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <../src/sickldmrs-private.h>
#include <sickldmrs.h>
#include <errno.h>


volatile int done = 0;
static int last_frame = -1;

static void
interrupt(int sig __attribute__ ((unused)))
{
	done++;
}

int
main(int argc __attribute__ ((unused)), char *argv[])
{
	struct sickldmrs_device *dev;
	char *address, *port;
	int rc;

	address = argv[1];
	port = argv[2];

	signal(SIGINT, interrupt);
	signal(SIGTERM, interrupt);

	dev = sickldmrs_init(address, port, true);
	if (dev == NULL) 
		exit(2);
	dev->debug = 1;
	if ((rc = sickldmrs_get_status(dev, -1)) < 0)
		errx(2, "sickldmrs_get_status: %s\n", strerror(-rc));
	if ((rc = sickldmrs_config_output(dev, 0x00ee, -1)) < 0)
		errx(2, "sickldmrs_config_output: %s\n", strerror(rc));
	/* scan frequency -> 25 Hz */
	if ((rc = sickldmrs_set_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, 6400, -1)) < 0)
		errx(2, "sickldmrs_set_parameter: %s", strerror(rc));
	if ((rc = sickldmrs_get_parameter(dev, SLDMRS_PARAM_SCAN_FREQUENCY, -1)) < 0)
		errx(2, "sickldmrs_get_parameter: %s", strerror(rc));

	usleep(400000);
	dev->priv->done = 1;
	dev->debug = 0;

	while (!done)
	{
		int rv = sickldmrs_receive_frame(dev, -1);
		if (rv != 0 && errno == ETIMEDOUT)
			continue;
		if (dev->scan != NULL && dev->scan->scan_number != last_frame)
		{
			printf("passei\n");
			sickldmrs_print_scan2(dev->scan);
			last_frame = dev->scan->scan_number;
		}
	}
	sickldmrs_end(dev);
	printf("bye\n");
	return 0;
}
