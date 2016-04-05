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
#include <sys/types.h>
#include <sys/socket.h>
#include <err.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "sickldmrs.h"
#include "sickldmrs-private.h"

/* timespec addition - only works with positive values */
static void
timespec_add(struct timespec *a, const struct timespec *b)
{
	a->tv_sec += b->tv_sec;
	a->tv_nsec = a->tv_nsec + b->tv_nsec;
	if (a->tv_nsec > 999999999) {
		a->tv_sec += a->tv_nsec / 1000000000;
		a->tv_nsec = a->tv_nsec % 1000000000;
	}
}

/**
 * @brief Waits for the specified reply for timeo ms
 *
 * @param dev pointer to communication structure
 * @param rqst identifier of the response to wait for
 * @param timeo < 0 -> block
 * timeo = 0 -> no wait
 * timeo > 0 wait..
 *
 * @return 0 if message arrived; <0 (-errno) otherwise;
 */
int
sickldmrs_wait_reply(struct sickldmrs_device *dev, int16_t rqst, int timeo)
{
	struct timespec d, ts;
	int rc;

	sickldmrs_lock(dev);
	if (timeo == 0) {
		if (dev->priv->pending == rqst) {
			dev->priv->pending = -1;
			sickldmrs_unlock(dev);
			return 0;
		}
	}
	if (timeo > 0) {
		d.tv_sec = timeo / 1000;
		d.tv_nsec = (timeo % 1000) * 1000000;
		clock_gettime(CLOCK_REALTIME, &ts);
		timespec_add(&ts, &d);
		rc = 0;
		/* wait for condition variable with timeout */
		while (dev->priv->pending != rqst && rc == 0)
			rc = pthread_cond_timedwait(&dev->priv->cond,
			    &dev->priv->lock, &ts);
		dev->priv->pending = -1;
		sickldmrs_unlock(dev);
		return -rc;
	}
	/* timeo < 0 -> blocking wait for condition variable */
	while ((dev->priv->pending & 0x1fff) != rqst) {
		rc = pthread_cond_wait(&dev->priv->cond, &dev->priv->lock);
		if (rc != 0) {
			warnx("%s: %s\n", __func__, strerror(rc));
			sickldmrs_unlock(dev);
			return -rc;
		}
	}
	/* check for error bit in the command id */
	if ((dev->priv->pending & 0x8000) != 0)
		rc = -EIO;
	else
		rc = 0;
	dev->priv->pending = -1;
	sickldmrs_unlock(dev);
	return rc;
}

/**
 * send a generic command to the device
 */
static int
sickldmrs_send_command(struct sickldmrs_device *dev, 
    void *data, size_t len, int timeo)
{
	struct sickldmrs_header *h;
	struct sickldmrs_cmd *cmd;
	uint8_t *buf;
	size_t pktlen;
	int ret;

	pktlen = sizeof(*h) + len;
	buf = malloc(pktlen);
	if (buf == NULL) {
		warn("%s: malloc error", __func__);
		return -1;
	}
	/* initialize header */
	memset(buf, 0, sizeof(*h));
	h = (struct sickldmrs_header *)buf;
	h->magic_word = htobe32(SLDMRS_MAGIC_WORD);
	h->msg_size = htobe32(len);
	h->data_type = htobe16(SLDMRS_COMMAND);
	memcpy(buf + sizeof(*h), data, len);
	cmd = (struct sickldmrs_cmd *)(buf + sizeof(*h));
	ret =  send(dev->priv->fd, buf, pktlen, 0);
	free(buf);
	if (timeo == 0 || ret < 0)
		return ret;
	return sickldmrs_wait_reply(dev, cmd->command_id, timeo);
}

/**
 * Send the Reset command to the device (section 5.3.1 page 14)
 */
int
sickldmrs_reset(struct sickldmrs_device *dev, int timeo)
{
	struct sickldmrs_cmd cmd;

	cmd.command_id = htole16(SLDMRS_CMD_RESET);
	cmd.reserved = 0;
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Get Status command (section 5.3.2, page 15)
 */
int
sickldmrs_get_status(struct sickldmrs_device *dev, int timeo)
{
	struct sickldmrs_cmd cmd;

	cmd.command_id = htole16(SLDMRS_CMD_GET_STATUS);
	cmd.reserved = 0;
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Save Config command (section 5.3.3 page 16)
 */
int
sickldmrs_save_config(struct sickldmrs_device *dev, int timeo)
{
	struct sickldmrs_cmd cmd;

	cmd.command_id = htole16(SLDMRS_CMD_SAVE_CONFIG);
	cmd.reserved = 0;
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Set Parameters commands to set the network configuration
 * (sections 5.3.4 and 5.4, pages 16 and 18-19)
 */
int
sickldmrs_config_network(struct sickldmrs_device *dev,
    const char *addr, const char *mask, const char *gateway, uint16_t port,
    int timeo)
{
	return -1;
}

/**
 * Send the Set Parameters commands to configure the scans
 * (section 5.3.4 and 5.4, pages 16 and 20)
 */
int
sickldmrs_config_scan(struct sickldmrs_device *dev,
    double start, double end, int timeo)
{
	struct sickldmrs_set_param cmd;
	int ret;

	cmd.command_id = htole16(SLDMRS_CMD_SET_PARAM);
	cmd.reserved = 0;
	cmd.param_index = htole16(SLDMRS_PARAM_START_ANGLE);
	cmd.value = start/M_PI*180*32;
	if (end < start)
		return -EINVAL;
	if (cmd.value < -1919 || cmd.value > 1600)
		return -EINVAL;
	cmd.value = htole32(cmd.value);
	ret = sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
	if (ret < 0)
		return ret;
	cmd.param_index = SLDMRS_PARAM_END_ANGLE;
	cmd.value = end/M_PI*180*32;
	if (cmd.value < -1919 || cmd.value > 1600)
		return -EINVAL;
	cmd.value = htole32(cmd.value);
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Set Parameter commands to configure the output frames
 * (section 5.3.4, pages 16 and 19)
 */
int
sickldmrs_config_output(struct sickldmrs_device *dev, uint16_t config, 
    int timeo)
{
	struct sickldmrs_set_param cmd;
	cmd.command_id = htole16(SLDMRS_CMD_SET_PARAM);
	cmd.reserved = 0;
	cmd.param_index = htole16(SLDMRS_PARAM_DATA_OUTPUT_FLAGS);
	cmd.value = htole16(config);
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Set Parameter command (section 5.3.5, pp 16-17)
 */
int
sickldmrs_set_parameter(struct sickldmrs_device *dev,
    uint16_t param, long value, int timeo)
{
	struct sickldmrs_set_param cmd;
	cmd.command_id = htole16(SLDMRS_CMD_SET_PARAM);
	cmd.reserved = 0;
	cmd.param_index = htole16(param);
	cmd.value = htole32(value & 0x0ffffffff);
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Get Parameter command (section 5.3.5, pp 16-17)
 */
int
sickldmrs_get_parameter(struct sickldmrs_device *dev, uint16_t param, int timeo)
{
	struct sickldmrs_get_param cmd;
	cmd.command_id = htole16(SLDMRS_CMD_GET_PARAM);
	cmd.reserved = 0;
	cmd.param_index = htole16(param);
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Start or Stop Measure command (section 5.3.7, page 17)
 *
 * start: True -> Start, False -> Stop
 */
int
sickldmrs_start_measure(struct sickldmrs_device *dev, bool start, int timeo)
{
	struct sickldmrs_cmd cmd;

	cmd.command_id = start ? htole16(SLDMRS_CMD_START_MEASURE) :
	    htole16(SLDMRS_CMD_STOP_MEASURE);
	cmd.reserved = 0;
	return sickldmrs_send_command(dev, &cmd, sizeof(cmd), timeo);
}

/**
 * Send the Set NTP Timestamp commands to set the time on the device
 * (section 5.3.9 and 5.3.10, pp 17-18)
 */
int
sickldmrs_settime(struct sickldmrs_device *dev,
    const struct timespec *ts, int timeo)
{
	return -1;
}
