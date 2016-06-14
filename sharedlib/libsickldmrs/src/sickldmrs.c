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
#include <netdb.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sickldmrs.h"
#include "sickldmrs-private.h"

static void *sickldmrsReadTask(void *);

/*
 * Connect to the specified IPv4 address and port
 *
 * addr is a string containing and IPv4 address like "192.168.1.1"
 * port is a string containing the port number, like "12000"
 */
static int
connect_fd(const char *addr, const char *port)
{
	struct addrinfo hints, *res, *rp;
	int fd, s;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET; /* ld-mrs only supports v4 */
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_NUMERICHOST;
	s = getaddrinfo(addr, port, &hints, &res);
	if (s != 0) {
		warn("getaddrinfo: %s", gai_strerror(s));
		return -1;
	}
	for (rp = res; rp != NULL; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype,rp->ai_protocol);
		if (fd == -1)
			continue;
		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break;
		close(fd);
	}
	freeaddrinfo(res);
	if (rp == NULL) {
		warn("could not connect to %s:%s\n", addr, port);
		return -1;
	}
	return fd;
}

/**
 * sickldmrs_init - Initialize the sick LD-MRS device
 *
 * addr is a string containing its IPv4 address like "192.168.1.1"
 * port is a string containing the port number, like "12000"
 */
struct sickldmrs_device *
sickldmrs_init(const char *addr, const char *port, bool readTask)
{
	struct sickldmrs_device *dev;
	struct sickldmrs_private *priv;

	dev = malloc(sizeof(struct sickldmrs_device));
	if (dev == NULL)
		return NULL;
	memset(dev, 0, sizeof(struct sickldmrs_device));
	priv = (struct sickldmrs_private *)
	    malloc(sizeof(struct sickldmrs_private));
	if (priv == NULL) {
		free(dev);
		return NULL;
	}
	dev->priv = priv;
	if (pthread_mutex_init(&priv->lock, NULL) != 0) {
		free(priv);
		free(dev);
		return NULL;
	}
	if (pthread_cond_init(&priv->cond, NULL) != 0) {
		pthread_mutex_destroy(&priv->lock);
		free(priv);
		free(dev);
		return NULL;
	}
	dev->priv->fd = connect_fd(addr, port);
	if (dev->priv->fd == -1) {
		pthread_mutex_destroy(&priv->lock);
		free(priv);
		free(dev);
		return NULL;
	}
	if (readTask) {
		if (pthread_create(&dev->priv->tid, NULL,
			sickldmrsReadTask, dev) != 0) {
			close(dev->priv->fd);
			free(priv);
			free(dev);
			return NULL;
		}
	} else
		priv->tid = -1;
	priv->pending = -1;
	return dev;
}

/**
 * sickldmrs_end - Terminate the connection and free associated resources
 */
void
sickldmrs_end(struct sickldmrs_device *dev)
{
	pthread_mutex_destroy(&dev->priv->lock);
	close(dev->priv->fd);
	free(dev->priv);
	free(dev);
}

/**
 * sickldmrs_lock - lock the device structure
 */
int
sickldmrs_lock(struct sickldmrs_device *dev)
{
	return pthread_mutex_lock(&dev->priv->lock);
}

/**
 * sickldmrs_unlock - unlock the device structure
 */
int
sickldmrs_unlock(struct sickldmrs_device *dev)
{
	return pthread_mutex_unlock(&dev->priv->lock);
}

/**
 * discard an unknown message
 *
 * Skip over the length of the message as specified in the header
 */
static int
sickldmrs_ignore_data(struct sickldmrs_device *dev, struct sickldmrs_header *h)
{
	ssize_t len;
	uint8_t *buf;

	len = h->msg_size;
	buf = malloc(len);
	if (buf == NULL) {
		warn("%s: malloc failed", __func__);
		return -1;
	}
	len = recv(dev->priv->fd, buf, len, MSG_WAITALL);
	if (len <= 0) {
		warn("%s: recv error", __func__);
		free(buf);
		return -1;
	}
	free(buf);
	return 0;
}

/**
 * receive and decode object data (section 4, pp 10-13)
 */
static int
sickldmrs_recv_object_data(struct sickldmrs_device *dev,
    struct sickldmrs_header *h)
{
	/* XXX for now */
	return sickldmrs_ignore_data(dev, h);
}

/**
 * receive and decode sensorinfo (section 8 pp 32-33)
 */
static int
sickldmrs_recv_sensorinfo(struct sickldmrs_device *dev,
    struct sickldmrs_header *h)
{
	/* XXX for now */
	return sickldmrs_ignore_data(dev, h);
}

/**
 * receive and decode a command reply (section 5 pp 13-26)
 */
static int
sickldmrs_recv_reply(struct sickldmrs_device *dev, struct sickldmrs_header *h)
{
	uint8_t buf[64];
	ssize_t len;
	struct sickldmrs_reply *r;
	struct sickldmrs_status *s;
	struct sickldmrs_param *p;
	uint16_t rid;

	len = h->msg_size;
	len = recv(dev->priv->fd, buf, len, MSG_WAITALL);
	if (len <= 0) {
		warn("%s: recv error", __func__);
		return -1;
	}
	r = (struct sickldmrs_reply *)&buf[0];
	rid = le16toh(r->reply_id);
	sickldmrs_lock(dev);
	if (dev->debug)
		printf("-- %s 0x%04x\n", __func__, rid);
	switch (rid) {
	case SLDMRS_CMD_GET_STATUS:
		s = (struct sickldmrs_status *)&buf[0];
		if (len !=  sizeof(struct sickldmrs_status)) {
			warnx("%s: size mismatch %ld, %lu\n", __func__,
			    len, sizeof(struct sickldmrs_status));
			sickldmrs_unlock(dev);
			return -1;
		}
		snprintf(dev->firmware_version, sizeof(dev->firmware_version),
		    "%x.%02x.%x",
		    (s->firmware_version & 0xf000) >> 12,
		    (s->firmware_version & 0x0ff0) >> 4,
		    (s->firmware_version & 0x000f));
		snprintf(dev->fpga_version, sizeof(dev->fpga_version),
		    "%x.%02x.%x",
		    (s->fpga_version & 0xf000) >> 12,
		    (s->fpga_version & 0x0ff0) >> 4,
		    (s->fpga_version & 0x000f));
		snprintf(dev->serial_number, sizeof(dev->serial_number),
		    "%04x.%04x.%04x", s->serial_number_0,
		    s->serial_number_1, s->serial_number_2);
		dev->status = s->scanner_status;
		if (s->temperature < 0x7fff)
			dev->temperature = -(s->temperature - 579.2364)/3.63;
		else
			dev->temperature = -273; /* XXX invalid */
		if (dev->debug) {
			printf("Device status: 0x%04x\n", dev->status);
			printf(" temperature: %02f\n", dev->temperature);
			printf(" dsp %s fpga %s serial %s\n", dev->firmware_version,
			    dev->fpga_version, dev->serial_number);
		}
		break;
	case SLDMRS_CMD_GET_PARAM:
		p = (struct sickldmrs_param *)&buf[0];
		if (len !=  sizeof(struct sickldmrs_param)) {
			warnx("%s: size mismatch %ld, %lu\n", __func__,
			    len, sizeof(struct sickldmrs_param));
			sickldmrs_unlock(dev);
			return -1;
		}
		if (dev->debug)
			printf("-- get param: %04x value %08x\n",
			    le16toh(p->param_index), le32toh(p->param_value));
		break;
	case SLDMRS_CMD_RESET:
	case SLDMRS_CMD_SAVE_CONFIG:
	case SLDMRS_CMD_SET_PARAM:
	case SLDMRS_CMD_RESET_FACTORY_DEFAULTS:
	case SLDMRS_CMD_START_MEASURE:
	case SLDMRS_CMD_STOP_MEASURE:
	case SLDMRS_CMD_SET_NTP_SEC:
	case SLDMRS_CMD_SET_NTP_FRAC_SEC:
		if (len !=  sizeof(struct sickldmrs_reply)) {
			warnx("%s: size mismatch %ld, %lu\n", __func__,
			    len, sizeof(struct sickldmrs_reply));
			sickldmrs_unlock(dev);
			return -1;
		}
		break;
	default:
		if ((rid & 0x8000) != 0) /* command error */
			break;
		warnx("%s: unknown command reply %x\n", __func__,
		    r->reply_id);
		sickldmrs_unlock(dev);
		return -1;
	}
	dev->priv->pending = rid;
	pthread_cond_broadcast(&dev->priv->cond);
	sickldmrs_unlock(dev);
	return 0;
}

/**
 * receive and decode error and warning messages (section 6 pp 27-30)
 */
static int
sickldmrs_recv_error(struct sickldmrs_device *dev, struct sickldmrs_header *h)
{
	struct sickldmrs_error e;
	ssize_t len;

	len = recv(dev->priv->fd, &e, sizeof(e), MSG_WAITALL);
	if (len <= 0) {
		warn("%s: recv error", __func__);
		return -1;
	}

	/*  store errors */
	sickldmrs_lock(dev);
	dev->errors.error_register_1 = le16toh(e.error_register_1);
	dev->errors.error_register_2 = le16toh(e.error_register_2);
	dev->errors.warning_register_1 = le16toh(e.warning_register_1);
	dev->errors.warning_register_2 = le16toh(e.warning_register_2);
	sickldmrs_unlock(dev);
	return 0;
}

/**
 * decode a NTP timestamp
 */
static void
sickldmrs_ts_decode(uint64_t t, struct timespec *ts)
{
	ts->tv_sec = (t & 0xffffffff00000000LL) >> 32;
	ts->tv_nsec = (t & 0x00000000ffffffffLL)*1000000000LL/0xffffffff;
}

/**
 * Convert ticks to radians (section 3, page 7)
 */
static double
tick_to_rad(struct sickldmrs_scan *scan, int16_t ticks)
{
	return 2.0 * M_PI * ticks / scan->angle_ticks_per_rotation;
}

/**
 * receive and decode scan data (section 3 pp 7-10)
 */
static int
sickldmrs_recv_scan(struct sickldmrs_device *dev, struct sickldmrs_header *h)
{
	struct sickldmrs_scan_header *scan;
	struct sickldmrs_scan *s;
	struct sickldmrs_point_wire *point;
	ssize_t len;
	uint8_t *buf, *p;
	int i;

	len = h->msg_size;
	buf = (uint8_t *)malloc(len);
	if (buf == NULL) {
		warn("%s: malloc error", __func__);
		return -1;
	}
	len = recv(dev->priv->fd, buf, len, MSG_WAITALL);
	if (len <= 0) {
		warn("%s: recv error", __func__);
		return -1;
	}
	scan = (struct sickldmrs_scan_header *)&buf[0];

	sickldmrs_lock(dev);
	/* adjust size of new scan structure */
	s = realloc(dev->scan, sizeof(struct sickldmrs_scan)
	    + le16toh(scan->scan_points) * sizeof(struct sickldmrs_point));
	if (s == NULL) {
		warn("%s: realloc returns NULL\n", __func__);
		free(dev->scan);
		dev->scan = NULL;
		sickldmrs_unlock(dev);
		return -1;
	}
	dev->scan = s;

	/* scan data is little endian */
	s->scan_number = le16toh(scan->scan_number);
	s->scanner_status = le16toh(scan->scanner_status);
	s->sync_phase_offset = le16toh(scan->sync_phase_offset);
	sickldmrs_ts_decode(le64toh(scan->scan_start_time),
	    &s->scan_start_time);
	sickldmrs_ts_decode(le64toh(scan->scan_end_time),
	    &s->scan_end_time);
	s->angle_ticks_per_rotation = le16toh(scan->angle_ticks_per_rotation);
	s->start_angle = tick_to_rad(s, le16toh(scan->start_angle));
	s->end_angle = tick_to_rad(s, le16toh(scan->end_angle));
	s->scan_points = le16toh(scan->scan_points);
	s->mount_yaw = tick_to_rad(s, le16toh(scan->mount_yaw));
	s->mount_pitch = tick_to_rad(s, le16toh(scan->mount_pitch));
	s->mount_roll = tick_to_rad(s, le16toh(scan->mount_roll));
	s->mount_x = le16toh(scan->mount_x) / 100.0;
	s->mount_y = le16toh(scan->mount_y) / 100.0;
	s->mount_z = le16toh(scan->mount_z) / 100.0;

	p = buf + sizeof(struct sickldmrs_scan_header);
	for (i = 0; i < scan->scan_points; i++) {
		point = (struct sickldmrs_point_wire *)p;
		/* store point */
		s->points[i].layer = point->layer_echo & 0x0f;
		s->points[i].echo = (point->layer_echo & 0xf0) >> 4;
		s->points[i].flags = point->flags;
		s->points[i].horizontal_angle
		    = tick_to_rad(s, le16toh(point->horizontal_angle));
		s->points[i].radial_distance
		    = le16toh(point->radial_distance) / 100.0;
		s->points[i].pulse_width
		    = le16toh(point->pulse_width) / 100.0;
		p += sizeof(struct sickldmrs_point_wire);
	}
	free(buf);
	sickldmrs_unlock(dev);
	return 0;
}


/**
 * wait for a message with timeout
 */
int
sickldmrs_msg_wait(int s, void *data, size_t len, int timeo)
{
	struct pollfd pfd;
	ssize_t nbytes;
	int rc;

	if (timeo >= 0) {
		pfd.fd = s;
		pfd.events = POLLIN;
		for (;;) {
			rc = poll(&pfd, 1, timeo);
			if (rc == -1 && errno == EINTR)
				continue;
			if (rc == -1)
				return rc;
			if (rc == 0) /* timeout */ {
				errno = ETIMEDOUT;
				return -1;
			}
			break;
		}
	}
	nbytes = recv(s, data, len, MSG_WAITALL);
	if (nbytes < 0) {
#ifdef SOCKET_DEBUG
		warn("%s: recv", __func__);
#endif
		return -1;
	}

	/* paranoid check ... */
	if (nbytes < len) {
		warnx("%s: recv: incomplete frame", __func__);
	}
	return nbytes;
}


/**
 * receive and decode a frame from the LD-MRS device
 */
int
sickldmrs_receive_frame(struct sickldmrs_device *dev, int timeo)
{
	struct sickldmrs_header h;
	ssize_t len;
	int retval = -1;

	len = sickldmrs_msg_wait(dev->priv->fd, &h, sizeof(h), timeo);
	if (len != sizeof(h)) {
		warnx("%s: recv error", __func__);
		return -1;
	}
	/* header values are sent as big endian */
	if (be32toh(h.magic_word) != SLDMRS_MAGIC_WORD) {
		warnx("%s: bad magic word %04x %04x", __func__,
		    h.magic_word, be32toh(h.magic_word));
		return -1;
	}
	/* convert remaining header fields to host endianness */
	h.data_type = be16toh(h.data_type);
	h.msg_size = be32toh(h.msg_size);
	h.ntp_time = be64toh(h.ntp_time);

	if (dev->debug)
		printf("-- %s: frame 0x%04x len %d\n", __func__,
	    h.data_type, h.msg_size);
	switch (h.data_type) {
	case SLDMRS_COMMAND:
		warnx("%s: unexpected command frame", __func__);
		break;
	case SLDMRS_REPLY:
		retval = sickldmrs_recv_reply(dev, &h);
		break;
	case SLDMRS_SCAN_DATA:
		retval = sickldmrs_recv_scan(dev, &h);
		break;
	case SLDMRS_IBEO_SCAN:
		warnx("%s: unexpected IBEO scan data", __func__);
		sickldmrs_ignore_data(dev, &h);
		break;
	case SLDMRS_ERROR_WARNING:
		retval = sickldmrs_recv_error(dev, &h);
		break;
	case SLDMRS_OBJECT_DATA:
		retval = sickldmrs_recv_object_data(dev, &h);
		break;
	case SLDMRS_MOVEMENT_DATA:
		retval = sickldmrs_ignore_data(dev, &h);
		break;
	case SLDMRS_SENSORINFO:
		retval = sickldmrs_recv_sensorinfo(dev, &h);
		break;
	default:
		warnx("%s: unknown data type 0x%x\n", __func__, h.data_type);
		retval = sickldmrs_ignore_data(dev, &h);
		break;
	}
	return retval;
}


/**
 * a task reading data from the CAN controller and decoding messages
 * it also unlocks waiting requests functions
 */
static void *
sickldmrsReadTask(void *arg)
{
	struct sickldmrs_device *dev = (struct sickldmrs_device *)arg;
	int rv;

	while (!dev->priv->done) {
		rv = sickldmrs_receive_frame(dev, -1);
		if (rv != 0 && errno == ETIMEDOUT)
			continue;
		if (rv != 0) {
			warn("sickldmrsReadTask: sickldmrs_receive_frame");
			break;
		}
	}
	return NULL;
}
