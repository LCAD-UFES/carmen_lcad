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

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdarg.h>

#include "orc.h"
#include "packet.h"
#include "timespec.h"
#include "clog.h"
#include "log.h"

#include "orcconstants.h"

static uint8_t garbage[256];

#define VPRINTALLOC                                                \
	int sz = 4096;						   \
	char *buf = NULL;                                          \
	_revprintf:                                                \
        buf = (char*) realloc(buf,sz);	               		   \
	va_list ap;                                                \
	int len;                                                   \
	va_start(ap, fmt);                                         \
	len = vsnprintf(buf, sz-1, fmt, ap);                       \
	if (len < 0 || len>=sz)                                    \
            { sz*=2; goto _revprintf; }                            \
	va_end(ap);                                                



static ssize_t read_fully(int fd, uint8_t *buf, ssize_t count)
{
        ssize_t readsofar = 0;
        ssize_t readthistime;

        while (readsofar < count) {
                readthistime = read(fd, &buf[readsofar], count - readsofar);
                if (readthistime < 0) {
			LOG_ERROR("ERROR: read failed with %ld (%s)\n", (long) readthistime, strerror(errno));
                        return -1; // error!
                }
                if (readthistime == 0) {
                        LOG_ERROR("ERROR: got EOF while reading %ld\n", (long) readthistime);
                        return -1; // that's an error for us.
                }

                readsofar += readthistime;
        }

        return readsofar;
}

static ssize_t write_fully(int fd, uint8_t *buf, ssize_t count)
{
        ssize_t writtensofar = 0;
        ssize_t writtenthistime;

        while (writtensofar < count) {
                writtenthistime = write(fd, &buf[writtensofar], count - writtensofar);
                if (writtenthistime <= 0)
                        return -1;
                writtensofar += writtenthistime;
        }

        return writtensofar;
}

static void setup_thread()
{
	int ot;
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,&ot);
	signal(SIGPIPE, SIG_IGN);
}

static void handle_heartbeat_packet(orc_t *orc, uint8_t *p)
{
	orc->heartbeat_time = packet_16u(p, 3);
	orc->heartbeat_flags = packet_8u(p, 7);
}

static void handle_pad_packet(orc_t *orc, uint8_t *p)
{
	orc->pad_switches = p[PACKET_DATA + 1];
	orc->pad_updown = p[PACKET_DATA + 2];
	orc->pad_leftright = p[PACKET_DATA + 3];
	orc->pad_joyx = p[PACKET_DATA + 4];
	orc->pad_joyy = p[PACKET_DATA + 5];

	pthread_mutex_lock(&orc->pad_mutex);
	pthread_cond_broadcast(&orc->pad_cond);
	pthread_mutex_unlock(&orc->pad_mutex);
}

void *reader_thread(void *_arg)
{
	orc_t *orc = (orc_t*) _arg;

	setup_thread();

	orc->fd = -1;

	int reconnectcount = 0;

reconnect:
	
	// reconnect, if necessary.
	while (orc->fd < 0) {
		LOG_INFO("Trying to connect to orcboard...(%i)", reconnectcount++);
		orc->fd = orc->impl->connect(orc->impl);
		
		if (orc->fd < 0)
			sleep(1);
	}

	// read for a while
	while (1) {

		// read a packet
		uint8_t buf[3];
		int res = read_fully(orc->fd, buf, 1);
		if (res <= 0)
			goto disconnected;
		if (buf[0] != 0xED) {
			LOG_DEBUG("Recovering sync [%02X]", buf[0]);
			continue;
		}

		res = read_fully(orc->fd, &buf[1], 2);
		if (res <= 0)
			goto disconnected;

		int id = buf[1];
		int datalen = buf[2];

		transaction_t *t = &orc->transactions[id];
		memcpy(t->response, buf, 3);

		res = read_fully(orc->fd, &t->response[PACKET_DATA], datalen + 1);
		if (res <= 0)
			goto disconnected;
		if (!packet_test_checksum(t->response)) {
			LOG_WARN("Bad checksum received from Orc");
			continue;
		}

		if (t->response == garbage && t->response[1]>=orc->idLow && t->response[1]<=orc->idHigh)
			LOG_VERBOSE("Unsolicited ack, id = %02X", t->response[1]);

		// is this a message from orcd, assigning us a client id?
		if (t->response[1] == 0xf7) {
			orc->idLow = t->response[4];
			orc->idHigh = t->response[5];
			orc->idLast = orc->idLow;
			LOG_INFO("Got client transaction range: %02x-%02x", orc->idLow, orc->idHigh);
		}

		if (t->response[1] == 0xfe)
			handle_pad_packet(orc, t->response);

		if (t->response[1] == PACKET_ID_ORCBOARD_BROADCAST &&
		    packet_16u(t->response, 1) == MSG_ASYNC_HEARTBEAT)
			handle_heartbeat_packet(orc, t->response);

		pthread_mutex_lock(&t->mutex);
		pthread_cond_signal(&t->cond);
		pthread_mutex_unlock(&t->mutex);

	}
	
disconnected:
	orc->impl->disconnect(orc->impl, orc->fd);
	orc->fd = -1;
	goto reconnect;

	// silence compiler
	return NULL;
}

orc_t *orc_create(orc_comms_impl_t *impl)
{
	orc_t *orc = (orc_t*) calloc(sizeof(orc_t), 1);
	orc->impl = impl;

	pthread_attr_t threadAttr;
	pthread_attr_init(&threadAttr);
	pthread_attr_setstacksize(&threadAttr, 32768);

	pthread_t newthread;

	if (pthread_create(&newthread, &threadAttr, reader_thread, orc)) {
		LOG_ERROR("Couldn't create orc reader thread: %s", strerror(errno));
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < 256; i++) {
		pthread_mutex_init(&orc->transactions[i].mutex, NULL);
		pthread_cond_init(&orc->transactions[i].cond, NULL);
		orc->transactions[i].inuse = 0;
		orc->transactions[i].response = garbage;
	}

	pthread_mutex_init(&orc->writeLock, NULL);
	pthread_mutex_init(&orc->idMutex, NULL);
	pthread_cond_init(&orc->idCond, NULL);

	orc->idUsed = 0;
	orc->idLow = 0x00; // initially, we can use any id
	orc->idHigh = 0xef;
	orc->idLast = orc->idLow;

	pthread_mutex_init(&orc->pad_mutex, NULL);
	pthread_cond_init(&orc->pad_cond, NULL);

	orc_null(orc, 0, 0);

	return orc;
}

void orc_destroy(orc_t *orc)
{
	free(orc);
}

static int alloc_transaction_id(orc_t *orc)
{
	int id;

	pthread_mutex_lock(&orc->idMutex);

	// wait for a transaction to become available
	int max = orc->idHigh - orc->idLow + 1;
	while (orc->idUsed >= max)
		pthread_cond_wait(&orc->idCond, &orc->idMutex);
		
	// search for a transaction id, beginning our search at idLast
	do {
		orc->idLast++;
		if (orc->idLast > orc->idHigh)
			orc->idLast = orc->idLow;
	} while (orc->transactions[orc->idLast].inuse);

	// initialize the transaction
	id = orc->idLast;
	
	orc->idUsed++;
	pthread_mutex_unlock(&orc->idMutex);

	return id;
}

static void free_transaction_id(orc_t *orc, int id)
{
	pthread_mutex_lock(&orc->idMutex);
	orc->transactions[id].inuse = 0;
	orc->idUsed--;
	pthread_cond_signal(&orc->idCond);
	pthread_mutex_unlock(&orc->idMutex);
}

int orc_transaction_once(orc_t *orc, uint8_t *request, uint8_t *response)
{
	while (orc->fd <= 0) {
		usleep(1000);
	}
	
	// grab an id
	int id = alloc_transaction_id(orc);

	// fix up the packet.
	request[0] = PACKET_MARKER;
	request[PACKET_ID] = id;
	packet_fill_checksum(request);

	// fill out the transaction record
	transaction_t *t = &orc->transactions[id];
	pthread_mutex_lock(&t->mutex);
	t->response = response;

	// send the packet
	pthread_mutex_lock(&orc->writeLock);
	write_fully(orc->fd, request, request[PACKET_DATALEN] + 4);
	pthread_mutex_unlock(&orc->writeLock);

	// compute our timeout time
	struct timespec ts;
	timespec_now(&ts);
	timespec_addms(&ts, 100);
	
	// wait!
	int res = pthread_cond_timedwait(&t->cond, &t->mutex, &ts);

	t->response = garbage;
	
	// cleanup
	pthread_mutex_unlock(&t->mutex);
	free_transaction_id(orc, id);

	if (res == ETIMEDOUT) {
		LOG_VERBOSE("Timeout packet, id = %02X", id);
		return -1;
	}

	return 0;
}

void orc_transaction_async(orc_t *orc, uint8_t *request)
{
	// fix up the packet.
	request[0] = PACKET_MARKER;
	request[PACKET_ID] = PACKET_ID_NO_ACK;
	packet_fill_checksum(request);

	// send the packet
	pthread_mutex_lock(&orc->writeLock);
	write_fully(orc->fd, request, request[PACKET_DATALEN] + 4);
	pthread_mutex_unlock(&orc->writeLock);
}

void orc_transaction_retry(orc_t *orc, uint8_t *request, uint8_t *response)
{
	int res;

loop:
	res = orc_transaction_once(orc, request, response);
	if (res==0)
		return;

	usleep(5000);
	goto loop;
}

#define CMDPROLOGUE uint8_t req[256]; \
	uint8_t resp[256];

void orc_lcd_clear(orc_t *orc)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 1;
	req[PACKET_DATA +0] = CMD_LCD_CLEAR;

	orc_transaction_once(orc, req, resp);
}

void orc_lcd_console_home(orc_t *orc)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 1;
	req[PACKET_DATA +0] = CMD_CONSOLE_HOME;

	orc_transaction_once(orc, req, resp);
}

#define CHUNKSIZE 20

static int min(int a, int b)
{
	return a < b ? a : b;
}

void orc_lcd_console_write(orc_t *orc, const char *fmt, ...)
{
	CMDPROLOGUE;

	VPRINTALLOC; // this sets up variables 'buf' and 'len'

	for (int i = 0; i < len; i+=CHUNKSIZE) {
		int thislen = min(len-i, CHUNKSIZE);

		req[PACKET_DATALEN] = 1+thislen;
		req[PACKET_DATA + 0] = CMD_CONSOLE_WRITECHARS;
		memcpy(&req[PACKET_DATA + 1], &buf[i], thislen);

		orc_transaction_once(orc, req, resp);
	}

	free(buf);
}

void orc_lcd_draw_string(orc_t *orc, int x, int y, int font, const char *fmt, ...)
{
	CMDPROLOGUE;

	VPRINTALLOC; // this sets up variables 'buf' and 'len'

	for (int i = 0; i < len; i+=CHUNKSIZE) {
		int thislen = min(len - i, CHUNKSIZE);

		req[PACKET_DATALEN] = thislen + 4;
		req[PACKET_DATA + 0] = CMD_LCD_DRAWCHARS;
		req[PACKET_DATA + 1] = x;
		req[PACKET_DATA + 2] = y;
		req[PACKET_DATA + 3] = font;
		memcpy(&req[PACKET_DATA + 4], &buf[i], thislen);
		
		orc_transaction_once(orc, req, resp);
		x += resp[PACKET_DATA + 1];
	}

	free(buf);
}

void orc_lcd_console_goto(orc_t *orc, int x, int y)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 3;
	req[PACKET_DATA + 0] = CMD_CONSOLE_GOTO;
	req[PACKET_DATA + 1] = x;
	req[PACKET_DATA + 2] = y;
	orc_transaction_retry(orc, req, resp);	
}

void orc_lcd_write(orc_t *orc, int x, int y, uint8_t *data, uint8_t datalen)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 3 + datalen;
	req[PACKET_DATA + 0] = CMD_LCD_WRITE;
	req[PACKET_DATA + 1] = x;
	req[PACKET_DATA + 2] = y;
	memcpy(&req[PACKET_DATA+3], data, datalen);

	orc_transaction_retry(orc, req, resp);	
}

int orc_analog_read(orc_t *orc, int port)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 2;
	req[PACKET_DATA + 0] = CMD_ANALOG_READ;
	req[PACKET_DATA + 1] = port;
	orc_transaction_retry(orc, req, resp);

	return packet_16u(resp, 1);
}

int orc_quadphase_read(orc_t *orc, int port)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 2;
	req[PACKET_DATA + 0] = CMD_QUADPHASE_READ;
	req[PACKET_DATA + 1] = port;
	orc_transaction_retry(orc, req, resp);

	return packet_16u(resp, 1);
}

void orc_motor_set(orc_t *orc, int port, int spwm)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 3;
	req[PACKET_DATA + 0] = CMD_MOTOR_SET;
	req[PACKET_DATA + 1] = port;
	req[PACKET_DATA + 2] = spwm;
	orc_transaction_retry(orc, req, resp);
}

orc_button_state_t orc_pad_begin_poll(orc_t *orc)
{
	orc_button_state_t s;

	s.up = (orc->pad_updown&0xf0)>>4;
	s.down = orc->pad_updown&0x0f;
	s.left = (orc->pad_leftright&0xf0)>>4;
	s.right = orc->pad_leftright&0x0f;	

	return s;
}

int orc_pad_poll(orc_t *orc, orc_button_state_t *s)
{
	int res = orc->pad_switches;

	int tup = (orc->pad_updown&0xf0)>>4;
	int tdown = orc->pad_updown&0x0f;
	int tleft = (orc->pad_leftright&0xf0)>>4;
	int tright = orc->pad_leftright&0x0f;
	
	if (tup != s->up)
		res |= ORC_PAD_UP;
	if (tdown != s->down)
		res |= ORC_PAD_DOWN;
	if (tleft != s->left)
		res |= ORC_PAD_LEFT;
	if (tright != s->right)
		res |= ORC_PAD_RIGHT;
	
	s->up = tup;
	s->down = tdown;
	s->right = tright;
	s->left = tleft;

	return res;
}

int orc_pad_switches(orc_t *orc)
{
	return orc->pad_switches;
}

int orc_pad_gets(orc_t *orc)
{
	orc_button_state_t s = orc_pad_begin_poll(orc);
	int res = 0;

	pthread_mutex_lock(&orc->pad_mutex);
	while ((res = orc_pad_poll(orc, &s))==0) {
		pthread_cond_wait(&orc->pad_cond, &orc->pad_mutex);
	}
	pthread_mutex_unlock(&orc->pad_mutex);

	return res;
}

int orc_in_menu(orc_t *orc)
{
	return (orc->heartbeat_flags&1)!=0;
}

int orc_pad_connected(orc_t *orc)
{
	return (orc->heartbeat_flags&2)!=0;
}

void orc_null(orc_t *orc, int reqlen, int resplen)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 2+reqlen;
	req[PACKET_DATA + 0] = CMD_ORC_NULL;
	req[PACKET_DATA + 1] = resplen;

	orc_transaction_retry(orc, req, resp);	
}

// added by finale
void orc_pinmode_set( orc_t *orc, int port, int mode){

	CMDPROLOGUE;
	
	req[PACKET_DATALEN] = 3;
	req[PACKET_DATA + 0] = CMD_PINMODE_SET;
	req[PACKET_DATA + 1] = port;
	req[PACKET_DATA + 2] = mode;
	orc_transaction_retry(orc, req, resp);
}


// this takes in an int btw -128 and 127
void orc_motor_set_signed( orc_t *orc, int port, int pwm )
{
  // set the signed pwm value
  int spwm  = pwm;
  if( pwm < 0 ){
    spwm = -1 * ( 128 + pwm );  // twos compliment conversion, note pwm is neg
  }
  orc_motor_set( orc, port, spwm );
}


int  orc_sonar_read(orc_t *orc __attribute__ ((unused)), int port __attribute__ ((unused))){
  return -1;
}


int  orc_digital_read(orc_t *orc, int port){
  
        orc_pinmode_set( orc, port, DIGIN );

	CMDPROLOGUE;

	req[PACKET_DATALEN] = 2;
	req[PACKET_DATA + 0] = CMD_DIGIN_READ;
	req[PACKET_DATA + 1] = port;
	orc_transaction_retry(orc, req, resp);

	return packet_16u(resp, 1);
}

void  orc_digital_set(orc_t *orc, int port, int val){
  
        orc_pinmode_set( orc, port, DIGOUT );

	CMDPROLOGUE;
	
	req[PACKET_DATALEN] = 3;
	req[PACKET_DATA + 0] = CMD_DIGOUT_SET;
	req[PACKET_DATA + 1] = port;
	req[PACKET_DATA + 2] = val;
	orc_transaction_retry(orc, req, resp);
}


// between 0 and 1; 1 is full cycle on
void  orc_pwm_set(orc_t *orc, int port, float v){

        orc_pinmode_set( orc, port, PWM );

	CMDPROLOGUE;
	
	int pwm = (int) (v*65535);
	
	req[PACKET_DATALEN] = 4;
	req[PACKET_DATA + 0] = CMD_SERVO_SET;
	req[PACKET_DATA + 1] = port;
	req[PACKET_DATA + 2] = pwm >> 8 ;
	req[PACKET_DATA + 3] = pwm & 0xff ;
	orc_transaction_retry(orc, req, resp);
}

// warning: see orc manual: sets clock for entire pwm bank
void orc_clk_set(orc_t *orc, int port, int divider){

        CMDPROLOGUE;

	int chn = port/4;

	req[PACKET_DATALEN] = 3;
	req[PACKET_DATA + 0] = CMD_PWMCLK_SET;
	req[PACKET_DATA + 1] = chn;
	req[PACKET_DATA + 2] = divider&0xff;
	orc_transaction_retry(orc, req, resp);
}

// reads ticks in 60 Hz windows
int orc_quadphase_read_velocity(orc_t *orc, int port)
{
	CMDPROLOGUE;

	req[PACKET_DATALEN] = 2;
	req[PACKET_DATA + 0] = CMD_QUADPHASE_READ;
	req[PACKET_DATA + 1] = port;
	orc_transaction_retry(orc, req, resp);

	return packet_16u(resp, 3);
}


int orc_quadphase_read_velocity_signed( orc_t *orc, int port )
{
  // set the signed pwm value
  int v  = orc_quadphase_read_velocity( orc, port );
  short real_velocity = v & 0x0000FFFF; 
  return (int) real_velocity;
}
