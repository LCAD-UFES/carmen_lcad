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

#ifndef _ORC_H
#define _ORC_H

#include <pthread.h>
#include <stdint.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct orc_comms_impl orc_comms_impl_t;

typedef struct 
{
	int up, down, left, right;
} orc_button_state_t;

struct orc_comms_impl
{
	int (*connect)(orc_comms_impl_t *impl);
	void (*disconnect)(orc_comms_impl_t *impl, int fd);
	void *_p;
};

typedef struct
{
	uint8_t inuse;
	pthread_cond_t cond;
	pthread_mutex_t mutex;
	uint8_t *response;
} transaction_t;

typedef struct
{
	int fd;
	orc_comms_impl_t *impl;
	transaction_t transactions[256];
	pthread_mutex_t writeLock;

	// The minimum and maximum id #s that we're allowed to use
	int idLow; 
	int idHigh;
	// The number of ids we're using currently
	int idUsed;
	// The last id we used-- a hint where should we start searching for the next id [idLow, idHigh]?
	int idLast;

	pthread_mutex_t idMutex;
	pthread_cond_t idCond;

	int pad_switches;
	int pad_updown;
	int pad_leftright;
	int pad_joyx;
	int pad_joyy;
	pthread_mutex_t pad_mutex;
	pthread_cond_t pad_cond;

	int heartbeat_time;
	int heartbeat_flags;
} orc_t;


// ---- low level stuff ---- //
orc_t *orc_create(orc_comms_impl_t *impl);
void orc_destroy(orc_t *orc);
void orc_transaction_retry(orc_t *orc, uint8_t *request, uint8_t *response);
int  orc_transaction_once(orc_t *orc, uint8_t *request, uint8_t *response);
void orc_transaction_async(orc_t *orc, uint8_t *request);

orc_comms_impl_t *orc_rawprovider_create(char *port);
orc_comms_impl_t *orc_tcpprovider_create(char *hostname, int port);


// ---- orc board --- //
#define DIGIN 0
#define DIGOUT 1
#define PWM 3

void orc_pinmode_set( orc_t *orc, int port, int mode);

int  orc_quadphase_read(orc_t *orc, int port);

// reads ticks in 60 Hz windows -- be aware of overflow and underflow
// the output is 7 bits plus a sign
int  orc_quadphase_read_velocity(orc_t *orc, int port);
int  orc_quadphase_read_velocity_signed(orc_t *orc, int port);

int  orc_analog_read(orc_t *orc, int port);
int  orc_digital_read(orc_t *orc, int port);
void orc_digital_set(orc_t *orc, int port, int val);

// not currently implemented!!!
int  orc_sonar_read(orc_t *orc, int port);

// pwm is a 7 bit value + sign bit = 8 bits total
// note: this is not the same as -128 to 127!
void orc_motor_set( orc_t *orc, int port, int spwm );

// this takes in an int btw -128 and 127
void orc_motor_set_signed( orc_t *orc, int port, int pwm );

// should be between 0 and 1 (1 is full duty cycle on)
void orc_pwm_set(orc_t *orc, int port, float v);

// warning: see orc manual: sets clock for entire pwm bank
void orc_clk_set(orc_t *orc, int port, int divider);



// ---- orc pad ---- //
void orc_lcd_clear(orc_t *orc);
void orc_lcd_console_home(orc_t *orc);
void orc_lcd_console_write(orc_t *orc, const char *fmt, ...);
void orc_lcd_console_goto(orc_t *orc, int x, int y);
void orc_lcd_draw_string(orc_t *orc, int x, int y, int font, const char *fmt, ...);

void orc_lcd_write(orc_t *orc, int x, int y, uint8_t *data, uint8_t datalen);
void orc_null(orc_t *orc, int reqlen, int resplen);

/** Sample the state of the buttons on the orcpad. (Note this will
    never return up/down/left/right). **/
int orc_pad_switches(orc_t *orc);

/** Wait for a button to be pressed. **/
int orc_pad_gets(orc_t *orc);

/** Returns 1 if the user is in the built-in menu. **/
int orc_in_menu(orc_t *orc);
int orc_pad_connected(orc_t *orc);

orc_button_state_t orc_pad_begin_poll(orc_t *orc);

int orc_pad_poll(orc_t *orc, orc_button_state_t *state);

#define ORC_PAD_STOP 1
#define ORC_PAD_A 2
#define ORC_PAD_B 4
#define ORC_PAD_STICK 8
#define ORC_PAD_UP 16
#define ORC_PAD_DOWN 32
#define ORC_PAD_LEFT 64
#define ORC_PAD_RIGHT 128

#ifdef __cplusplus
}
#endif

#endif
