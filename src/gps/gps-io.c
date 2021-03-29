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

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_messages.h>

#include "gps.h"
#include "gps-io.h"

void
DEVICE_init_params( SerialDevice *p )
{
  strncpy( p->ttyport, DEFAULT_GPS_PORT, MAX_NAME_LENGTH );
  p->baud                    = DEFAULT_GPS_BAUD;
  p->parity                  = DEFAULT_GPS_PARITY;
  p->databits                = DEFAULT_GPS_DATABITS;
  p->stopbits                = DEFAULT_GPS_STOPBITS;
  p->hwf                     = DEFAULT_GPS_HWF;
  p->swf                     = DEFAULT_GPS_SWF;
  p->fd                      = -1;
}

int
iParity( enum PARITY_TYPE par )
{
  if (par==NO)
    return(IGNPAR);
  else
    return(INPCK);
}

int
iSoftControl( int flowcontrol )
{
  if (flowcontrol)
    return(IXON);
  else
    return(IXOFF);
}

int
cDataSize( int numbits )
{
  switch(numbits) {
  case 5:
    return(CS5);
    break;
  case 6:
    return(CS6);
    break;
  case 7:
    return(CS7);
    break;
  case 8:
    return(CS8);
    break;
  default:
    return(CS8);
    break;
  }
}

int
cStopSize( int numbits )
{
  if (numbits==2) {
    return(CSTOPB);
  } else {
    return(0);
  }
}

int
cFlowControl( int flowcontrol )
{
  if (flowcontrol) {
    return(CRTSCTS);
  } else {
    return(CLOCAL);
  }
}

int
cParity( enum PARITY_TYPE par )
{
  if (par!=NO) {
    if (par==ODD) {
      return(PARENB | PARODD);
    } else {
      return(PARENB);
    }
  } else {
    return(0);
  }
}

int
cBaudrate( int baudrate )
{
  switch(baudrate) {
  case 0:
    return(B0);
    break;
  case 300:
    return(B300);
    break;
  case 600:
    return(B600);
    break;
  case 1200:
    return(B1200);
    break;
  case 2400:
    return(B2400);
    break;
  case 4800:
    return(B4800);
    break;
  case 9600:
    return(B9600);
    break;
  case 19200:
    return(B19200);
    break;
  case 38400:
    return(B38400);
    break;
  case 57600:
    return(B57600);
    break;
  case 115200:
    return(B115200);
   case 230400:
    return(B230400);
  
    break;
#if !defined(CYGWIN) && !defined(__APPLE__)
  case 500000:
    /* to use 500k you have to change the entry of B460800 in you kernel:
       /usr/src/linux/drivers/usb/serial/ftdi_sio.h:
       ftdi_8U232AM_48MHz_b460800 = 0x0006    */
    return(B460800);
    break;
#endif
  default:
    return(B9600);
    break;
  }

}

int 
DEVICE_bytes_waiting( int sd )
{
  int available=0;
  if ( ioctl( sd, FIONREAD, &available ) == 0 )
    return available;
  else
    return -1;
}    

void 
DEVICE_set_params_old( SerialDevice dev )
{
  struct termios  ctio;

  tcgetattr(dev.fd, &ctio); /* save current port settings */

  ctio.c_iflag = iSoftControl(dev.swf) | iParity(dev.parity);
  ctio.c_oflag = 0;
  ctio.c_cflag =
    CREAD                            |
    cFlowControl(dev.hwf || dev.swf) |
    cParity(dev.parity)              | 
    cDataSize(dev.databits)          |
    cStopSize(dev.stopbits);

  ctio.c_lflag = 0;
  ctio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
  ctio.c_cc[VMIN]     = 0;   /* blocking read until 0 chars received */

  cfsetispeed ( &ctio, (speed_t) cBaudrate(dev.baud) );
  cfsetospeed ( &ctio, (speed_t) cBaudrate(dev.baud) );

  tcflush(dev.fd, TCIFLUSH);
  tcsetattr(dev.fd,TCSANOW,&ctio);

}

void 
DEVICE_set_params(SerialDevice dev)
{
	struct termios ctio;

	tcgetattr(dev.fd, &ctio); /* get current port settings */

	cfsetispeed (&ctio, (speed_t) cBaudrate(dev.baud));
	cfsetospeed (&ctio, (speed_t) cBaudrate(dev.baud));

	// Enable the receiver and set local mode
	ctio.c_cflag |= (CLOCAL | CREAD);

	// Set character size, parity  and stop bits
	ctio.c_cflag &= ~(CSIZE|PARENB);
	ctio.c_cflag |= cParity(dev.parity);
	ctio.c_cflag |= cDataSize(dev.databits);
	ctio.c_cflag |= cStopSize(dev.stopbits);

	// Disable hardware flow control
	ctio.c_cflag &= ~CRTSCTS;
	ctio.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);

	// Disable software flow control
	ctio.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);

	// Set Raw output
	ctio.c_oflag &= ~OPOST;

	// Blocking read until 1 chars received. Inter-character timer unused.
	ctio.c_cc[VMIN]  = 1;
	ctio.c_cc[VTIME] = 0;

	// Set the new options for the port
	tcsetattr(dev.fd, TCSANOW, &ctio);
	tcflush(dev.fd, TCIOFLUSH);
}

void 
DEVICE_set_baudrate( SerialDevice dev, int brate )
{
  struct termios  ctio;

  tcgetattr(dev.fd,&ctio); /* save current port settings */

  cfsetispeed ( &ctio, (speed_t) cBaudrate(brate) );
  cfsetospeed ( &ctio, (speed_t) cBaudrate(brate) );

  tcflush(dev.fd, TCIFLUSH);
  tcsetattr(dev.fd,TCSANOW,&ctio);
}

int
DEVICE_connect_port( SerialDevice *dev )
{
	if ((dev->fd = open(dev->ttyport, O_RDWR | O_NOCTTY)) < 0)
		return (-1);

	fprintf(stderr, "INFO: set device:\n" );
	fprintf(stderr, "INFO:    port   = %s\n", dev->ttyport );
	fprintf(stderr, "INFO:    baud   = %d\n", dev->baud );
	fprintf(stderr, "INFO:    params = %d%s%d\n", dev->databits, dev->parity==NO?"N":dev->parity==ODD?"O":"E", dev->stopbits);

	DEVICE_set_params(*dev);

	return (dev->fd);
}


int
writeData( int fd, unsigned char *buf, int nChars )
{
  int written = 0;
  while (nChars > 0) {
    written = write( fd, buf, nChars );
    if (written < 0) {
      return FALSE;
    } else {
      nChars -= written;
      buf    += written;
    }
    usleep(1000);
  }
  return TRUE;
}

int
DEVICE_send( SerialDevice dev, unsigned char *cmd, int len )
{
#ifdef IO_DEBUG
  int i;
  fprintf( stderr, "\n---> " );
  for (i=0;i<len;i++) {
    fprintf( stderr, "%c", cmd[i] );
  }
  fprintf( stderr, "\n" );
#endif

  if (writeData( dev.fd, cmd, len )) {
    return(TRUE);
  } else {
    return(FALSE);
  }
}


int
DEVICE_read_data(SerialDevice dev)
{
	int chars_read;
	static char buffer[BUFFER_LENGTH];
	char *temp_buffer;
	static size_t buffer_pos = 0;
	static size_t j;
	int chars_available;
	int start, end, data_ok;
//	static double previous_time = 0.0;
//	double time;

	chars_available = DEVICE_bytes_waiting(dev.fd);
	if (chars_available > 0)
	{
		if ((chars_available + buffer_pos) >= BUFFER_LENGTH)
		{
			temp_buffer = (char *) malloc(chars_available * sizeof(char));
			carmen_test_alloc(temp_buffer);
			read(dev.fd, temp_buffer, chars_available);
			free(temp_buffer);
			buffer_pos = 0;
			return (FALSE);
		}
				
		chars_read = read(dev.fd, &(buffer[buffer_pos]), chars_available);
		if (chars_read > 0)
		{
			buffer_pos += chars_read;
			data_ok = FALSE;

			while (buffer_pos > 100) // 100 eh ~ tamanho de uma linha NMEA. Le e trata todas do buffer ateh este tamanho.
			{
				for (j = 0; j < buffer_pos; j++)
					if (buffer[j] == '$')
						break;
				start = j;

				for ( ; j < buffer_pos; j++)
					if (buffer[j] == '*')
						break;
				end = j;
				
				if ((buffer[end] == '*') && ((end - start) > 0))
				{
//					 buffer[end] = '\0';
//					 printf("%s   , %d, %d\n", &(buffer[start]), chars_available, buffer_pos);

					if (carmen_gps_parse_data(&(buffer[start]), end - start))
						data_ok = TRUE;
					for (j = (end + 1); j < buffer_pos; j++)
					{
						buffer[j - (end + 1)] = buffer[j];
					}
					buffer_pos = j - (end + 1);

	/*				time = carmen_get_time();
					printf("delta_t %lf, time %lf, previous_time %lf, bp = %d\n", time - previous_time, time, previous_time, buffer_pos);
					previous_time = time;
	*/
				}
			}

			return (data_ok);
		}
	}

	return (FALSE);
}
