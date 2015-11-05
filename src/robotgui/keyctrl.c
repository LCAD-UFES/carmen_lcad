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

#ifndef KEYCTRL_H
#define KEYCTRL_H

#include <carmen/carmen.h>
#include <sys/ioctl.h>


#ifdef CYGWIN
void cfmakeraw(struct termios *termios_p)
{
  termios_p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  termios_p->c_oflag &= ~OPOST;
  termios_p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  termios_p->c_cflag &= ~(CSIZE|PARENB);
  termios_p->c_cflag |= CS8;
}
#endif

void carmen_initialize_keyboard(void)
{
  struct termios term_struct;
  int flags;
  tcflag_t oflags;

  flags = fcntl(STDIN_FILENO, F_GETFL);           /* initialize asyncronous */
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);    /* keyboard input */
  tcgetattr(0, &term_struct);
  oflags = term_struct.c_oflag;
  cfmakeraw(&term_struct);
  term_struct.c_oflag = oflags;
  term_struct.c_lflag |= ISIG;
  tcsetattr(0, TCSANOW, &term_struct);
}

int carmen_read_char(char *c)
{
  long available;
  int i;

  ioctl(0, FIONREAD, &available);

  if(available > 0) {
    for(i = 0; i < available; i++)
      read(0, c, 1);
    return 1;
  }
  else
    return 0;
}

int carmen_keyboard_control(char c, double max_tv, double max_rv, 
			    double *tv, double *rv)
{
  int quit = 0;
  if(c >= 'A' && c <= 'Z')
    c += 32;
  switch(c) {
  case 'i':
    *tv = max_tv;
    *rv = 0;
    break;
  case 'u':
    *tv = max_tv;
    *rv = max_rv;
    break;
  case 'o':
    *tv = max_tv;
    *rv = -max_rv;
    break;
  case 'j':
    *tv = 0;
    *rv = max_rv;
    break;
  case 'l':
    *tv = 0;
    *rv = -max_rv;
    break;
  case ',':
    *tv = -max_tv;
    *rv = 0;
    break;
  case 'm':
    *tv = -max_tv;
    *rv = -max_rv;
    break;
  case '.':
    *tv = -max_tv;
    *rv = max_rv;
    break;
  case 'q':
    quit = -1;
    break;
  default:
    *tv = 0;
    *rv = 0;
    break;
  }
  if(quit >= 0)
    return 0;
  return -1;
}

#endif
