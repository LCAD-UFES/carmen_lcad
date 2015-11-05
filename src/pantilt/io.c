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

#include <carmen/carmen.h>
#include "pantilt.h"

char *idlechar = "|/-\\";

#ifndef TIOCGETP 

#define TIOCGETP        0x5481
#define TIOCSETP        0x5482
#define RAW             1
#define CBREAK          64

struct sgttyb
{
    unsigned short sg_flags;
    char sg_ispeed;
    char sg_ospeed;
    char sg_erase;
    char sg_kill;
    struct termios t;
    int check;
};

#endif

long 
numChars(int sd)
{
  long available=0;
  if (ioctl(sd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}    

int
writeN(PantiltDeviceType dev, char *buf, int nChars)
{
  int amountWritten = 0;
#ifdef VERBOSE
  int i;
  fprintf( stderr, "-> " );
  for (i=0; i<nChars; i++)
    fprintf( stderr, "%c", buf[i] );
  fflush(stderr);
#endif
  while (nChars > 0) {
    amountWritten = write(dev.fd, buf, nChars);
    if (amountWritten < 0) {
      if (errno == EWOULDBLOCK) {
	fprintf(stderr, "\nWARNING: writeN: EWOULDBLOCK: trying again!\n");
      } else {
	return FALSE;
      }
    }
    else {
      nChars -= amountWritten;
      buf += amountWritten;
    }
  }
  return TRUE;
}

int
checkLine( char *line, int numB )
{
  int i;
#ifndef UNIBONN
  for (i=0; i<numB; i++)
    if (line[i]=='\n') return(1);
  return(0);
#else
  // 
  int endOfEcho = -1;
  for (i=0; i<numB; i++)
    if (line[i]=='*') {
      endOfEcho = i;
    }
  if(endOfEcho >= 0) {
    for (i=endOfEcho; i<numB; i++)
      if (line[i]=='\n') return(1);
  }
  return(0);
#endif
}

int
writeCMD(PantiltDeviceType dev, char *cmd, char *retstr )
{
  struct timeval startTime;
  struct timeval currentTime;
  char scmd[80];
  char buffer[512];
  int val, lval, i, len;
  strcpy( scmd, cmd );
  scmd[strlen(cmd)] = '\n';
  scmd[strlen(cmd)+1] = '\0';
  gettimeofday(&startTime, NULL);
  writeN( dev, scmd, strlen(scmd) );
  lval=0;
  while( !checkLine( buffer, lval ) ) {
    gettimeofday(&currentTime, NULL);
    if ( !( currentTime.tv_sec < startTime.tv_sec + MAX_CMD_TIME || 
	    ( currentTime.tv_sec == startTime.tv_sec + MAX_CMD_TIME &&
	      currentTime.tv_usec < startTime.tv_usec  ) ) ) {
      break;
    } else if ((val=numChars(dev.fd))) {
      val = read(dev.fd,&buffer[lval], 512);
      lval = lval+val;
    }
    usleep(20000);
  }
  buffer[lval]='\0';
#ifdef VERBOSE
  fprintf( stderr, "<- %s\n", buffer );
  fflush(stderr);
#endif
  sscanf(buffer, "%s", scmd );
  if (strlen(buffer)>3) {
    len = (int)(strlen(buffer)-2);
    for(i=2;i<len;i++)
      retstr[i-2] = buffer[i];
    retstr[i-2] = '\0';
  } else
    strcpy(retstr, "");
  if (!strcmp(scmd,"*"))
    return CMD_OK;
  else
    return CMD_ERROR;
}


void 
m_setrts( int fd )
{
  if(fd){}

#if defined(TIOCM_RTS) && defined(TIOCMODG)
  int mcs=0;
  ioctl(fd, TIOCMODG, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(fd, TIOCMODS, &mcs);
#endif
#ifdef _COHERENT
  ioctl(fd, TIOCSRTS, 0);
#endif
  fd = 0;
}

#define _POSIX

void 
m_setparms( int fd, int baud, char par, int bits, int hwf, int swf )
{
  int spd = -1;

#ifdef _POSIX
  struct termios tty;
  tcgetattr(fd, &tty);
#else
  struct sgttyb tty;
  ioctl(fd, TIOCGETP, &tty);
#endif

  /* We generate mark and space parity ourself. */
  if (bits == 7 && (par == 'M' || par == 'S'))
    bits = 8;
  /* Check if 'baudr' is really a number */

  switch(baud) {
  case 0:
#ifdef B0
		spd = B0;	break;
#else
		spd = 0;	break;
#endif
  case 3:	spd = B300;	break;
  case 6:	spd = B600;	break;
  case 12:	spd = B1200;	break;
  case 24:	spd = B2400;	break;
  case 48:	spd = B4800;	break;
  case 96:	spd = B9600;	break;
#ifdef B19200
  case 192:	spd = B19200;	break;
#else
#ifdef EXTA
  case 192:	spd = EXTA;	break;
#else
  case 192:	spd = B9600;	break;
#endif	
#endif	
#ifdef B38400
  case 384:	spd = B38400;	break;
#else
#ifdef EXTB
  case 384:	spd = EXTB;	break;
#else
  case 384:	spd = B9600;	break;
#endif
#endif	
#ifdef B57600
  case 576:	spd = B57600;	break;
#endif
#ifdef B115200
  case 1152:	spd = B115200;	break;
#endif
  }
  
#if defined (_BSD43) && !defined(_POSIX)
  if (spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  /* Number of bits is ignored */
  tty.sg_flags = RAW | TANDEM;
  if (par == 'E')
	tty.sg_flags |= EVENP;
  else if (par == 'O')
	tty.sg_flags |= ODDP;
  else
  	tty.sg_flags |= PASS8 | ANYP;
  ioctl(fd, TIOCSETP, &tty);
#ifdef TIOCSDTR
  /* FIXME: huh? - MvS */
  ioctl(fd, TIOCSDTR, 0);
#endif
#endif

#if defined (_V7) && !defined(_POSIX)
  if (spd != -1) tty.sg_ispeed = tty.sg_ospeed = spd;
  tty.sg_flags = RAW;
  if (par == 'E')
	tty.sg_flags |= EVENP;
  else if (par == 'O')
	tty.sg_flags |= ODDP;
  ioctl(fd, TIOCSETP, &tty);
#endif

#ifdef _POSIX
  if (spd != -1) {
	cfsetospeed(&tty, (speed_t)spd);
	cfsetispeed(&tty, (speed_t)spd);
  }
  switch (bits) {
  case 5:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
    break;
  case 6:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
    break;
  case 7:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
    break;
  case 8:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    break;
  default:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    break;
  }		
  /* Set into raw, no echo mode */
#if !defined(_DGUX_SOURCE) && !defined(__APPLE__)
  tty.c_iflag &= ~(IGNBRK | IGNCR | INLCR | ICRNL | IUCLC | 
  	IXANY | IXON | IXOFF | INPCK | ISTRIP);
  tty.c_iflag |= (BRKINT | IGNPAR);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag = ~(ICANON | ISIG | ECHO | ECHONL | ECHOE | ECHOK | IEXTEN);
  tty.c_cflag |= CREAD | CRTSCTS;
#else /* Okay, this is better. XXX - Fix the above. */
  tty.c_iflag =  IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cflag |= CLOCAL | CREAD;
#endif
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;

  /* Flow control. */
  if (hwf) {
    tty.c_cflag |= CRTSCTS;
    tty.c_cflag &= ~CLOCAL;
  }
  else {
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL;
  }

  if (swf) {
    tty.c_iflag |= (IXON | IXOFF);
  }
  else {
    tty.c_iflag &= ~(IXON | IXOFF);
  }

  tty.c_cflag &= ~(PARENB | PARODD);

  if (par == 'E')
	tty.c_cflag |= PARENB;
  else if (par == 'O')
	tty.c_cflag |= PARODD;

  tcsetattr(fd, TCSANOW, &tty);

  m_setrts(fd);
#  ifdef _DGUX_SOURCE
  m_sethwf(fd, hwf);
#  endif
#endif
}

int
connectTTY(PantiltDeviceType * dev)
{
  static struct termios newtio;
  /* 0x8000 ist SYNC */  
  if ( (dev->fd = open( (dev->port), (O_RDWR | 0x8000 | O_NOCTTY),0)) < 0 ) {
    return (dev->fd);
  }
  
  tcgetattr((dev->fd),&newtio);
  
  newtio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  newtio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF);
  newtio.c_cflag &= ~(CSIZE | PARENB | PARODD);
  newtio.c_cflag |= (CS8);       
  newtio.c_oflag &= ~(OPOST);
  
  newtio.c_cc[VTIME] = 1;      
  newtio.c_cc[VMIN] = 0;       
  
  tcflush((dev->fd), TCIFLUSH);
  tcsetattr((dev->fd), TCSANOW, &newtio);
  
  return (dev->fd);
}

int
ClearInputBuffer( PantiltDeviceType* pDevice )
{
  unsigned char buffer[4096];
  int val=0;
  val = numChars(pDevice->fd);
  if (val>0) {
    read(pDevice->fd,&buffer,val);
  }
  return(val);
}

void
ProcessLine( char *line, int length __attribute__ ((unused)) )
{
  char command[64];
  sscanf( line, "%s", command );
  if (!strcmp(command,"PP")) {
    fprintf( stderr, "<- %s", line );
  } else if (!strcmp(command,"TP")) {
  }
//  fprintf( stderr, "## %s (%d)", line, length );
}

int
PantiltInitializeDevice( PantiltDeviceType * pDevice )
{
  fprintf( stderr, "connect TTY %s ... ", pDevice->port );
  connectTTY( pDevice );
  if( pDevice->fd == -1) {
    fprintf( stderr, " failed\n" );
    return FALSE;
  }
  fprintf( stderr, " ok\n" );
  m_setparms( pDevice->fd, pDevice->baud, pDevice->parity,
	      pDevice->bits, pDevice->hwf, pDevice->swf );
  sleep(1);
  return TRUE;
}

int
prad2ticks( double radians )
{
  return((int) -(floor((double)
		       (rad2deg(radians)*3600.0/(double) pSettings.res_pan)+
		       pSettings.toff_pan )));
}

int
pdeg2ticks( double degrees )
{
  return(prad2ticks(deg2rad(degrees)));
}

int
trad2ticks( double radians )
{
  return((int) (floor((double)
		      (rad2deg(radians)*3600.0/(double) pSettings.res_tilt)+
		      pSettings.toff_tilt)));
}

int
tdeg2ticks( double degrees )
{
  return(trad2ticks(deg2rad(degrees)));
}

double
pticks2rad( int ticks )
{
  return(deg2rad(-floor((double)((ticks+pSettings.toff_pan)*
				 pSettings.res_pan/3600.0))));
}

double
pticks2deg( int ticks )
{
  return(rad2deg(pticks2rad(ticks)));
}

double
tticks2deg( int ticks )
{
  return(rad2deg(tticks2rad(ticks)));
}

double
tticks2rad( int ticks )
{
  return(deg2rad(floor((double)((ticks-pSettings.toff_tilt)*
				pSettings.res_tilt/3600.0))));
}

void
set_pan( double pan )
{
  char buffer[512], command[80];
  int pticks =  prad2ticks( pan );
  if (pticks<pSettings.min_pan)
    pticks = pSettings.min_pan;
  if (pticks>pSettings.max_pan)
    pticks = pSettings.max_pan;
  sprintf( command, "PP%d", pticks );
  if (writeCMD(pDevice,command,buffer)==CMD_OK)
    pPos.pan = pticks;
}

void
set_tilt( double tilt )
{
  char buffer[512], command[80];
  int tticks = trad2ticks( tilt );
  if (tticks<pSettings.min_tilt)
    tticks = pSettings.min_tilt;
  if (tticks>pSettings.max_tilt)
    tticks = pSettings.max_tilt;
  sprintf( command, "TP%d", tticks );
  if (writeCMD(pDevice,command,buffer)==CMD_OK)
    pPos.tilt = tticks;
} 

void
set_pantilt( double pan, double tilt )
{
  set_pan( pan );
  set_tilt( tilt );
}

