#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "serial.h"

#include <unistd.h>
#include <errno.h>

struct termios oldtio,newtio;

int openPort( char *serialDev )
{
  int fd;

  printf("Opening port %s\n", serialDev);
  fd = open(serialDev, O_RDWR | O_NOCTTY );
  if (fd <0) {
    perror(serialDev);
    return(1);
  }

  tcgetattr(fd,&oldtio); /* save current port settings */

  bzero(&newtio, sizeof(newtio));
  //newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_cflag &= ~PARENB;
  newtio.c_cflag &= ~CSTOPB;
  newtio.c_cflag &= ~CSIZE;
  newtio.c_cflag |= BAUDRATE | CS8 | CLOCAL | CREAD;

  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  //newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //0;
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME]    = 2;   /* inter-character timer unused .2 second */
  newtio.c_cc[VMIN]     = 0;   /* blocking read until 1 chars received or timeout */

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);

  return fd;
}

void closePort(int fd)
{
  tcsetattr(fd,TCSANOW,&oldtio);
  close(fd);
}


void sendData(int fd, unsigned char *data, int len)
{
   write(fd, data, len);
}

void getData(int fd, unsigned char *data, int *len)
{

  char buf[255];
  int i,j;

  for (j=0; j<10; j++)
  {
    *len = read(fd,buf,255);
    if (*len > 0)
    {
      printf("Got: ");
      for (i=0; i<*len; i++)
        printf("%i ", buf[i]);
      printf("\n");
    }
  }

}

