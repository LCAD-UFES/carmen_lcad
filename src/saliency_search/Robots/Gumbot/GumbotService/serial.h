
#ifndef SERIAL_H
#define SERIAL_H

#define BAUDRATE B57600

extern struct termios oldtio,newtio;

int openPort( char *serialDev );
void closePort(int fd);
void sendData(int fd, unsigned char *data, int len);
void getData(int fd, unsigned char *data, int *len);

#endif
