#include <stdio.h>
 #include <sys/time.h>
#include <carmen/carmen.h>
#include "carmenserial.h"
#include <sys/ioctl.h>

int fd;

void
cleanup(int signo)
{
  if (signo == SIGINT)
  {
	carmen_serial_close(fd);
    exit(0);
  }
}

int setCTS(int fd, int value) {
    int status;
    ioctl(fd, TIOCMGET, &status); // get the current port status
    if (value)
    	status |= TIOCM_CTS; // rise the CTS bit
    else
    	status &= ~TIOCM_CTS; // drop the CTS bit
    ioctl(fd, TIOCMSET, &status); // set the modified status
    return 0;
}

int main()
{
	//int cts = 0;
	//int ant = 0;
	//int count = 0;
	char* device_name = "/dev/ttyUSB0";
	struct timeval tim;
	//double diff_time = 0;
	int i = 0;
	int sercmd = TIOCM_DTR;
	//int bit_chain[1000000];

	signal(SIGINT, cleanup);


	if(!carmen_serial_connect(&fd, device_name))
	{
		carmen_serial_configure(fd, 115200, "O");

		gettimeofday(&tim, NULL);
		double t1 = (tim.tv_sec * 1000) + (tim.tv_usec / 1000.0);
		double t2;
		while (1)
		{
			//ant = cts;
			//cts = carmen_serial_read_cts(fd);
			//bit_chain[count] = cts;
			//count++;
			gettimeofday(&tim, NULL);
			t2 = (tim.tv_sec * 1000) + (tim.tv_usec / 1000.0);

			//if (ant != cts) {
				//diff_time = t2-t1;
				//printf("%d\n",cts);
				//t1 = t2;
			//}
			//ioctl(fd, TIOCMBIC, &sercmd); continue;
			if (i%2) {
				//setCTS(fd,1);
				ioctl(fd, TIOCMBIS, &sercmd); // Set the pin.	
			}else {
				//setCTS(fd,0);
				ioctl(fd, TIOCMBIC, &sercmd); // Reset the pin.
			}

			if (t2 - t1 >= 2000.0)
			{
/*
				int i;
				for(i=0; i < count; i++)
					printf("%d", bit_chain[i]);

				printf("\n\n\n");
				printf("%d\n",count);
				count=0;
*/
				i++;
				printf("%d\n",i);
				//ioctl(fd, TIOCMBIS, &sercmd); // Set the pin.
				t1 = t2;
			}

			
		}
	}
	else
	{
		printf("cannot open device!\n");
	}

	return 0;
}
