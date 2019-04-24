#include "sick_laser.h"
#include <unistd.h>
#include <signal.h>
#include <stdio.h>

int has_to_stop;

void sigquit_handler(int q __attribute__((unused))){
	has_to_stop=1;
}

int main(int argc, char** argv){
	has_to_stop=0;
	signal(SIGINT, sigquit_handler);
	//unsigned char buffer[8192];
	sick_laser_t sick;
	//int reply;
	int config_retries=5;
	//int connect_retries=5;
	int i;

	int connected=0;
	int configured=0;
	//	int monitoringMode=0;
	
	if (argc!=2){
	  fprintf(stderr, "usage: %s <device>\n", argv[0]);
	  return 0;
	}
	i=0;
	fprintf(stderr, "device is %s\n", argv[1]);
	connected=sick_connect(&sick, argv[1], 500000);
	if (connected<=0)
		return -1;

	i=0;
	do{
		configured=sick_configure(&sick, 0, 80, 0, 100, 25);
		i++;
	}while (configured<=0 && i<config_retries);
	return 0;
}
