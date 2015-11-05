#include "s300_laser.h"
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
	s300_laser_t sick;

	unsigned int irange[1024];
	struct timeval timestamp;
	int connected=0;
	
	if (argc!=2){
	  fprintf(stderr, "usage: %s <device>\n", argv[0]);
	  return 0;
	}

	fprintf(stderr, "device is %s\n", argv[1]);
	connected=s300_connect(&sick, argv[1], 500000);
	if (connected<=0)
		return -1;

	while(! has_to_stop){
		unsigned int n_ranges;
		s300_get_next_scan(&sick, &n_ranges, irange, &timestamp);
		fprintf(stderr,"%d\t%d\t%d\n",irange[0],irange[270],irange[540]);		
	}
	
	close(sick.fd);
	return 0;
}
