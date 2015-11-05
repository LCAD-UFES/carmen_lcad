#include "hokuyourg.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main (int argc, const char** argv){
  if (argc<2){
    printf( "Usage: urg_test <device> ");
    return 0;
  }
  char buf[URG_BUFSIZE];
  HokuyoURG urg;
  int o=hokuyo_open(&urg,argv[1]);
  if (o<=0)
    return -1;
  o=hokuyo_init(&urg);
  if (o<=0)
    return -1;
  o=hokuyo_startContinuous(&urg, 0, 768, 0);
  if (o<=0){
    return -1;
  }
  //double astep=2*M_PI/1024;
  int k=0;
  while (k<20){
    hokuyo_readPacket(&urg, buf, URG_BUFSIZE,10);
    HokuyoRangeReading reading;
    hokuyo_parseReading(&reading, buf);

    //if we  get too much maxranges, restart the laser
    int validBeamsCount=0;
    for (int i=0; i<reading.n_ranges; i++){
      if (reading.ranges[i]>20 && reading.ranges[i]< 5601)
	validBeamsCount++;
    }

    fprintf(stderr, "ts=%d, ss=%d, es=%d, cc=%d, nr=%d, st=%d vbc=%d\n", 
	    reading.timestamp, 
	    reading.startStep, 
	    reading.endStep, 
	    reading.clusterCount, 
	    reading.n_ranges, 
	    reading.status,
	    validBeamsCount);

//     cout << "set size ratio -1" << endl;
//     cout << "plot [-5600:5600][-5600:5600]'-' w l" << endl;
//     for (int i=0; i<reading.n_ranges; i++){
//       double alpha=(-135./180.)*M_PI+astep*i;
//       cout << reading.ranges[i] * cos(alpha) << " " << reading.ranges[i] * sin (alpha) << endl; 
//     }
//     cout << "e" << endl << endl;
    k++;
  }
  hokuyo_close(&urg);
}
