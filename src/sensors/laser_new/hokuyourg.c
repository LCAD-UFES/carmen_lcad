#include "hokuyourg.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//parses an int of x bytes in the hokyo format
unsigned int parseInt(int bytes, char** s){
  char* b=*s;
  unsigned int ret=0;
  int j=bytes-1;
  for (int i=0; i<bytes;){
    if (*b==0||*b=='\n'){
      *s=0;
      return 0;
    }
    if (*(b+1)=='\n'){ //check for a wrapped line
      b++;
    } else {
      unsigned char c=*b-0x30;
      ret+=((unsigned int ) c)<<(6*j);
      i++;
      j--;
    }
    b++;
  }
  *s=b;
  return ret;
}


//skips a line
carmen_inline char* skipLine(char* buf){
  while (*buf!=0 && *buf!='\n')
    buf++;
  return (*buf=='\n')?buf+1:0;
}

//parses a reading response
void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer){
  char* s=buffer;
  int expectedStatus=0;
  if (s[0]=='M')
    expectedStatus=99;
  if (s[0]=='C')
    expectedStatus=00;
  
  int beamBytes=0;
  if (s[1]=='D') beamBytes=3;
  if (s[0]=='C') beamBytes=3;
  
  if (! beamBytes || ! expectedStatus){
    fprintf(stderr, "Invalid return packet, cannot parse reading\n");
    r->status=-1;
    return;
  }
  s+=2;
  char v[5];
  v[4]=0;
  strncpy(v,s,4); r->startStep=atoi(v); s+=4;
  strncpy(v,s,4); r->endStep=atoi(v);   s+=4;
  v[2]=0; strncpy(v,s,2); r->clusterCount=atoi(v);
  
  s=skipLine(s);
  if (s==0){
    fprintf(stderr, "error, line broken when reading the range parameters\n");
    r->status=-1;
    return;
  }

  strncpy(v,s,2); r->status=atoi(v); s+=2;

  if (r->status==expectedStatus){
  } else {
    fprintf(stderr,"Error, Status=%d",r->status);
    return;
  }
  r->timestamp=parseInt(4,&s);
  s=skipLine(s);

  int i=0;
  while(s!=0){
    r->ranges[i++]=parseInt(beamBytes,&s);
  }
  i--;
  r->n_ranges=i;
}




unsigned int hokuyo_readPacket(HokuyoURG* urg, char* buf, int bufsize, int faliures){
  int failureCount=faliures;
  if (urg->fd<=0){
    fprintf(stderr, "Invalid urg->fd\n");
    return -1;
  }

  memset(buf, 0, bufsize);
  int wasLineFeed=0;
  char* b=buf;
  while (1){
    int c=read(urg->fd, b, bufsize);
    if (! c){
      fprintf(stderr, "null" );
      usleep(25000);
      failureCount--;
    }else {
      for (int i=0; i<c; i++){
	if (wasLineFeed && b[i]=='\n'){
	  b++;
	  return b-buf;
	}
	wasLineFeed=(b[i]=='\n');
      }
      b+=c;
    }
    if (failureCount<0)
      return 0;
  }
}

unsigned int hokuyo_readStatus(HokuyoURG* urg, char* cmd){
  char buf[URG_BUFSIZE];
  write (urg->fd,  cmd, strlen(cmd));
  while (1){
    int c=hokuyo_readPacket(urg, buf, URG_BUFSIZE,10);
    if (c>0 && !strncmp(buf,cmd+1,strlen(cmd)-1)){
      char*s=buf;
      s=skipLine(s);
      char v[3]={s[0], s[1], 0};
      return atoi(v);
    }
  }
  return 0;
    
}


#define HK_QUIT  "\nQT\n"
#define HK_SCIP  "\nSCIP2.0\n"
#define HK_BEAM  "\nBM\n"
#define HK_RESET "\nRS\n"
#define wcm(cmd) write (urg->fd,  cmd, strlen(cmd))



int hokuyo_open(HokuyoURG* urg, const char* filename){
  urg->isProtocol2=0;
  urg->isInitialized=0;
  urg->isContinuous=0;
  urg->fd=open(filename, O_RDWR| O_NOCTTY | O_SYNC);
  return urg->fd;
}

int hokuyo_init(HokuyoURG* urg){
  if (urg->fd<=0){
    return -1;
  }

#ifdef HOKUYO_ALWAYS_IN_SCIP20
  fprintf(stderr, "\nAssuming laser is already in SCIP2.0 mode!\n"); 
  fprintf(stderr, " (if the dirver hangs, either your Hokuyo has the old\n");
  fprintf(stderr, " firmware or you have to disable -DHOKUYO_ALWAYS_IN_SCIP20\n");
  fprintf(stderr, " in the Makefile of the laser driver)\n");

  urg->isContinuous=0;  
  urg->isProtocol2=1;
#else

  // stop the  device anyhow

  fprintf(stderr, "Stopping the device... "); 
  wcm(HK_QUIT);
  wcm(HK_QUIT);
  wcm(HK_QUIT);
  fprintf(stderr, "done\n"); 
  urg->isContinuous=0;
  
  // put the urg in SCIP2.0 Mode
  fprintf(stderr, "Switching to enhanced mode (SCIP2.0)... "); 
  fprintf(stderr, " (if the dirver hangs you probably configured your Hokuyo\n");
  fprintf(stderr, " so that SCIP is alwas on. IN this case, enable the Option\n");
  fprintf(stderr, " CFLAGS += -DHOKUYO_ALWAYS_IN_SCIP20\n");
  fprintf(stderr, " in the Makefile of the laser driver)\n");



  int status=hokuyo_readStatus(urg, HK_SCIP);
  if (status==0){
    fprintf(stderr, "\nSwitching to SCIP 2.0 was successful!\n");
    urg->isProtocol2=1;
  } else {
    fprintf(stderr, "Error. Unable to switch to SCIP2.0 Mode, please upgrade the firmware of your device.\n");
    return -1;
  }

#endif

  fprintf(stderr, "Device initialized successfully\n");
  urg->isInitialized=1;
  return 1;
}

int hokuyo_startContinuous(HokuyoURG* urg, int startStep, int endStep, int clusterCount){
  if (! urg->isInitialized)
    return -1;
  if (urg->isContinuous)
    return -1;

  // switch on the laser
  fprintf(stderr, "Switching on the laser emitter...  "); 
  int status=hokuyo_readStatus(urg, HK_BEAM);
  if (! status){
    fprintf(stderr, "Ok\n"); 
  } else {
    fprintf(stderr, "Error. Unable to control the laser, status is %d\n", status);
    return -1;
  }

  char command[1024];
  sprintf (command, "\nMD%04d%04d%02d000\n", startStep, endStep, clusterCount);


  status=hokuyo_readStatus(urg, command);
  if (status==99 || status==0){
    fprintf(stderr, "Continuous mode started with command %s\n", command);
    urg->isContinuous=1;
    return 1;
  }
  fprintf(stderr, "Error. Unable to set the continuous mode, status=%02d\n", status);

  return -1;
}

int hokuyo_stopContinuous(HokuyoURG* urg){
  if (! urg->isInitialized)
    return -1;
  if (! urg->isContinuous)
    return -1;

  int status=hokuyo_readStatus(urg, HK_QUIT);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->isContinuous=0;
  } else {
    fprintf(stderr, "Error. Unable to stop the laser\n");
    return -1;
  }
  return 1;
}

int hokuyo_reset(HokuyoURG* urg){
  if (! urg->isInitialized)
    return -1;

  int status=hokuyo_readStatus(urg, HK_RESET);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->isContinuous=0;
  } else {
    fprintf(stderr, "Error. Unable to reset laser\n");
    return -1;
  }
  return 1;
}

int hokuyo_close(HokuyoURG* urg){
  if (! urg->isInitialized)
    return -1;
  hokuyo_stopContinuous(urg);
  close(urg->fd);
  urg->isProtocol2=0;
  urg->isInitialized=0;
  urg->isContinuous=0;
  urg->fd=-1;
  return 1;
}
