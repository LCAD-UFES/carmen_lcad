#ifndef URG_H
#define URG_H

/* needed for new carmen_inline def for gcc >= 4.3 */
#include <carmen/carmen.h>


//#define HOKUYO_ALWAYS_IN_SCIP20

#define URG_BUFSIZE 8192
#define URG_ANGULAR_STEP (M_PI/512.)
#define URG_MAX_BEAMS 768

typedef struct HokuyoRangeReading{
  int timestamp;
  int status;
  int n_ranges;
  unsigned short ranges[URG_MAX_BEAMS];
  unsigned short startStep, endStep, clusterCount;
} HokuyoRangeReading;

typedef struct HokuyoURG{
  int fd;
  int isProtocol2;
  int isContinuous;
  int isInitialized;
} HokuyoURG;

// opens the urg, returns <=0 on failure
int hokuyo_open(HokuyoURG* urg, const char* filename);

// initializes the urg and sets it to the new scip2.0 protocol
// returns <=0 on failure
int hokuyo_init(HokuyoURG* urg);

// reads a packet into the buffer
unsigned int hokuyo_readPacket(HokuyoURG* urg, char* buf, int bufsize, int faliures);

// starts the continuous mode
int hokuyo_startContinuous(HokuyoURG* urg, int startStep, int endStep, int clusterCount);

// starts the continuous mode
int hokuyo_stopContinuous(HokuyoURG* urg);
int hokuyo_reset(HokuyoURG* urg);
int hokuyo_close(HokuyoURG* urg);
void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer);

#endif

