#include "sick_laser.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <signal.h>
#include <carmen/carmenserial.h>

#define INI                              -1
#define TIO                              0
#define STX                              0x02
#define UKN                              0x05
#define ACK                              0x06
#define DLE                              0x10
#define NAK                              0x15
#define LID                              0x80

#define CRC16_GEN_POL                    0x8005
#define CRC16_GEN_POL0                   0x80
#define CRC16_GEN_POL1                   0x05

#ifdef USE_TCP862
#include <carmen/tcp862.h>
#define serial_configure(_fd, _baudrate, _parity) tcp862_setBaud(_fd,_baudrate)
#define serial_readn(_fd,_buf,_size)   tcp862_readn(_fd,_buf,_size)
#define serial_writen(_fd,_buf,_size)  tcp862_writen(_fd,_buf,_size)
#define serial_ClearInputBuffer(_fd) tcp862_clearInputBuffer(_fd)
#else
#define serial_configure(_fd, _baudrate, _parity)  carmen_serial_configure(_fd, _baudrate, _parity)
#define serial_readn(_fd,_buf,_size)  carmen_serial_readn(_fd,_buf,_size)
#define serial_writen(_fd,_buf,_size) carmen_serial_writen(_fd,_buf,_size)
#define serial_ClearInputBuffer(_fd)  carmen_serial_ClearInputBuffer(_fd)
#endif


//helper functions for reading packets.
//on X86 architectures they are not needed, but i let there for compatibility
//BEGIN  command formatting
carmen_inline unsigned char sick_parse_int8(unsigned char*buf){
  return (unsigned char)*buf;
}

carmen_inline char sick_parse_uint8(unsigned char* buf){
  return (char) *buf;
}

carmen_inline unsigned short sick_parse_uint16(unsigned char*buf){
  unsigned char c0=*buf, c1=*(buf+1);
  return (unsigned short) c0|((unsigned short)c1<<8);
}

carmen_inline short sick_parse_int16(unsigned char* buf){
  unsigned char c0=*buf, c1=*(buf+1);
  return (short) c0|((short)c1<<8);
}

carmen_inline int sick_parse_int32(unsigned char* buf){
  unsigned char c0=*buf, c1=*(buf+1), c2=*(buf+2), c3=*(buf+3);
  return (unsigned int) c0|((unsigned int)c1<<8)|((unsigned int)c2<<16)|((unsigned int)c3<<24);
}

carmen_inline int sick_parse_uint32(unsigned char* buf){
  unsigned char c0=*buf, c1=*(buf+1), c2=*(buf+2), c3=*(buf+3);
  return (unsigned int) c0|((unsigned int)c1<<8)|((unsigned int)c2<<16)|((unsigned int)c3<<24);
}

carmen_inline unsigned char* sick_format_int8(unsigned char* buf, char d){
  *buf++=d;
  return buf;
}

carmen_inline unsigned char* sick_format_uint8(unsigned char* buf, unsigned char d){
  *buf++=d;
  return buf;
}

carmen_inline unsigned char* sick_format_int16(unsigned char* buf, short d){
  *buf++=(unsigned char)(d&0x00ff);
  *buf++=(unsigned char)(d>>8);
  return buf;
}

carmen_inline unsigned char* sick_format_uint16(unsigned char* buf, unsigned short d){
  *buf++=(unsigned char)(d&0x00ff);
  *buf++=(unsigned char)(d>>8);
  return buf;
}

carmen_inline unsigned char* sick_format_int32(unsigned char* buf, int d){
  int i;	
  for (i=0; i<4; i++){
    *buf++=(unsigned char)(d&0x00ff);
    d=d>>8;
  }
  return buf;
}

carmen_inline unsigned char* sick_format_uint32(unsigned char* buf, unsigned int d){
  int i;	
  for (i=0; i<4; i++){
    *buf++=(unsigned char)(d&0x00ff);
    d=d>>8;
  }
  return buf;
}

//END  command formatting

//BEGIN communication facilities
static unsigned short sick_compute_checksum(unsigned char *CommData, int uLen)
{
  unsigned char abData[2] = {0, 0};
  unsigned short uCrc16=0;
  
  while(uLen--) {
    abData[1]=abData[0];
    abData[0] = *CommData++;
    if(uCrc16 & 0x8000) {
      uCrc16=(uCrc16&0x7fff)<<1;
      uCrc16^=CRC16_GEN_POL;
    } 
    else {
      uCrc16<<=1;
    }
    uCrc16^=sick_parse_uint16(abData);
  }
  return uCrc16;
}

//tries to synchronize with a packet
int sick_wait_ack(sick_laser_t* sick,  int max_retries){
  int j=0;
  unsigned char c;
  ; //printf("Wait for ack...  ");
  while (j<max_retries){
    int val=serial_readn(sick->fd, &c, 1);
    if(val>0){
      if (c==ACK){
	//printf("OK\n");
	return 1;
      }
      if (c==NAK){
	fprintf(stderr, "ERROR, NAK received\n");
	return 1;
      }
    }
    j++;
  }
  ; //printf(" Timeout \n");
  return 0;
}

int sick_wait_packet_ts(sick_laser_t* sick, unsigned char* reply, struct timeval *tv ){
  unsigned char tempBuf[8192];
  unsigned char*t=tempBuf;
  unsigned int packet_size=0;
  unsigned char checksum[3];
  unsigned short s_checksum=0;
  unsigned short computed_checksum=0;
  int header_found=0;
  int val=0;
  int maxchars=2048;
  int chars=0;
  while (! header_found && chars<maxchars){
    t=tempBuf;
    val=serial_readn(sick->fd, t, 1);
    //printf("%02x \n", *t);
    if (val<=0){
      return 0;
    }
    if (*t==STX){
      t++;
      header_found=1;
      if (tv){
	gettimeofday(tv, NULL);
      }
      //fprintf(stderr,"H");
    }
    chars++;
  }
  val=serial_readn(sick->fd, t, 3); 
  if (val<=0)
    return 0;
  t+=val;
  packet_size=sick_parse_uint16(tempBuf+2);
  //fprintf (stderr,"[%d]",(int)packet_size);
  if (packet_size>1024){
    //fprintf(stderr,"A");
    return 0;
  }
  val=serial_readn(sick->fd, t, packet_size);
  if (val<=0)
    return 0;
  //printf ("val=%d\n",val);
  t+=packet_size;
  val=serial_readn(sick->fd, checksum, 2);
  if (val<=0)
    return 0;
  s_checksum=sick_parse_uint16(checksum);

  computed_checksum=sick_compute_checksum(tempBuf,t-tempBuf);
  //printf("buffer= %d\n",t-tempBuf);
  if (computed_checksum==s_checksum){
    memcpy(reply,tempBuf,t-tempBuf);
    //printf ("size=%d, returned=%d\n",(int)packet_size, t-tempBuf);
    gettimeofday(&sick->last_packet_time, NULL);
    return t-tempBuf;
  }
  fprintf (stderr, " checksum error ");
  return 0;
}

int sick_wait_packet(sick_laser_t* sick, unsigned char* reply){
  return sick_wait_packet_ts(sick, reply, NULL);
}


int sick_dump_output(sick_laser_t* sick, int max_retries){
  struct timeval t;
  fd_set set;
  int j=0;

  char buf[1024];

  t.tv_sec = 1;
  t.tv_usec = 0;
  FD_ZERO(&set);
  FD_SET(sick->fd, &set);
	
  while (j<max_retries){
    int sv = select(sick->fd + 1, &set, NULL, NULL, &t);
    if (sv){
      int val=read(sick->fd, buf, 1024);
      int i=0;
      while(val>0){
	; //printf("%02x ", (int)(buf[i]));
	i++;
	val--;
      }
    } else {
      ; //printf("timeout\n");
    }
    j++;
  }
  return 0;
}


int sick_send_packet_noanswer(sick_laser_t* sick, const unsigned char* command, unsigned short size){
  //Here we build up the packet
  unsigned char buf[1024];
  unsigned char* b=buf+2;
  unsigned short checksum=0;
  buf[0]=STX; buf[1]=0;
  b=sick_format_uint16(b, size);
  memcpy(b, command, size);
  b+=size;
  checksum=sick_compute_checksum(buf, b-buf);
  b=sick_format_uint16(b, checksum);
  serial_writen(sick->fd, buf, b-buf);
  return 1;
}

		        
int sick_send_packet(sick_laser_t* sick, unsigned char* reply, const unsigned char* command, unsigned short size){
  //Here we build up the packet
  unsigned char buf[1024];
  unsigned char* b=buf+2;
  unsigned short checksum=0;
  buf[0]=STX; buf[1]=0;
  b=sick_format_uint16(b, size);
  memcpy(b, command, size);
  b+=size;
  checksum=sick_compute_checksum(buf, b-buf);
  b=sick_format_uint16(b, checksum);
  serial_writen(sick->fd, buf, b-buf);
  if (! sick_wait_ack(sick,1))
    return 0;
  return sick_wait_packet(sick, reply);
}

int sick_send_packet_noack(sick_laser_t* sick, unsigned char* reply, const unsigned char* command, unsigned short size){
  int j;	
  //Here we build up the packet
  unsigned char buf[1024];
  unsigned char* b=buf+2;
  unsigned short checksum=0;
  buf[0]=STX; buf[1]=0;
  b=sick_format_uint16(b, size);
  memcpy(b, command, size);
  b+=size;
  checksum=sick_compute_checksum(buf, b-buf);
  b=sick_format_uint16(b, checksum);
  //serial_writen(sick->fd, b, b-buf);
  serial_writen(sick->fd, buf, b-buf);
  ; //printf("packet: ");
  for (j=0; j<b-buf; j++){
    ; //printf("%02x ", (unsigned int)buf[j]);
  }
  ; //printf("\n");
  //	if (! sick_wait_ack(sick,1))
  //	return 0;
  return sick_wait_packet(sick, reply);
}

//END  communication facilities


unsigned char  cmd_request_status[]={0x31};
unsigned short cmd_request_status_size=1;

unsigned char  cmd_request_measured_values[]={0x30,0x01};
unsigned short cmd_request_measured_values_size=2;

unsigned char  cmd_start_continuous_mode[]={0x20,0x24};
unsigned short cmd_start_continuous_mode_size=2;

unsigned char  cmd_start_continuous_interlaced_mode[]={0x20,0x2A};
unsigned short cmd_start_continuous_interlaced_mode_size=2;

unsigned char  cmd_start_continuous_remission_mode[]={0x20, 0x2b, 0x01, 0x00, 0x01, 0x00, 0xB5, 0x00 };
unsigned short cmd_start_continuous_remission_mode_size=8;



unsigned char  cmd_stop_continuous_mode[]={0x20,0x25};
unsigned short cmd_stop_continuous_mode_size=2;

unsigned char  cmd_set_config_mode[]={0x20,0x00};
unsigned short cmd_set_config_mode_size=2; // the password must follow

unsigned char  cmd_switch_variant[]={0x3b, 0x00, 0x00, 0x00, 0x00};
unsigned short cmd_switch_variant_size=5; // the password must follow

unsigned char  cmd_get_config[]={0x74};
unsigned short cmd_get_config_size=1; // the password must follow



int sick_test_baudrate(sick_laser_t* sick, int max_retries){
  //	printf ("Test baudrate ... ");
  int baudrates[]={9600, 19200, 38400, 500000};
  int baudrates_no=4;
  unsigned char reply[8192];
  int i;
  sick->baudrate=0;
  for (i=0; i<baudrates_no; i++){
    int j=0;
    serial_configure(sick->fd, baudrates[i], "8N1");
#ifdef USE_TCP862
    tcp862_setBaud(sick->fd,baudrates[i]);
#endif
    fprintf (stderr, "%d ", baudrates[i]);
    while(j<max_retries){
      j++;
      fprintf(stderr,".");
      //FIXME
      int response=sick_send_packet_noack(sick, reply, cmd_stop_continuous_mode, cmd_stop_continuous_mode_size);
      if (! response){
	continue;
      }
      usleep(10000);
      //			fprintf(stderr, "c");
      response=sick_send_packet(sick, reply, cmd_stop_continuous_mode, cmd_stop_continuous_mode_size);
      if (response){
	sick->baudrate=baudrates[i];
	fprintf(stderr," success\n");
	return baudrates[i];
      }
    }
    fprintf(stderr," failed\n                                     ");
  }
  return -1;
}


int sick_set_baudrate(sick_laser_t* sick, int max_retries, int brate){
  unsigned char command[2];
  unsigned char reply[1024];
  int j=0;
  int currentbrate=sick->baudrate;
  int max_response_retries=5;
  command[0]=0x20;
  if(brate == 500000)
    command[1] = 0x48;
  else if(brate == 38400) 
    command[1] = 0x40;
  else if(brate == 19200) 
    command[1] = 0x41;
  else
    command[1] = 0x42;

  serial_configure(sick->fd, currentbrate, "8N1");
  while(j<max_retries){
    sick_send_packet_noanswer(sick, command, 2);
    //sick_send_packet_noack(sick, reply, command, 2);
    serial_configure(sick->fd, currentbrate, "8N1");
	  
    //serial_ClearInputBuffer(sick->fd);
    int response=0;
    int response_retries=0;
    while(! response && response_retries< max_response_retries){
      response=sick_send_packet_noack(sick, reply, cmd_request_status, cmd_request_status_size);
		  
      response_retries+=(!response);
      usleep(1);
      //      fprintf(stderr, " %d", response_retries);
    }
    if (response){
      return response;
    }
    j++;
  }
  return 0;
}

int sick_get_configuration(sick_laser_t* sick){
  unsigned char conf[1024];
  unsigned char *command;
  int answerSize=0;
  int i=0;
  int retries=3;
  while(i<retries){
    answerSize=sick_send_packet(sick, conf, cmd_get_config, cmd_get_config_size);	
    if (! answerSize){
      //			fprintf(stderr, "unable to get the configuraion\n");
      return 0;
    }
    if (answerSize==40 ||  answerSize==38)
      break;
    i++;
  }
  if (i==retries)
    return 0;
  //fprintf(stderr, "\nreply[%d]:  ", answerSize);
  //for (i=0; i<answerSize; i++){
  //	fprintf(stderr, "%02x ", conf[i]);
  //}
  //fprintf(stderr, "\n");
  answerSize-=6;
  command=conf+4;
  memcpy(sick->lms_configuration, command, answerSize);
  sick->lms_conf_size=answerSize;
  return 1;
}


int sick_set_config_mode(sick_laser_t* sick){
  unsigned char reply[1024];
  unsigned char command[10];
  unsigned char *c=command;
  memcpy(c,cmd_set_config_mode,cmd_set_config_mode_size);
  c+=cmd_set_config_mode_size;
  memcpy(c,"SICK_LMS",8);
  c+=8;
  if (sick_send_packet(sick, reply, command, c-command)){
    sick->laser_mode=1;
    return 1;
  }
  return 0;
}



int sick_stop_continuous_mode(sick_laser_t* sick){
  unsigned char reply[1024];
  if (sick_send_packet(sick, reply, cmd_stop_continuous_mode, cmd_stop_continuous_mode_size)){
    sick->laser_mode=0;
    return 1;
  }
  return 0;
}


//unit 0=cm, 1=mm, 2=dm

int sick_set_range_reflectivity(sick_laser_t* sick, unsigned char * reply, int unit, int range, int remission){
  unsigned char conf[1024];
  unsigned char * command;
  memset(conf,0,1024);
  memcpy(conf, sick->lms_configuration, sick->lms_conf_size);
  int answerSize=sick->lms_conf_size;
  //fprintf(stderr, "\nconfig[%d]", answerSize);
  //for (i=0; i< answerSize; i++){
  //	fprintf(stderr, "%02x ", conf[i]);
  //}

  command=conf;

  command[0]=0x77;

  //laser sunlight
  command[5]|=1;

  assert(range==8 || range==16 || range==32 || range==80 || range==160 || range==320);
  assert(unit==0 || unit==1 || unit==2);
  assert(  range!=80 || unit==0);
  assert(  range!=160 || unit==0);
  assert(  range!=320 || unit==0);
  if (remission) {
    command[6]=13;
  } else {
    unsigned char r=0;
    switch(range){
    case 8:  r=0x01; break;
    case 16: r=0x03; break;
    case 32: r=0x05; break;
    case 80:  r=0x01; break;
    case 160: r=0x03; break;
    case 320: r=0x05; break;
    default: return 0;
    }
    command[6]=r;
  }
  command[7]=(unsigned char) unit;
  //command[11]=0X02;
  //command[33]=0X00;//availability level 3

  //fprintf(stderr, "\ncommand[%d]", answerSize);
  //for (i=0; i< answerSize; i++){
  //	fprintf(stderr, "%02x ", command[i]);
  //}
  //	fprintf(stderr, "\n");
  if ( (answerSize=sick_send_packet(sick, reply, command, 33)) ){
    sick->device_resolution=unit;
    sick->device_range=range;
    sick->remission_mode=remission;
    if (reply[5]==0x01)
      fprintf(stderr, "config accepted\n");
    else
      fprintf(stderr, "config refused\n");
    //for (i=0; i< answerSize; i++){
    //	fprintf(stderr, "%02x ", reply[i]);
    //}
    //fprintf(stderr, "\n");
    return answerSize;
  }
  return 0;
}

int sick_switch_variant(sick_laser_t* sick, int angular_range, int angular_resolution){
  unsigned char reply[1024];
  unsigned char command[100];
  unsigned char *c=command;
  memcpy(c,cmd_switch_variant,cmd_switch_variant_size);
  c+=1;

  //HACK: for correctly setting te sick in interlaced mode at 0.25 degrees, one has to first enable the
  //100 deg fov and 0.25 deg resolution. So I do :-)
  int fake_range=angular_range;
  if (angular_resolution == 25) {
    fake_range=100;
  }

  c=sick_format_int16(c, (short) fake_range);
  c=sick_format_int16(c, (short) angular_resolution);
  if (sick_send_packet(sick, reply, command, c-command)){
    fprintf(stderr, "accepted\n");
    sick->angular_resolution=angular_resolution;
    if (angular_range==180 && angular_resolution==25){
      sick->angular_resolution=0;
      //fprintf(stderr, " interlaced mode\n");
    }
    return 1;
  }
  //	fprintf(stderr, "refused .. ");
  return 0;
}


int sick_start_continuous_mode(sick_laser_t* sick){
  unsigned char reply[1024];
  unsigned char* command=cmd_start_continuous_mode;
  int cmd_size=cmd_start_continuous_mode_size;
  int interlaced=(sick->angular_resolution==0);
  if (interlaced){
    fprintf(stderr, "I");
    command=cmd_start_continuous_interlaced_mode;
    cmd_size=cmd_start_continuous_interlaced_mode_size;
  } else if (sick->remission_mode){
    fprintf(stderr, "R");
    command=cmd_start_continuous_remission_mode;
    cmd_size=cmd_start_continuous_remission_mode_size;
  } else {
    fprintf(stderr, "N");
  }
  if (sick_send_packet(sick, reply, command, cmd_size)){
    sick->laser_mode=0;
    return 1;
  }
  return 0;
}

unsigned char sick_parse_measurement(
				     sick_laser_t* sick __attribute__((unused)),
				     unsigned int* offset,
				     unsigned int * n_ranges,
				     unsigned int* range, 
				     unsigned int* glare, 
				     unsigned int* wfv, 
				     unsigned int* sfv,
				     unsigned int * n_remissions,
				     unsigned int* remission,
				     const unsigned char* packet){

  int off=0;
  int numMeasurements;
  int conversion=1;
  int i = 0, LoB = 0, HiB = 0, bit14, bit15;
  int offs;
  //int parts, mend, mstart;

  packet+=4;
  //	fprintf(stderr, "%02x ", *packet);
  if (*packet!=0xb0 && *packet!=0xf5){
    fprintf(stderr, "Error, no measurement packet receiver. Header=%02x ", *packet);
    return 0;
  }
	

  switch(*packet){
  case 0xb0:
    numMeasurements = ((int)packet[1] + ((int)packet[2] << 8)) & 0x000001FF;
    off=(packet[2] >>3) & 0x03;
    if (offset)
      *offset = off;
    
    bit14 = packet[2] & 0x40;
    bit15 = packet[2] & 0x80;
    
    if(!bit15)
      if(!bit14)
	conversion = 10;
      else
	conversion = 1;
    else
      conversion = 100;

    if (n_ranges)
      *n_ranges=numMeasurements;
    for (i = 0; i < numMeasurements; i++) {
      LoB = packet[i * 2 + 3]; 
      HiB = packet[i * 2 + 4];
      //fprintf(stderr, "%d ", (((HiB & 0x1f) << 8) + LoB) * conversion);
      if (range!=NULL){
	range[i] = (((HiB & 0x1f) << 8) + LoB) * conversion;
      }
      if (glare!=NULL)
	glare[i] = (HiB & 0x20) >> 5; 
      if (wfv!=NULL)
	wfv[i] = (HiB & 0x40) >> 6;  
      if (sfv!=NULL)
	sfv[i] = (HiB & 0x80) >> 7;
    }
    //fprintf(stderr, "l(%d,%d)", off, numMeasurements);
    return 0xb0;
  case 0Xf5:
      if (offset)
	  *offset=0;
//    parts = packet[1] & 0x7;
    offs = 0;
//    mstart = ((packet[offs + 4] << 8) + packet[offs + 3]);
//    mend   = ((packet[offs + 6] << 8) + packet[offs + 5]);
    //fprintf(stderr, "mstart, mend = %d, %d\n", mstart, mend);
    numMeasurements = ((packet[offs + 8] << 8) + packet[offs + 7]) & 0x3FFF;
    //fprintf(stderr, "num_measurem. = %d\n",numMeasurements);

    if (n_ranges)
      *n_ranges=numMeasurements;
    if (n_remissions)
      *n_remissions=numMeasurements;

    bit14 = packet[offs + 8] & 0x40;
    bit15 = packet[offs + 8] & 0x80;
    if(!bit15)
      if(!bit14)
	conversion = 10;
      else
	conversion = 1;
    else
      conversion = 100;

    for (i = 0; i < numMeasurements; i++) {
      LoB = packet[i * 4 + 9]; 
      HiB = packet[i * 4 + 10];
      if (range!=NULL){
	range[i] = ((HiB << 8) + LoB) * conversion;
      }
      if (remission!=NULL){
	*n_remissions=numMeasurements;
	remission[i] = packet[i * 4 + 12] * 256 + packet[i * 4 + 11];
      }
    }
    //fprintf(stderr, "r(%d)", numMeasurements);
    return 0Xf5;
  default:
    return 0;
  }
}


int sick_connect(sick_laser_t* sick, char* filename, int baudrate){
  int currentbrate=0;
  int brateSetted=0;
  if(carmen_serial_connect(&(sick->fd),filename)<0){
    fprintf(stderr,"error\n");
    fprintf(stderr,"  carmen_serial_connect failed!\n");
    return -1;
  }
  fprintf(stderr,"connected\n");
  serial_configure(sick->fd, 9600, "8N1");
  
  carmen_serial_set_low_latency(sick->fd);
  
  //fprintf(stderr,"Serial Configured\n");
  
  serial_ClearInputBuffer(sick->fd);
  
  fprintf(stderr,"  Querying baudrate ................ ");
  currentbrate=sick_test_baudrate(sick, 8);
  if (currentbrate<0){
    fprintf(stderr,"error\n  sick_test_baudrate failed!\n");
    return -2;
  } 
  fprintf(stderr,"  Detected baudrate ................ %d kbps\n", currentbrate);
  if (currentbrate!=baudrate){
    fprintf(stderr,"  Setting new baudrate ............. ");
    brateSetted=sick_set_baudrate(sick, 1, baudrate);
    if (!brateSetted){
      fprintf(stderr,"failed\n");
      return -3;
    }
    else {
      fprintf(stderr,"done (reconnect needed)\n");
      return -4;
    }

  } else {
    fprintf(stderr,"  Baudrate is already correct ...... done\n");
  }
  return 1;
}

int sick_configure(sick_laser_t* sick, int unit, int range, int reflectivity, int angular_range, int angular_res){
  unsigned char buffer[8192];
  int laserConfigured=0;
  int monitoringMode=0;
  int retries=10;
  int i;

  fprintf(stderr,"  Querying configuration ........... ");
  i=0;
  do {
    laserConfigured=sick_get_configuration(sick);
    i++;
  } while(laserConfigured<=0 && i< retries);

  if (!laserConfigured){
    fprintf(stderr," failed\n");
    return -2;
  } else {
    fprintf(stderr,"ok\n");
  }
  sleep(1);
  //fprintf(stderr, "configuration: unit=%d, range=%d, reflectivity=%d, angular_range=%d, angular_res=%d\n",
  //	unit, range, reflectivity, angular_range, angular_res);
  fprintf(stderr,"  Setting the config mode .......... ");
  i=0;
  do {
    laserConfigured=sick_set_config_mode(sick);
    i++;
  } while(laserConfigured<=0 && i<retries);
	
  if (!laserConfigured){
    fprintf(stderr,"failed \n");
    return -2;
  } else {
    fprintf(stderr,"done (laser light is red)\n");
  }

  sleep(1);
  fprintf(stderr,"  Setting range and remission ...... ");
  i=0;
  do {
    laserConfigured=sick_set_range_reflectivity(sick, buffer, unit, range, reflectivity);
    i++;
  } while(laserConfigured<=0 && i<retries);
  if (!laserConfigured){
    fprintf(stderr,"failed\n");
    return -2;
  } else {
    //		fprintf(stderr,"done\n");
  }

  sleep(1);
  fprintf(stderr,"  Setting variant .................. ");
  i=0;
  do {
    laserConfigured=sick_switch_variant(sick, angular_range, angular_res);
    i++;
  } while(laserConfigured<=0 && i<retries);

  if (!laserConfigured){
    fprintf(stderr,"failed\n");
    return -2;
  } else {
    //		fprintf(stderr,"done\n");
  }

  sleep(1);
  fprintf(stderr,"  Switching to monitorning mode ...");
  i=0;
  do {
    monitoringMode=sick_stop_continuous_mode(sick);
    i++;
  } while(monitoringMode<=0 && i<retries);
  if (!monitoringMode){
    fprintf(stderr," failed ");
    return -2;
  } else {
    fprintf(stderr," done\n");
  }
  return 1;
}



