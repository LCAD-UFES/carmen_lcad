#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <carmen/carmen.h>
#include "carmen_hokuyo.h"
#include "hokuyourg.h"

#ifdef __APPLE__
#include <limits.h>
#include <float.h>
#define MAXDOUBLE DBL_MAX
#else
#include <values.h>
#endif 

#include "laser_messages.h"


int carmen_hokuyo_init(carmen_laser_device_t* device){
  HokuyoURG* urg=malloc(sizeof(HokuyoURG));
  carmen_test_alloc(urg);
  device->device_data=urg;
  hokuyo_init(urg);
  return 1;
}

int carmen_hokuyo_connect(carmen_laser_device_t * device, char* filename, int baudrate __attribute__((unused)) ){
  int result;
  HokuyoURG* urg=(HokuyoURG*)device->device_data;
  result=hokuyo_open(urg,filename);
  if (result<=0){
    fprintf(stderr, "error\n  Unable to opening device\n");
    return result;
  }
  return 1;
}

int carmen_hokuyo_configure(carmen_laser_device_t * device ){
  //device->config.start_angle=hokuyo_getStartAngle(urg,-1);
  device->config.angular_resolution=URG_ANGULAR_STEP;
  device->config.accuracy=0.001;
  device->config.maximum_range=5.600;	
  return 1;
}

int carmen_hokuyo_handle_sleep(carmen_laser_device_t* device __attribute__ ((unused)) ){
  sleep(1);
  return 1;
}

int carmen_hokuyo_handle(carmen_laser_device_t* device){
  HokuyoURG* urg=(HokuyoURG*)device->device_data;

  struct timeval timestamp;
  char buf[URG_BUFSIZE];

  int c=hokuyo_readPacket(urg, buf, URG_BUFSIZE,10);
  HokuyoRangeReading reading;
  hokuyo_parseReading(&reading, buf);
  if (c>0 && (reading.status==0 || reading.status==99) ){
    carmen_laser_laser_static_message message;
    message.id=device->laser_id;
    message.config=device->config;
    message.num_readings=reading.n_ranges;
    message.num_remissions=0;
    gettimeofday(&timestamp, NULL);
    message.timestamp=timestamp.tv_sec + 1e-6*timestamp.tv_usec;
    for (int j=0; j<reading.n_ranges; j++){
      message.range[j]=0.001*reading.ranges[j];
      if (message.range[j] <= 0.02) {
	message.range[j] += device->config.maximum_range;
      }
    }
    if (device->f_onreceive!=NULL)
      (*device->f_onreceive)(device, &message);
    return 1;
  } else {
    fprintf(stderr, "E");
  }
  return 0;
}

int carmen_hokuyo_start(carmen_laser_device_t* device){
  HokuyoURG* urg=(HokuyoURG*)device->device_data;
  int rv=hokuyo_init(urg);
  if (rv<=0)
    return 0;
  int bfov=(int)(device->config.fov/URG_ANGULAR_STEP);
  if (bfov>768)
    bfov=768;
  int bmin=URG_MAX_BEAMS/2-bfov/2;
  int bmax=URG_MAX_BEAMS/2+bfov/2;
  fprintf(stderr, "Configuring hokuyo continuous mode, bmin=%d, bmax=%d\n", bmin, bmax);
  rv=hokuyo_startContinuous(urg, bmin, bmax, 0);
  if (rv<=0){
    fprintf(stderr, "Error in configuring continuous mode\n");
  }
  device->f_handle=carmen_hokuyo_handle;
  return 1;
}

int carmen_hokuyo_stop(carmen_laser_device_t* device){
  HokuyoURG* urg=(HokuyoURG*)device->device_data;
  int rv=hokuyo_stopContinuous(urg);
  if (rv<=0){
    fprintf(stderr, "Error in stopping continuous mode\n");
    return 0;
  }
  device->f_handle=carmen_hokuyo_handle_sleep;
  return 1;
}



//FIXME I do not want  to malloc the ranges!

int carmen_hokuyo_close(struct carmen_laser_device_t* device){
  HokuyoURG* urg=(HokuyoURG*)device->device_data;
  return hokuyo_close(urg);
}

carmen_laser_device_t* carmen_create_hokuyo_instance(carmen_laser_laser_config_t* config, int laser_id){
  fprintf(stderr,"init hokuyo\n");
  carmen_laser_device_t* device=(carmen_laser_device_t*)malloc(sizeof(carmen_laser_device_t));
  carmen_test_alloc(device);
  device->laser_id=laser_id;
  device->config=*config;
  device->f_init=carmen_hokuyo_init;
  device->f_connect=carmen_hokuyo_connect;
  device->f_configure=carmen_hokuyo_configure;
  device->f_start=carmen_hokuyo_start;
  device->f_stop=carmen_hokuyo_stop;
  device->f_handle=carmen_hokuyo_handle_sleep;
  device->f_close=carmen_hokuyo_close;
  device->f_onreceive=NULL;
  return device;
}


carmen_laser_laser_config_t carmen_hokuyo_valid_configs[]=
  {{HOKUYO_URG,MAXDOUBLE,MAXDOUBLE,MAXDOUBLE,MAXDOUBLE,MAXDOUBLE,REMISSION_NONE}};

int carmen_hokuyo_valid_configs_size=1;

int carmen_init_hokuyo_configs(void){
  int i;
  carmen_laser_laser_config_t* conf=carmen_laser_configurations+carmen_laser_configurations_num;
  for (i=0; i<carmen_hokuyo_valid_configs_size; i++){
    *conf=carmen_hokuyo_valid_configs[i];
    conf++;
  }
  carmen_laser_configurations_num+=carmen_hokuyo_valid_configs_size;
  return carmen_laser_configurations_num;
}



