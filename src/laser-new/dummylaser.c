#include <carmen/carmen.h>
#include "carmen_laser_device.h"
#include "carmen_laser_message_queue.h"
#include "laser_messages.h"
#include <signal.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>


volatile int has_to_stop=0;
carmen_laser_message_queue_t queue;


int carmen_laser_enqueue(struct carmen_laser_device_t * device __attribute__((unused)), carmen_laser_laser_static_message* message){
  carmen_laser_message_queue_add(&queue, message);
  return 0;
}

void sigquit_handler(int q __attribute__((unused))){
  has_to_stop=1;
}

void* laser_fn (struct carmen_laser_device_t * device __attribute((unused))){
  while (! has_to_stop){
      int i;
      carmen_laser_laser_static_message m;
      m.timestamp=carmen_get_time();
      m.num_readings=10;
      m.num_remissions=0;
      for (i=0; i<m.num_readings; i++){
	m.range[i]=0.9*i;
      }
      carmen_laser_enqueue(NULL,&m);
      usleep(100000);
  }
  fprintf(stderr, "stopping\n");
  return 0;
}

int main(int argc, char **argv) 
{
  int num_laser_devices=1;
  int i;
  int c=0;
  pthread_t* laser_thread;
  void * tresult;
  char* hostname;
  static carmen_laser_laser_message msg;
  static double ranges[4096];
  static double remissions[4096];
  msg.range=ranges;
  msg.remission=remissions;


  tresult=0;
  hostname = carmen_get_host();
  fprintf(stderr, "Hostname=%s\n", hostname); 

  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);
  signal(SIGINT, sigquit_handler);

  
  //this initializes the driver stuctures
  carmen_laser_register_devices();
  carmen_laser_define_alive_message();
  for(i=0; i<num_laser_devices;i++)
    carmen_laser_define_laser_message(i);

  laser_thread = calloc(sizeof(pthread_t), num_laser_devices);

  for(i=0; i<num_laser_devices;i++) {
    //starts the reading thread
    fprintf(stderr, "starting reader for laser %d\n", i);
    pthread_create (&(laser_thread[i]), NULL, (void*(*)(void*))laser_fn, NULL);
  }
  fprintf(stderr, "done\n");

  //waits in the queue
  //double oldTime=0;
  while (! has_to_stop){
    carmen_laser_laser_static_message m;    
    int queued=carmen_laser_message_queue_wait_get(&queue, &m);
    if (queued<0){
      usleep(5000);
      continue;
    }    

    c++;
    msg.num_readings = m.num_readings;
    msg.num_remissions = m.num_remissions;
    msg.config = m.config;
    fprintf(stderr, "r=%d\n", m.num_readings);
    if (m.num_readings){ 
      memcpy(ranges,m.range, m.num_readings*sizeof(double));
      msg.range=ranges;
    } else {
      msg.range=NULL;
    }

    if (m.num_remissions){
      memcpy(remissions, m.remission, m.num_remissions*sizeof(double));
      msg.remission=remissions;
    } else {
      msg.remission=NULL;
    }
    

    msg.timestamp = m.timestamp;
    msg.host = hostname;
    
    carmen_laser_publish_laser_message(m.id, &msg);

    if (!(c%10)){
      int i;
      double time=msg.timestamp;
      
      fprintf(stderr, "(%d), %lf m=[ ", msg.num_readings, time);
      for (i=0; i<msg.num_readings; i++){
	fprintf(stderr, "%.2f ", msg.range[i]);
      }
      fprintf(stderr, "]\n ");
     // oldTime=time;
    }
  }
  
  for(i=0; i<num_laser_devices;i++) {
    //join the pending thread
    pthread_join(laser_thread[i], &tresult);
  }

  for(i=0; i<num_laser_devices;i++) {
    //cleanup phase
    fprintf(stderr, "cleanup laser %d\n", i);
  }    
  
  return 0;
}
