#include <carmen/carmen.h>
#include "carmen_laser_device.h"
#include "carmen_laser_message_queue.h"
#include "laser_messages.h"
#include <signal.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>
#include <errno.h>


//#define MAX_REQUESTED_LASER_IDS 100

volatile int carmen_laser_has_to_stop=0;
carmen_laser_message_queue_t carmen_laser_queue;

carmen_laser_device_t** carmen_laser_pdevice = NULL;
int  *carmen_laser_flipped=NULL;

int carmen_laser_use_device_locks = 0;



int index_to_id(int idx) {
  return idx+1;
}

int id_to_index(int id) {
  return id-1;
}


void generate_lock_filename(char* result, char* filename){
  char prefix[]="/tmp/lock_carmen_laser";
  strcpy(result, prefix);
  result+=strlen(prefix);
  strcpy(result,filename);
  while (*result!=(char)0){
    *result=(*result=='/')?'_':*result;
    result++;
  }
}

int create_lock(char* filename){
  char buf[1024];
  generate_lock_filename(buf,filename);
  int fd=open(buf, O_CREAT|O_EXCL, O_RDWR);
  if (fd==-1){
    if (errno==EEXIST)
      return -1;
  } else
    close(fd);
  //  carmen_warn("creating lock file \"%s\"\n",buf);
  return 0;
}

int delete_lock(char* filename){
  char buf[1024];
  generate_lock_filename(buf,filename);
  return unlink(buf);
}

int carmen_laser_enqueue(struct carmen_laser_device_t * device __attribute__((unused)), carmen_laser_laser_static_message* message){
  carmen_laser_message_queue_add(&carmen_laser_queue, message);
  return 0;
}

int carmen_laser_enqueue_correct(struct carmen_laser_device_t * device __attribute__((unused)), carmen_laser_laser_static_message* message){
  double expectedTime=device->last_packet_time+device->expected_period;
  double dt=message->timestamp-device->last_packet_time;
  device->last_packet_time=message->timestamp;
  if (fabs(dt-device->expected_period)/device->expected_period>0.5){

    message->timestamp=expectedTime;
    //fprintf(stderr,"c");
  }
  carmen_laser_message_queue_add(&carmen_laser_queue, message);
  return 0;
}


void sigquit_handler(int q __attribute__((unused))){
  carmen_laser_has_to_stop=1;
}


void* laser_fn (struct carmen_laser_device_t * device){
  int result=0;
  result=(*(device->f_start))(device);
  if (! result){
    carmen_laser_has_to_stop=1;
    return 0;
  }
  while (! carmen_laser_has_to_stop){
    result=(*(device->f_handle))(device);
  }
  fprintf(stderr, "stopping\n");
  result=(*(device->f_stop))(device);
  if (! result){
    return 0;
  }
  return 0;
}

void* calibrate_and_start_laser_fn (struct carmen_laser_device_t * device){
  int result=0;
  fprintf(stderr, "Timestamp calibration for laser %d, please wait a few seconds (100 scans)\n",
	  device->laser_id);
  carmen_laser_calibrate_timestamp(device, 100);
  result=(*(device->f_start))(device);
  if (! result){
    carmen_laser_has_to_stop=1;
    return 0;
  }
  int calibrated=0;
  while (! carmen_laser_has_to_stop){
    result=(*(device->f_handle))(device);
    calibrated=!device->max_frames;
    if (calibrated){
      device->f_onreceive=carmen_laser_enqueue_correct;
    }
  }
  result=(*(device->f_stop))(device);
  if (! result){
    return 0;
  }
  return 0;
}


int carmen_laser_create_device(int laser_id,
			       carmen_laser_device_t** pdevice,
			       carmen_laser_laser_config_t config,
			       char* devicename, int baud){
  
  int result=0;
  
  //check whether the configuration is valid
  fprintf(stderr, "Checking requested configuration ... see below\n");
  int config_valid=carmen_laser_laser_message_check_configuration(&config);
  if (! config_valid){
    fprintf(stderr,"\nERROR: The configuration seems to be invalid (at least it \n");
    fprintf(stderr,"       is not added as a valid configuration in this driver)\n");
    return -1;
  }
/*   else */
/*     fprintf(stderr,"done\n"); */

  //connect, and set the serial line (inf any) to the desired baudrate

  if (carmen_laser_use_device_locks) {
    fprintf(stderr, "Checking lock ...................... ");
    int lock=create_lock(devicename);
    if (lock==-1){
      fprintf(stderr, "failed\n\n");
      carmen_warn("  A lock file on the device %s exist.\n", devicename);
      carmen_warn("  This means that another laser instance is accessing the device\n");
      carmen_warn("  or that a previously running laser has been terminated incorrectly.\n");
      char buf[1024];
      generate_lock_filename(buf, devicename);
      carmen_warn("  Check for a running laser instance and delete the file\n  \"%s\" and restart laser.\n", buf);
      return -2;
    } 
    else
      fprintf(stderr, "lock created\n");
  }

  
  //attempts to create a laser instance
  //  fprintf(stderr, "Creating internatal device ........... ");
  (*pdevice)=carmen_create_laser_instance(&config, laser_id , devicename);
  if (! (*pdevice)){
    //    fprintf(stderr, "error\n");
    fprintf(stderr, "Error while creating interal device structure.\n");
    return -3;
  }
  
  //install the enqueuing handler
  (*pdevice)->f_onreceive=carmen_laser_enqueue;
  (*pdevice)->config=config;
  
  //attempt initializing the device
  fprintf(stderr, "Initialization device .............. ");
  result=(*((*pdevice)->f_init))((*pdevice));
  if (! result){
    fprintf(stderr, "error\n");
    return -4;
  }
  else 
    fprintf(stderr, "done\n");
  
  fprintf(stderr, "\n");  
  fprintf(stderr, "Connecting device .................. ");

  result=(*((*pdevice)->f_connect))((*pdevice), devicename, baud);
  if (result<=0){
    fprintf(stderr, "Connection to laser device failded\n");
    return -5;
  }
  else
    fprintf(stderr, "Connection to laser device succeded\n");
  
  fprintf(stderr, "\n");  
  //configure the device
  fprintf(stderr, "Configuring laser device ........... see below\n");
  result=(*((*pdevice)->f_configure))((*pdevice));
  if (! result){
    fprintf(stderr, "Configuration of laser device failded\n");
    return -6;
  }
  else
    fprintf(stderr, "Configuration of laser device succeded\n");
  
  return 0;
}


int carmen_laser_read_parameters(int argc, char **argv)
{
  int num_laser_devices=0;
  int i=0;
  char **device=NULL;
  char **remission_mode_string=NULL;
  double *resolution=NULL;
  double *fov=NULL;
  int  *laser_used_in_inifile=NULL;
  int  *baud=NULL;
  char **laser_type=NULL;
/*   char **range_mode=NULL; */
  double *maxrange=NULL;

  char var_name[256];
  char var_remission_mode[256];
  char var_dev[256];
  char var_type[256];
  char var_res[256];
  char var_fov[256];
  char var_baud[256];
  char var_flipped[256];
/*   char var_maxrange[256]; */
/*   char var_range_mode[256]; */

  carmen_param_t laser_num_devs[] = {
    {"laser", "num_laser_devices", CARMEN_PARAM_INT, &num_laser_devices, 0, NULL},
    {"laser", "use_device_locks", CARMEN_PARAM_ONOFF, &carmen_laser_use_device_locks, 0, NULL} };

  carmen_param_install_params(argc, argv, laser_num_devs, 
			      sizeof(laser_num_devs) / sizeof(laser_num_devs[0]));
  
  carmen_laser_pdevice        =  calloc(  num_laser_devices, sizeof(carmen_laser_device_t*) );
  carmen_test_alloc(carmen_laser_pdevice);

  if (num_laser_devices<1)
    carmen_die("You have to specify at least one laser device to run laser.\nPlease check you ini file for the parameter num_laser_devices.\n");

  int requested_lasers[num_laser_devices];
  for (i=0; i<num_laser_devices; i++) {
    requested_lasers[i]=0;
  }
  int cmdline_laser_cnt=0;

  int c=1;
  while (c<argc && strcmp(argv[c],"-id")) 
    c++;
  c++;
  if (c < argc){
    for (; c<argc; c++){
      int id=atoi(argv[c]);

      if (id < 1)
	carmen_die("ERROR: Invalid laser id (%d). The id must be bigger than 0.\n", id);
      else if (id > num_laser_devices)
	carmen_die("ERROR: The laser with id=%d is not specified in the ini file.\n", id);
      else if (cmdline_laser_cnt == num_laser_devices)
	carmen_die("ERROR: There are more id in the command line than in the ini file!\n");
      else {
	int index = id_to_index(id);
	requested_lasers[index] = 1;
	cmdline_laser_cnt++;
      }
    }
  } else {
    cmdline_laser_cnt = num_laser_devices;
    for (i=0; i<num_laser_devices; i++){
      requested_lasers[i]=1;
    }
  }

  if (cmdline_laser_cnt==0) 
    carmen_die("Error: You cannot run the module without a single laser device.\n");

  fprintf(stderr, "This instance of laser will use the lasers with the ids: [");
  int first =1;
  for (c=0; c<num_laser_devices; c++){
    if (requested_lasers[c]) {
      if (first) {
	first=0;
	fprintf(stderr,"%d", index_to_id(c));
      }
      else
	fprintf(stderr," %d", index_to_id(c));
    }
  }
  fprintf(stderr, "]\n");
  

  for (i=0; i< num_laser_devices; i++)  
    carmen_laser_pdevice[i] = NULL;

  device         = calloc( num_laser_devices, sizeof(char*) );
  carmen_test_alloc(device);
  remission_mode_string = calloc( num_laser_devices, sizeof(char*) );
  carmen_test_alloc(remission_mode_string);
  resolution     = calloc( num_laser_devices, sizeof(double) );
  carmen_test_alloc(resolution);
  fov            = calloc( num_laser_devices, sizeof(double) );
  carmen_test_alloc(fov);
  laser_used_in_inifile      = calloc( num_laser_devices, sizeof(int) );
  carmen_test_alloc(laser_used_in_inifile);
  laser_type     = calloc( num_laser_devices, sizeof(char*) );
  carmen_test_alloc(laser_type);
  baud           = calloc( num_laser_devices, sizeof(int) );
  carmen_test_alloc(baud);
  carmen_laser_flipped        = calloc( num_laser_devices, sizeof(int) );
  carmen_test_alloc(carmen_laser_flipped);
  maxrange       = calloc( num_laser_devices, sizeof(double) );
  carmen_test_alloc(maxrange);
/*   range_mode     = calloc( num_laser_devices, sizeof(char*) ); 
  carmen_test_alloc(range_mode);
*/


  for (i=0; i<num_laser_devices; i++)  {

    int id_of_laser_i = index_to_id(i);

    sprintf(var_name,"laser%d", id_of_laser_i);
    
    strcpy(var_dev,var_name);
    strcat(var_dev, "_dev");
    strcpy(var_type,var_name);
    strcat(var_type, "_type");
    strcpy(var_remission_mode,var_name);
    strcat(var_remission_mode, "_use_remission");
    strcpy(var_res,var_name);
    strcat(var_res, "_resolution");
    strcpy(var_fov,var_name);
    strcat(var_fov, "_fov");
    strcpy(var_baud,var_name);
    strcat(var_baud, "_baud");
    strcpy(var_flipped,var_name);
    strcat(var_flipped, "_flipped");

/*     strcpy(var_range_mode,var_name); */
/*     strcat(var_range_mode, "_range_mode"); */
/*     strcpy(var_maxrange,var_name); */
/*     strcat(var_maxrange, "_maxrange"); */

    carmen_param_t laser_dev[] = {
      {"laser", var_dev, CARMEN_PARAM_STRING, &(device[i]), 0, NULL},
    };
    carmen_param_install_params(argc, argv, laser_dev, 
				sizeof(laser_dev) / sizeof(laser_dev[0]));

    if(strncmp(device[i], "none", 4) != 0) {
      laser_used_in_inifile[i] = 1;

      carmen_param_t param_laser_type[] = {
	{"laser", var_type, CARMEN_PARAM_STRING, &(laser_type[i]), 0, NULL},
      };
      carmen_param_install_params(argc, argv, param_laser_type, 
				  sizeof(param_laser_type) / sizeof(param_laser_type[0]));

	if ( !strcmp(laser_type[i], "pls") || !strcmp(laser_type[i], "PLS") ) {
		carmen_warn("Sorry, the SICK PLS is currently NOT supported. Within the next weeks\n");
		carmen_warn("the driver will be updates and should then support the PLS. Please use\n");
		carmen_warn("the old laser driver if you want to use a PLS\n");
		carmen_die("Laser aborted.\n");
	}
	else if ( !strcmp(laser_type[i], "lms") || !strcmp(laser_type[i], "LMS") ) {
	
		maxrange[i] = 81.9;
		
		carmen_param_t laser_params[] = {
		{"laser", var_res, CARMEN_PARAM_DOUBLE,              &(resolution[i]),     0, NULL},
		{"laser", var_remission_mode, CARMEN_PARAM_STRING,   &(remission_mode_string[i]), 0, NULL},
		{"laser", var_fov, CARMEN_PARAM_DOUBLE,              &(fov[i]),            0, NULL},
		{"laser", var_baud, CARMEN_PARAM_INT,                &(baud[i]),           0, NULL},
		/* 	  {"laser", var_range_mode, CARMEN_PARAM_STRING,       &(range_mode[i]),     0, NULL}, */
		/* 	  {"laser", var_maxrange, CARMEN_PARAM_DOUBLE,         &(maxrange[i]),       0, NULL}, */
		{"laser", var_flipped, CARMEN_PARAM_INT,             &(carmen_laser_flipped[i]),        0, NULL}};
		
		carmen_param_install_params(argc, argv, laser_params,
					sizeof(laser_params) / 
					sizeof(laser_params[0]));
	}
	else if ( !strcmp(laser_type[i], "s300") || !strcmp(laser_type[i], "S300") ) {
	
		maxrange[i] = 30.0;
		
		carmen_param_t laser_params[] = {
		{"laser", var_res, CARMEN_PARAM_DOUBLE,              &(resolution[i]),     0, NULL},
		{"laser", var_remission_mode, CARMEN_PARAM_STRING,   &(remission_mode_string[i]), 0, NULL},
		{"laser", var_fov, CARMEN_PARAM_DOUBLE,              &(fov[i]),            0, NULL},
		{"laser", var_baud, CARMEN_PARAM_INT,                &(baud[i]),           0, NULL},
		/* 	  {"laser", var_range_mode, CARMEN_PARAM_STRING,       &(range_mode[i]),     0, NULL}, */
		/* 	  {"laser", var_maxrange, CARMEN_PARAM_DOUBLE,         &(maxrange[i]),       0, NULL}, */
		{"laser", var_flipped, CARMEN_PARAM_INT,             &(carmen_laser_flipped[i]),        0, NULL}};
		
		carmen_param_install_params(argc, argv, laser_params,
					sizeof(laser_params) / 
					sizeof(laser_params[0]));
	}
	else if ( !strcmp(laser_type[i], "hokuyourg") || !strcmp(laser_type[i], "HOKUYOURG")) {
	
		carmen_param_t laser_params[] = {
		  {"laser", var_flipped, CARMEN_PARAM_INT,             &(carmen_laser_flipped[i]),        0, NULL},
		  {"laser", var_fov, CARMEN_PARAM_DOUBLE,              &(fov[i]),            0, NULL}};
		carmen_param_install_params(argc, argv, laser_params,
					sizeof(laser_params) / 
					sizeof(laser_params[0]));
		
		remission_mode_string[i] = "none";
		resolution[i] = 0;
		baud[i] = 0;
		carmen_laser_flipped[i] = 0;
		/* 	range_mode[i] = 0; */
		maxrange[i] = 0;
	}
	else {
		carmen_die("ERROR: the parameter laser_type does not allow the value %s.\nUse lms, pls, S300 or hokuyourg", laser_type[i]);
	}

    }
    else {
      laser_used_in_inifile[i] = 0;
      laser_type[i] = NULL;
      resolution[i] = 0;
      remission_mode_string[i] = NULL;
      fov[i] = 0;
      baud[i] = 0;
      carmen_laser_flipped[i] = 0;
/*       range_mode[i] = 0; */
      maxrange[i] = 0;
    }
  }

  for (i=0; i< num_laser_devices; i++)  {
    if (requested_lasers[i] && laser_used_in_inifile[i]) {

      carmen_warn("\n--------------------------------------------------\n");

      carmen_laser_laser_config_t config;
      config.laser_type = UMKNOWN_PROXIMITY_SENSOR;
      config.remission_mode = REMISSION_NONE;

	if ( !strcmp(laser_type[i], "lms") || !strcmp(laser_type[i], "LMS")) {
		config.laser_type = SICK_LMS;
	}
	else if ( !strcmp(laser_type[i], "s300") || !strcmp(laser_type[i], "S300")) {
		config.laser_type = SICK_S300;
	}
	else if ( !strcmp(laser_type[i], "pls") || !strcmp(laser_type[i], "PLS")) {
		config.laser_type = SICK_PLS;
	}
	else if ( !strcmp(laser_type[i], "hokuyourg") || !strcmp(laser_type[i], "HOKUYOURG")) {
		config.laser_type = HOKUYO_URG;
	}
	else {
	  carmen_die("ERROR: the parameter laser_type does not allow the value %s.\nUse lms, s300, pls, or hokuyourg", laser_type[i]);
	}

      if ( !strcmp(remission_mode_string[i], "none")) {
	config.remission_mode = REMISSION_NONE;
      }
      else if ( !strcmp(remission_mode_string[i], "direct")) {
	config.remission_mode = REMISSION_DIRECT;
      }
      else if ( !strcmp(remission_mode_string[i], "normalized")) {
	config.remission_mode = REMISSION_NORMALIZED;
      }
      else
	carmen_die("ERROR: the parameter laser_remission does not allow the value %s.\nUse none, direct, or normalized!\n", remission_mode_string[i]);

      config.fov = carmen_degrees_to_radians(fov[i]);
      config.start_angle = -0.5 * config.fov;
      config.angular_resolution = carmen_degrees_to_radians(resolution[i]);
      config.maximum_range = maxrange[i]; 
      config.accuracy = 0.01;
      
      carmen_warn("Laser id ........................... %d\n", index_to_id(i));
	
      if (baud[i]>0)
	carmen_warn("Requested baud rate ................ %d kbps\n", baud[i]);
      else
	carmen_warn("Requested baud rate ................ not required\n");
      int result = carmen_laser_create_device(index_to_id(i),
					      &(carmen_laser_pdevice[i]), 
					      config,
					      device[i], 
					      baud[i]);
      
      carmen_warn("\n");

      if (result != 0){
	//	carmen_warn("Releasing locks\n");

	if (carmen_laser_use_device_locks) {
	  int end=i;
	  if (result>=-2) // lock file has not been created
	    end=i-1;
	  int j;
	  for (j=0; j<=end; j++){
	    if (requested_lasers[j]) {
	      carmen_warn("  freeing lock of laser %d .......... ", index_to_id(j));
	      delete_lock(device[j]);
	      carmen_warn("done\n");
	      
	    }
	  }
	}
	carmen_die("ERROR: Could not init all requested laser devices!\n");
      }
      carmen_warn("Initializing of laser with %d successfully completed!\n",i);
    }
  }
  carmen_warn("\n--------------------------------------------------\n");

  free(device);
  free(remission_mode_string);
  free(resolution);
  free(baud);
  free(fov);
  free(laser_used_in_inifile);
  free(laser_type);
  free(maxrange);
/*   free(range_mode); */

  return num_laser_devices;
}



int main(int argc, char **argv) 
{

  int num_laser_devices;
  int i;
  int result;
  int c=0;
  pthread_t* laser_thread;
  void * tresult;
  char* hostname;
  static carmen_laser_laser_message msg;
  static double ranges[4096];
  static double remissions[4096];
  msg.range=ranges;
  msg.remission=remissions;

  hostname = carmen_get_host();
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

/*  do not allow ctrl-c while setting baud rate */
/*   signal(SIGINT, sigquit_handler); */

  
  //this initializes the driver stuctures
  carmen_laser_register_devices();

  num_laser_devices = carmen_laser_read_parameters(argc, argv);
  
  carmen_laser_define_alive_message();
  for(i=1; i<=num_laser_devices;i++)
    carmen_laser_define_laser_message(i);
    
  
  laser_thread = calloc(num_laser_devices, sizeof(laser_thread));
  carmen_test_alloc(laser_thread);

  for(i=0; i<num_laser_devices;i++) {
    if (carmen_laser_pdevice[i]) {
      //starts the reading thread
      fprintf(stderr, "Info: Starting thread for laser %d\n", index_to_id(i) );
      pthread_create (&(laser_thread[i]), NULL, (void*(*)(void*))calibrate_and_start_laser_fn, carmen_laser_pdevice[i]);
    }
    else {
      fprintf(stderr, "Info: The laser %d is not used in this instance of laser\n", index_to_id(i) );
    }
  }
  carmen_warn("\n--------------------------------------------------\n");
  
  signal(SIGINT, sigquit_handler);


  //waits in the queue
  double lastTime=0;
  carmen_laser_laser_static_message m;
  while (! carmen_laser_has_to_stop){
    carmen_laser_message_queue_wait_get(&carmen_laser_queue, &m);

    if (lastTime==0)
      lastTime = carmen_get_time();
    c++;
    msg.num_readings = m.num_readings;
    msg.num_remissions = m.num_remissions;
    msg.config = m.config;
    msg.id = m.id;
  
    //    fprintf(stderr, "r=%d\n", m.num_readings);
    if (m.num_readings){ 
      msg.range=ranges;
      memcpy(msg.range,m.range, m.num_readings*sizeof(double));
    } else {
      msg.range=NULL;
    }

    if (m.num_remissions){
      msg.remission=remissions;
      memcpy(msg.remission, m.remission, m.num_remissions*sizeof(double));
    } else {
      msg.remission=NULL;
    }

    /* is the laser flipped (mouted upside down) */
    int idx = id_to_index(msg.id);
    if (carmen_laser_flipped[idx]) {
      double tmp=0;
      int i;
      int upto=msg.num_readings/2;
      if (msg.range != NULL) {
	for (i=0; i < upto; i++) {
	  tmp = msg.range[i] ;
	  msg.range[i] = msg.range[msg.num_readings-i];
	  msg.range[msg.num_readings-i] = tmp;
	}
      }
      upto=msg.num_remissions/2;
      if (msg.remission != NULL) {
	for (i=0; i < upto; i++) {
	  tmp = msg.remission[i] ;
	  msg.remission[i] = msg.remission[msg.num_remissions-i];
	  msg.remission[msg.num_remissions-i] = tmp;
	}
      }
    }

    msg.timestamp = m.timestamp;
    msg.host = hostname;
    
    if (msg.id>0) {
      carmen_laser_publish_laser_message(msg.id, &msg);
      //printf("publish: id: %d\n", msg.id);
    }
    else {
	printf("ERROR: NOT PUBLISHING: MSG ID <= 0\n");
    }
    
    if (c>0 && !(c%10)){
      double time=carmen_get_time();
      if (time-lastTime > 3.0) {
	
	fprintf(stderr, "status:   send-queue: %d msg(s),   laser-msg freqency: %.3f Hz (globally)\n", 
		carmen_laser_queue.size, ((double)c)/(time-lastTime));
	c=0;
	lastTime=time;
      }
    }
  }
  
  for(i=0; i<num_laser_devices;i++) {
    if (carmen_laser_pdevice[i]) {
      //join the pending thread
      pthread_join(laser_thread[i], &tresult);
    }
  }

  for(i=0; i<num_laser_devices;i++) {
    if (carmen_laser_pdevice[i]) {
      
      //cleanup phase
      fprintf(stderr, "Cleaning up structures from laser %d ...... ", index_to_id(i));
      result=(*(carmen_laser_pdevice[i]->f_close))(carmen_laser_pdevice[i]);

      if (carmen_laser_use_device_locks) {
	delete_lock(carmen_laser_pdevice[i]->device_name);
      }
      if (! result){
	fprintf(stderr, "error\n");
	return -3;
      }
      else
	fprintf(stderr, "done\n");
    }
  }    

  if (carmen_laser_flipped)
    free(carmen_laser_flipped);  
  return 0;
}
