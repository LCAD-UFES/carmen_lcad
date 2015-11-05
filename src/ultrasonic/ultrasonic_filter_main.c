 /*********************************************************
	---   Ultrasonic Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/ultrasonic_filter_interface.h>
#include <string.h>

/*********************************************************
		   --- Publishers ---
************************************************/


/*********************************************************
		   --- Handlers ---	

**********************************************************/

void publish_carmen_ultrasonic_sonar_sensor_message(ipc_ultrasonic_sensor_message_t *ipc_message)
{
   IPC_RETURN_TYPE err;
   int i;

   carmen_ultrasonic_sonar_sensor_message message;

   message.number_of_sonars = 4;
   message.sonar_beans = 360;
   message.fov = 0.628318; //36 degrees
   message.angle_step = 0.001745; //0.1 degrees
   message.start_angle = -0.314159; //-18 degrees
   message.max_range = 3.10;
   for(i = 0; i < 4; i++)
	message.sensor[i] = ipc_message->sensor[i];
   message.timestamp = ipc_message->timestamp;
   message.host = carmen_get_host();

   err = IPC_publishData(CARMEN_ULTRASONIC_SONAR_SENSOR_NAME, &message);
   carmen_test_ipc_exit(err, "Could not publish", CARMEN_ULTRASONIC_SONAR_SENSOR_FMT);
}

void 
ultrasonic_handler(ipc_ultrasonic_sensor_message_t *ipc_message)
{
	int i;

	for(i = 0; i < 4; i++)
		printf("%6.2lfm ", ipc_message->sensor[i]);
	printf("\n");
	publish_carmen_ultrasonic_sonar_sensor_message(ipc_message);
	//printf("Mensagem publicada.\n");
}	

void 
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("ultrasonic_filter: disconnected.\n");

    exit(0);
  }
}

  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Define messages that your module publishes */
  carmen_ultrasonic_sonar_sensor_define_messages();

  /* Subscribe to sensor messages */
  carmen_subscribe_message(IPC_ULTRASONIC_SENSOR_MESSAGE_NAME, 
                           IPC_ULTRASONIC_SENSOR_MESSAGE_FMT,
                           NULL, sizeof(ipc_ultrasonic_sensor_message_t), 
			   (carmen_handler_t) ultrasonic_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
