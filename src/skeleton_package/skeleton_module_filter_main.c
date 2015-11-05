 /*********************************************************
	---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/skeleton_module_filter_interface.h>
#include <carmen/skeleton_module_sensor_interface.h>
#include <string.h>

static int waiting_cicles = 0;

/*********************************************************
		   --- Publishers ---
**********************************************************/

void 
publish_upkeymessage(carmen_skeleton_module_sensor_keymessage_message* sensor_message)
{
  IPC_RETURN_TYPE err;
  int i = 0;
  float f = 0;

  carmen_skeleton_module_filter_upkeymessage_message filter_message;

  filter_message.is_up = 1;
  filter_message.up_caracter = toupper(sensor_message->caracter);
  filter_message.timestamp = sensor_message->timestamp; /* !!! Must have the same sensor message timestamp !!!*/
  filter_message.host = sensor_message->host;

  if (waiting_cicles)
  {
    while (i < 100000)
    {
      f += sqrt((sin(i) * sin(i)) + (cos(i) * cos(i)));
      i++;
    }
  }

  printf("%c", filter_message.up_caracter);

  err = IPC_publishData(CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_NAME, &filter_message);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_FMT);
}


/*********************************************************
		   --- Handlers ---
**********************************************************/

void 
keymessage_handler(carmen_skeleton_module_sensor_keymessage_message *message)
{
  publish_upkeymessage(message);
}


void 
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("skeleton_module_filter: disconnected.\n");

    exit(0);
  }
}


void 
publish_heartbeats()
{
  carmen_publish_heartbeat("skeleton_filter_module");
}

  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  if (argc > 1)
    waiting_cicles = atoi(argv[1]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Define messages that your module publishes */
  carmen_skeleton_module_filter_define_messages();

  /* Subscribe to sensor messages */
  carmen_skeleton_module_sensor_subscribe_keymessage(NULL, (carmen_handler_t) keymessage_handler, CARMEN_SUBSCRIBE_ALL);

  /* Create a timer that periodicaly (1.0s) publish a heart beat that says that the module is alive */
  carmen_ipc_addPeriodicTimer(1.0, publish_heartbeats, NULL);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
