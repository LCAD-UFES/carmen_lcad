 /*********************************************************
	---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/skeleton_module_sensor_interface.h>

static int skeleton_module_sensor_getkey = 0;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void publish_keymessage(char c)
{
   IPC_RETURN_TYPE err;

   carmen_skeleton_module_sensor_keymessage_message message;

   message.caracter = c;
   message.timestamp = carmen_get_time();
   message.host = carmen_get_host();

   err = IPC_publishData(CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_NAME, &message);
   carmen_test_ipc_exit(err, "Could not publish", CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_FMT);
}

void publish_heartbeats()
{
    carmen_publish_heartbeat("skeleton_sensor_daemon");
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void shutdown_module(int signo)
{
  if(signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("skeleton_module_sensor: disconnected.\n");
     exit(0);
  }
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
	  {"skeleton_module_sensor", "getkey", CARMEN_PARAM_ONOFF, &skeleton_module_sensor_getkey, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int main(int argc, char **argv)
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Read application specific parameters (Optional) */
  read_parameters(argc, argv);

  /* Define published messages by your module */
  carmen_skeleton_module_sensor_define_messages();

  carmen_ipc_addPeriodicTimer(10, publish_heartbeats, NULL);

  while(1)
  {
    if(skeleton_module_sensor_getkey)
    {
	    char c = getchar();
            if(c == EOF)
		break;

	    printf("%c", c);
	    publish_keymessage(c);
	    usleep(1000);

    }
  }

  carmen_ipc_disconnect();
  return 0;
}
