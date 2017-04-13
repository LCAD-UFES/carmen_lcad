/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/semantic_segmentation_interface.h>

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void 
publish_upkeymessage()
{
	//  IPC_RETURN_TYPE err;
	//
	//  err = IPC_publishData(CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_NAME, &filter_message);
	//  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_FMT);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void 
bumblebee_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	printf("%lf\n", message->timestamp);
	publish_upkeymessage();
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("semantic_segmentation: disconnected.\n");

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
	carmen_semantic_segmentation_define_messages();

	/* Subscribe to sensor messages */
	carmen_bumblebee_basic_subscribe_stereoimage(8 , NULL, (carmen_handler_t) bumblebee_handler, CARMEN_SUBSCRIBE_LATEST);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return (0);
}
