/*********************************************************
	---  Visual car tracking Module ---
 **********************************************************/


#include "../visual_car_tracking/visual_car_tracking.h"


/* number of the camera that will be used */
static int camera;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


//TODO
void bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	cascade_car_finder(stereo_image);

	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void subscribe_bumblebee_basic_messages(int camera_organization)
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera_organization, NULL,
			(carmen_handler_t) bumblebee_basic_stereoimage_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	if ((argc != 2))
		carmen_die("%s: Wrong number of parameters. This module requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>",
				argv[0], argc - 1, argv[0]);

	/* defining the camera to be used */
	camera = atoi(argv[1]);

	/* connect to IPC server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* TODO: I'll finish the detection part first, after i start the messages and 2D to 3D transform*/
	/* Define messages that your module publishes */
	//carmen_bumblebee_car_tracking_define_messages();

	subscribe_bumblebee_basic_messages(camera);



	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return 0;
}
