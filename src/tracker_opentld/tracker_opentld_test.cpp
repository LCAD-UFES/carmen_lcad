/*
 * tracker_opentld_test.cpp
 *
 *  Created on: 07/07/2016
 *      Author: Vinicius
 */
#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_visual_tracker_output_message_handler(carmen_visual_tracker_output_message *msg)
{
	fprintf( stderr, "===================================\n" );
	fprintf( stderr, "        Tracker OpenTLD message\n" );
	fprintf( stderr, "===================================\n" );
	fprintf( stderr, " x:					%d\n", msg->rect.x );
	fprintf( stderr, " y:					%d\n", msg->rect.y );
	fprintf( stderr, " width:				%d\n", msg->rect.width );
	fprintf( stderr, " height:				%d\n", msg->rect.height );
	fprintf( stderr, " confidence:			%lf\n", msg->confidence );
	fprintf( stderr, " timestamp:			%lf\n", msg->timestamp );
	fprintf( stderr, "===================================\n" );
	fprintf( stderr, "\n" );

}


///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribers                                                                               //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_visual_tracker_subscribe_messages()
{
	carmen_visual_tracker_subscribe_output(NULL, (carmen_handler_t) carmen_visual_tracker_output_message_handler,CARMEN_SUBSCRIBE_LATEST);
}


///////////////////////////////////////////////////////////////////////////////////////////////


static void
shutdown_camera_view(int x)
{
    if (x == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("Disconnected. \n");
        exit(0);
    }
}


int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_camera_view);

	carmen_visual_tracker_subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}

