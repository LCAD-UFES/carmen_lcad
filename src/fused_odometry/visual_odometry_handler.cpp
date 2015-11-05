#include <carmen/carmen.h>
#include <carmen/visual_odometry_interface.h>
#include <tf.h>

#include "fused_odometry.h"
#include "visual_odometry_handler.h"
//#include "xsens_xyz_handler.h"


struct _visual_odometry_handler
{
};


static carmen_fused_odometry_control 
create_visual_odometry_control(carmen_visual_odometry_pose6d_message *visual_odometry_message)
{	
	carmen_fused_odometry_control v_control;

	v_control.v = visual_odometry_message->v;
	v_control.v_z = visual_odometry_message->velocity_6d.z;
	v_control.phi = visual_odometry_message->phi;
	v_control.v_pitch = visual_odometry_message->velocity_6d.pitch;
	v_control.z = visual_odometry_message->pose_6d.z;
	v_control.pitch = visual_odometry_message->pose_6d.pitch;
	v_control.roll = visual_odometry_message->pose_6d.roll;

	return v_control;
}


static void 
visual_odometry_message_handler(carmen_visual_odometry_pose6d_message *visual_odometry_message)
{
//	static FILE *fout = fopen("viso.txt","w");
//	fprintf(fout, "%lf %lf %lf\n", visual_odometry_message->v, visual_odometry_message->phi, visual_odometry_message->timestamp);
	carmen_fused_odometry_control ut = create_visual_odometry_control(visual_odometry_message);

	set_fused_odometry_control_vector(ut);

}


static void 
subscribe_messages(void)
{
	carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) visual_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


visual_odometry_handler *
create_visual_odometry_handler(int argc __attribute__ ((unused)), char ** argv __attribute__ ((unused)))
{
	visual_odometry_handler *viso_handler = (visual_odometry_handler*)malloc(sizeof(visual_odometry_handler));
	
	subscribe_messages();

	return viso_handler;
}


void 
reset_visual_odometry_handler(visual_odometry_handler* viso_handler __attribute__ ((unused)))
{
}


void 
destroy_visual_odometry_handler(visual_odometry_handler* viso_handler)
{
	free(viso_handler);
}
