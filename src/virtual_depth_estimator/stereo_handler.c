#include <carmen/carmen.h>
#include <carmen/stereo_interface.h>
#include <carmen/velodyne_camera_calibration.h>

#include "stereo_point_cloud_interface.h"

static carmen_simple_stereo_disparity_message stereo_message;
static carmen_stereo_point_cloud_message point_cloud_message;

static stereo_util su;


carmen_vector_3D_p
reproject_single_point_to_3D(const carmen_camera_parameters *camera, carmen_position_t right_point, double disparity)
{
	carmen_vector_3D_p p3D = NULL;

	if (disparity > 0)
	{
		double xr = right_point.x - camera->xc;
		double yr = right_point.y - camera->yc;
		double xl = xr + disparity;

		double fx_fy = camera->fx / camera->fy;

		// transform 2D image point to 3D Carmen coordinate frame
		double X = -(camera->baseline * camera->fx) / (xr - xl);
		double Y = (camera->baseline * (xr + xl)) / (2.0 * (xr - xl));
		double Z = -fx_fy * (camera->baseline * yr) / (xr - xl);

		if (!(isnan(X) || isnan(Y) || isnan(Z) ||
				isinf(X) ||	isinf(Y) ||	isinf(Z)))
		{
			p3D = (carmen_vector_3D_p)malloc(sizeof(carmen_vector_3D_t));
			carmen_test_alloc(p3D);

			p3D->x = X;
			p3D->y = Y;
			p3D->z = Z;
		}
	}
	return p3D;
}

void reproject_to_3D(float *disparityMap, carmen_vector_3D_t *image3D, double theta, carmen_camera_parameters *camera_params, int camera_index)
{
  float disparity;
  float MIN_DISPARITY = 0.1f;

  int x, y;
  for(y = 0; y < camera_params[camera_index].height; y++)
  {
    for(x = 0; x < camera_params[camera_index].width; x++)
    {
      disparity = disparityMap[y * camera_params[camera_index].width + x];
      disparity = fmax(disparity, MIN_DISPARITY);

      carmen_position_t P_r;
      P_r.x = x;
      P_r.y = y;

      carmen_vector_3D_p P_world_raw = reproject_single_point_to_3D(&camera_params[camera_index], P_r, disparity);
      carmen_vector_3D_p P_world = transform_camera_to_world_frame(P_world_raw, theta);

      image3D[y * camera_params[camera_index].width + x].x = P_world->x;
      image3D[y * camera_params[camera_index].width + x].y = P_world->y;
      image3D[y * camera_params[camera_index].width + x].z = P_world->z;

      free(P_world_raw);
      free(P_world);
    }
  }
}

static void 
publish_stereo_point_cloud(void)
{ 			
	int err = IPC_publishData(CARMEN_STEREO_POINT_CLOUD_NAME, &point_cloud_message); 	
	carmen_test_ipc_exit(err, "Could not publish stereo point cloud", CARMEN_STEREO_POINT_CLOUD_NAME);
}

static void stereo_message_handler(unsigned int image_width, unsigned int image_height)
{	
	carmen_vector_3D_t *image3D = malloc( image_width * image_height * sizeof(carmen_vector_3D_t) );

	reproject_to_3D(stereo_message.disparity, image3D, 0.0, su);

	point_cloud_message.num_points = image_width * image_height;
	point_cloud_message.points = realloc(point_cloud_message.points, point_cloud_message.num_points * sizeof(carmen_vector_3D_t));
	point_cloud_message.point_color = realloc(point_cloud_message.point_color, point_cloud_message.num_points * sizeof(carmen_vector_3D_t));

	int i;
	for(i=0; i<point_cloud_message.num_points; i++)
	{
		point_cloud_message.points[i].x = image3D[i].x;
		point_cloud_message.points[i].y = image3D[i].y;
		point_cloud_message.points[i].z = image3D[i].z;

		point_cloud_message.point_color[i].x = ((double)stereo_message.reference_image[3*i])/255.0;
		point_cloud_message.point_color[i].y = ((double)stereo_message.reference_image[3*i+1])/255.0;
		point_cloud_message.point_color[i].z = ((double)stereo_message.reference_image[3*i+2])/255.0;

	}

	point_cloud_message.timestamp = stereo_message.timestamp;
	point_cloud_message.host = carmen_get_host();

	publish_stereo_point_cloud();

	free(image3D);
}

static void 
register_ipc_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_STEREO_POINT_CLOUD_NAME, IPC_VARIABLE_LENGTH, CARMEN_STEREO_POINT_CLOUD_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_STEREO_POINT_CLOUD_NAME);

}


static void 
initialize_carmen_parameters(int argc, char** argv)
{
	int num_items;

	carmen_param_t param_list[] = {
		{"bumblebee", "basic6_width", CARMEN_PARAM_INT, &image_width, 0, NULL},
		{"bumblebee", "basic6_height", CARMEN_PARAM_INT, &image_height, 0, NULL}
		};
	
	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	su = get_stereo_instance(6, image_width, image_height);

	point_cloud_message.points = NULL;
}

int 
main(int argc, char** argv)
{ 	
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	initialize_carmen_parameters(argc, argv);
	register_ipc_messages();
	carmen_stereo_subscribe(6, &stereo_message, (carmen_handler_t) stereo_message_handler, CARMEN_SUBSCRIBE_LATEST);
  
	carmen_ipc_dispatch();

	carmen_ipc_disconnect();

	return 0;
}
