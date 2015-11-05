#include "Media/IpcFrameSeries.H"

#include "Media/FrameSeries.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimulationOpts.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"
#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/Pixels.H"

#include "../saliency_search_interface.h"
#include "../message_interpolation.cpp"

//#define USE_DEPTH
//#define GENERATE_DATA

//Used for dynamaicly loaded modules
SIMMODULEINSTFUNC(IpcInputFrameSeries);

GenericFrame frame;

static int saliency_points_list_size = 0;
static carmen_saliency_search_saliency_points_message saliency_points;

static long int frame_counter = -1;
static int has_new_frame = 0;
static int read_next_image = 1;
static int saliency_counter = 0;
static int publishing_saliencies = 0;
static int has_message_to_publish = 0;
static int stereo_received = 0;

static stereo_util camera_instance;
static int image_height, image_width;

unsigned short* temp_depth_map = NULL;
float* temp_disparity_map = NULL;

//MessageInterpolation<carmen_fused_odometry_message, carmen_simple_stereo_disparity_message> interpolator(24);
MessageInterpolation<carmen_localize_ackerman_globalpos_message, carmen_simple_stereo_disparity_message> interpolator(1);

void
alloc_saliency_points_structure()
{
	saliency_points.image_height = image_height;
	saliency_points.image_width = image_width;
	saliency_points.image_size = image_height * image_width * 3;
	saliency_points.saliency_list_size = saliency_points_list_size;

	saliency_points.saliency_list =  (carmen_saliency_t *) calloc (saliency_points.saliency_list_size, sizeof(carmen_saliency_t));
	carmen_test_alloc(saliency_points.saliency_list);
	saliency_points.reference_image = (unsigned char *) calloc (saliency_points.image_size, sizeof(unsigned char));
	carmen_test_alloc(saliency_points.reference_image);
}

void
copy_disparity_to_saliency_points_message(carmen_simple_stereo_disparity_message* message)
{
	saliency_points.timestamp = message->timestamp;
	saliency_points.host = carmen_get_host();

	memcpy(saliency_points.reference_image, message->reference_image, image_height * image_width * 3);
}

GenericFrame convert_disparity_message_to_generic_frame(carmen_simple_stereo_disparity_message* message)
{
	Image< PixRGB<byte> > rgbimg(Dims(image_width, image_height), ZEROS);

	get_depth_map_from_disparity_map(message->disparity, temp_depth_map, camera_instance, 50.0);

	for(int i = 0; i < (image_width * image_height); i++)
	{
		temp_disparity_map[i] = message->disparity[i];

		if(i > (image_width * 170) && i < ( image_width * (image_height - 110)) && temp_depth_map[i] <= 12000)
		{
			//temp_depth_map[i] = 12000.0 - temp_depth_map[i];
			rgbimg[i].p[0] = message->reference_image[3 * i];
			rgbimg[i].p[1] = message->reference_image[3 * i + 1];
			rgbimg[i].p[2] = message->reference_image[3 * i + 2];
		}
		else
		{
			temp_depth_map[i] = 0.0;
			rgbimg[i].p[0] = message->reference_image[3 * i] = 0.0;
			rgbimg[i].p[1] = message->reference_image[3 * i + 1] = 0.0;
			rgbimg[i].p[2] = message->reference_image[3 * i + 2] = 0.0;
		}
	}

	Image<unsigned short> depthimg(temp_depth_map, Dims(image_width, image_height));
	GenericFrame gf(rgbimg, depthimg);

	return gf;
}

void
carmen_saliency_define_saliency_points_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
						CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_NAME);
}

int
writePpm( char* 	szFilename,
		unsigned char* pucBuffer,
		int		width,
		int		height )
{
	FILE* stream;
	stream = fopen( szFilename, "wb" );
	if( stream == NULL)
	{
		perror( "Can't open image file" );
		return 1;
	}

	fprintf( stream, "P6\n%u %u 255\n", width, height );
	fwrite( pucBuffer, 3*width, height, stream );
	fclose( stream );
	return 0;
}

void
publish_saliency_points_message()
{
	FILE* fd;
	char image[256];

	sprintf(image, "/media/OS/Users/Lauro/Downloads/Log/global_map/datasets3/2012/%03dl.ppm", frame_counter);
	writePpm(image, saliency_points.reference_image, saliency_points.image_width, saliency_points.image_height);

	fd = fopen("/media/OS/Users/Lauro/Downloads/Log/global_map/datasets3/2012/data.txt", "a+");

	fprintf(fd, "%s ", image);
	fprintf(fd, "%f %f %f %f %f %f", saliency_points.pose.position.x,
									  saliency_points.pose.position.y,
									  saliency_points.pose.position.z,
									  saliency_points.pose.orientation.yaw,
									  saliency_points.pose.orientation.pitch,
									  saliency_points.pose.orientation.roll);

	for(int i = 0; i < saliency_points.saliency_list_size; i++)
	{
		fprintf(fd, " %d %d %f %f %f", saliency_points.saliency_list[i].coordinates.x, saliency_points.saliency_list[i].coordinates.y, saliency_points.saliency_list[i].pose.x, saliency_points.saliency_list[i].pose.y, saliency_points.saliency_list[i].pose.z);
	}

	fprintf(fd, "\n");
	fclose(fd);

//	  IPC_RETURN_TYPE err;
//
//	  publishing_saliencies = 1;
//	  err = IPC_publishData(CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_NAME, &saliency_points);
//	  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_FMT);
}

//void
//carmen_fused_odometry_odometry_message_handler(carmen_fused_odometry_message* message)
//{
//	interpolator.AddMessageToInterpolationList(message);
//}

void
carmen_localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message* message)
{
	interpolator.AddMessageToInterpolationList(message);
}

void
carmen_simple_stereo_disparity_handler(carmen_simple_stereo_disparity_message* message)
{
	static carmen_world_point_t previous_pose;
	carmen_world_point_t current_pose;

	carmen_localize_ackerman_globalpos_message nearest_message;
	nearest_message = interpolator.InterpolateMessages(message);

	current_pose.pose.x = nearest_message.pose.position.x;
	current_pose.pose.y = nearest_message.pose.position.y;
	current_pose.pose.theta = nearest_message.pose.orientation.yaw;

	double distance = carmen_distance_world(&current_pose, &previous_pose);

	if(has_message_to_publish)
	{
		publish_saliency_points_message();
		has_message_to_publish = 0;
	}

	if(fabs(distance - 1.0) < 0.10 || distance >= 1.0)
	{
		previous_pose = current_pose;

		if(read_next_image)
		{
			saliency_points.pose = nearest_message.pose;
			copy_disparity_to_saliency_points_message(message);
			frame = convert_disparity_message_to_generic_frame(message);
			frame_counter++;
			has_new_frame = 1;
			read_next_image = 0;
		}
	}
}

IpcInputFrameSeries::
IpcInputFrameSeries(OptionManager& mgr, int camera, int list_size, int width, int height,
		const std::string& descrName,
		const std::string& tagName) :
		SimModule(mgr, descrName, tagName),
		SIMCALLBACK_INIT(SimEventClockTick),
		SIMCALLBACK_INIT(SimEventWTAwinner)
{
	image_width = width;
	image_height = height;
	has_new_frame = 0;
	saliency_points_list_size = list_size;

	carmen_saliency_define_saliency_points_message();

	camera_instance = get_stereo_instance(camera, image_width, image_height);

	alloc_saliency_points_structure();
	temp_depth_map = (unsigned short *) calloc (image_height * image_width, sizeof(unsigned short));
	temp_disparity_map = (float *) calloc (image_height * image_width, sizeof(float));

	carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) carmen_simple_stereo_disparity_handler, CARMEN_SUBSCRIBE_ALL);

	//carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) carmen_fused_odometry_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,  (carmen_handler_t) carmen_localize_ackerman_globalpos_handler, CARMEN_SUBSCRIBE_ALL);
}

IpcInputFrameSeries::
~IpcInputFrameSeries()
{ }

// ######################################################################
void IpcInputFrameSeries::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
	static int first_sim_event_clock = 1;

	if(first_sim_event_clock)
	{
		Image< PixRGB<byte> > rgbimg(Dims(image_width, image_height), ZEROS);
		GenericFrame gf(rgbimg);

		rutz::shared_ptr<SimEventInputFrame>
					ev(new SimEventInputFrame(this, gf, 0));
					q.post(ev);

		first_sim_event_clock = 0;
	}

	if(has_new_frame)
	{
		has_new_frame = 0;

		if (frame.initialized())
		{
			rutz::shared_ptr<SimEventInputFrame>
			ev(new SimEventInputFrame(this, frame, frame_counter));
			q.post(ev);
		}
	}
}

void
IpcInputFrameSeries::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
	float disparity;
	double depth;
	const WTAwinner winner = e->winner();

	depth = temp_depth_map[winner.p.j * image_width + winner.p.i];
	disparity = temp_disparity_map[winner.p.j * image_width + winner.p.i];

	if(!read_next_image)
	{
		if(depth <= 12000)
		{
			saliency_points.saliency_list[saliency_counter].coordinates.x = winner.p.i;
			saliency_points.saliency_list[saliency_counter].coordinates.y = winner.p.j;

			carmen_position_t xy_coordinates;
			xy_coordinates.x = saliency_points.saliency_list[saliency_counter].coordinates.x;
			xy_coordinates.y = saliency_points.saliency_list[saliency_counter].coordinates.y;

			carmen_vector_3D_p point3D =  reproject_single_point_to_3D(&camera_instance, xy_coordinates, disparity);

			if(point3D != NULL)
			{
				saliency_points.saliency_list[saliency_counter].pose = *point3D;
				free(point3D);

				saliency_counter++;
			}

			if(saliency_counter > (saliency_points_list_size - 1))
			{
				has_message_to_publish = 1;
				read_next_image = 1;
				saliency_counter = 0;
			}
		}
	}
}
