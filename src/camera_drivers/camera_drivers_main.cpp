#include "camera_drivers.h"
#include "camera_drivers_process_image.hpp"


// FFmpeg library
av_ffmpeg *av_ffmpeg_params;

cv::VideoCapture cap;

void
check_parameters(int argc, char **argv)
{
	if (argc < 3)
	{
		carmen_die("---------------------------------------------\n Wrong number of parameters! \n---------------------------------------------\n\nUSAGE: %s <camera_name> <camera_id>\n\n", argv[0]);
		exit (0);
	}
}


int
allocate_image_data_and_compute_size_in_bytes_of_each_element(int data_type, int size, camera_image *img)
{
	switch (data_type)
	{
	case char_data:
		img->raw_data = (char*) malloc (size * sizeof (char));
		return (sizeof (char));

	case unsigned_char_data:
		img->raw_data = (unsigned char*) malloc (size * sizeof (unsigned char));
		return (sizeof (unsigned char));

	case signed_char_data:
		img->raw_data = (signed char*) malloc (size * sizeof (signed char));
		return (sizeof (signed char));

	case int_data:
		img->raw_data = (int*) malloc (size * sizeof (int));
		return (sizeof (int));

	case unsigned_int_data:
		img->raw_data = (unsigned int*) malloc (size * sizeof (unsigned int));
		return (sizeof (unsigned int));

	case short_data:
		img->raw_data = (short*) malloc (size * sizeof (short));
		return (sizeof (short));

	case unsigned_short_data:
		img->raw_data = (unsigned short*) malloc (size * sizeof (unsigned short));
		return (sizeof (unsigned short));

	case long_data:
		img->raw_data = (long*) malloc (size * sizeof (long));
		return (sizeof (long));

	case unsigned_long_data:
		img->raw_data = (unsigned long*) malloc (size * sizeof (unsigned long));
		return (sizeof (unsigned long));

	case float_data:
		img->raw_data = (float*) malloc (size * sizeof (float));
		return (sizeof (float));

	case double_data:
		img->raw_data = (double*) malloc (size * sizeof (double));
		return (sizeof (double));

	case long_double_data:
		img->raw_data = (long double*) malloc (size * sizeof (long double));
		return (sizeof (long double));

	default:
		carmen_die("---------------------------------------------\n Data type not found! \n---------------------------------------------\n\n");
		exit(0);
	}
}


void
setup_image_struct(int width, int height, int number_of_channels, int data_type, camera_image *image)
{
	image->width = width;
	image->height = height;
	image->number_of_channels = number_of_channels;
	image->size_in_bytes_of_each_element = allocate_image_data_and_compute_size_in_bytes_of_each_element(data_type, (width * height * number_of_channels), image);
	image->image_size = (width * height * number_of_channels * image->size_in_bytes_of_each_element);
	image->data_type = data_type;
}


void
setup_message(camera_message *message, int number_of_images)
{
	message->number_of_images = number_of_images;
	message->images = (camera_image*) malloc (number_of_images * sizeof (camera_image));
	message->undistorted = 0;
	message->host = carmen_get_host();
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers																			     //
///////////////////////////////////////////////////////////////////////////////////////////////


void 
publish_image_message(int camera_id, camera_message *message)
{	
	camera_drivers_define_message(camera_id);
	camera_drivers_publish_message(camera_id, message);
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers																					 //
///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
    if (signo == SIGINT)
	{
		if (av_ffmpeg_params)
		{
			if (!av_ffmpeg_params->use_with_opencv)
			{
				if (av_ffmpeg_params->decframe) av_frame_free(&av_ffmpeg_params->decframe);
				if (av_ffmpeg_params->frame) av_frame_free(&av_ffmpeg_params->frame);
				if (av_ffmpeg_params->avctx) avcodec_free_context(&av_ffmpeg_params->avctx);
				if (av_ffmpeg_params->inctx) avformat_close_input(&av_ffmpeg_params->inctx);
			}

			// if (av_ffmpeg_params->framebuf) delete(av_ffmpeg_params->framebuf);
			// if (av_ffmpeg_params->vcap) delete(av_ffmpeg_params->vcap);
			free(av_ffmpeg_params);
		}
        carmen_ipc_disconnect();
        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations																		     //
///////////////////////////////////////////////////////////////////////////////////////////////


int
find_camera_model(char *smodel)
{
	if (strcmp(smodel, "udp_rtp_h264") == 0)
		return (udp_rtp_h264);
	if (strncmp(smodel, "vip", 3) == 0)
		return (ip_opencv);
	if (strcmp(smodel, "ip_ffmpeg") == 0)
		return (ip_ffmpeg);
	if (strcmp(smodel, "ip_opencv") == 0)
		return (ip_opencv);
	return (usb_opencv);
}


void
read_parameters(int argc, char **argv, int &camera_id, char **camera_name, char **camera_model, 
	int &number_of_images, double &frame_rate, int &undistort, char **ip_address, char **port, char **network_interface, double &resize_factor)
{
	char *camera_name_, *camera_model_, *ip_address_, *port_, *network_interface_;
	camera_name_ = argv[1];
	camera_id = atoi(argv[2]);

	carmen_param_allow_unfound_variables(0);
	carmen_param_t param_list[] = 
	{
		{camera_name_, 			 (char*)"model",            CARMEN_PARAM_STRING, &camera_model_,    0, NULL},
		{camera_name_, 			 (char*)"number_of_images", CARMEN_PARAM_INT,    &number_of_images, 0, NULL},
		{camera_name_, 			 (char*)"fps",              CARMEN_PARAM_DOUBLE, &frame_rate,       0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

	av_ffmpeg_params = nullptr;
	if (strncmp(camera_model_, "ip", 2) == 0)
	{
		carmen_param_t param_list_[] = 
		{
			{camera_name_, (char*)"ip_adress", 	 CARMEN_PARAM_STRING, &ip_address_, 0, NULL},
		};
		carmen_param_install_params(argc, argv, param_list_, sizeof(param_list_)/sizeof(param_list_[0]));

		av_ffmpeg_params = (av_ffmpeg *)malloc(sizeof(av_ffmpeg));

		*ip_address = (char*) malloc((strlen(ip_address_)+1)*sizeof(char));
		strcpy(*ip_address, ip_address_);
	}
	else if (strcmp(camera_model_, "udp_rtp_h264") == 0) 
	{
		carmen_param_t param_list_[] = 
		{
			{camera_name_, (char*)"ip_adress", 	 	   CARMEN_PARAM_STRING, &ip_address_,        0, NULL},
			{camera_name_, (char*)"port", 	           CARMEN_PARAM_STRING, &port_,              0, NULL},
			{camera_name_, (char*)"network_interface", CARMEN_PARAM_STRING, &network_interface_, 0, NULL},
		};
		carmen_param_install_params(argc, argv, param_list_, sizeof(param_list_)/sizeof(param_list_[0]));

		*ip_address = (char*) malloc((strlen(ip_address_)+1)*sizeof(char));
		strcpy(*ip_address, ip_address_);

		*port = (char*) malloc((strlen(port_)+1)*sizeof(char));
		strcpy(*port, port_);

		*network_interface = (char*) malloc((strlen(network_interface_)+1)*sizeof(char));
		strcpy(*network_interface, network_interface_);
	}

	*camera_name = (char*) malloc((strlen(camera_name_)+1)*sizeof(char));
	strcpy(*camera_name, camera_name_);
	*camera_model = (char*) malloc((strlen(camera_model_)+1)*sizeof(char));
	strcpy(*camera_model, camera_model_);

	carmen_param_allow_unfound_variables(1);
	carmen_param_t param_list_[] = 
	{
		{(char *) "commandline", (char*)"undistort",        CARMEN_PARAM_ONOFF,  &undistort,        0, NULL},
		{camera_name_, 			 (char*)"resize_factor",    CARMEN_PARAM_DOUBLE, &resize_factor,    0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list_, sizeof(param_list_)/sizeof(param_list_[0]));

	// if (undistort && (strcmp(*camera_model, "ip_ffmpeg") == 0))
	// {
	// 	printf("undistortion not defined for ip_ffmpeg mode.\n");
	// 	undistort = 0;
	// }
}


int
main(int argc, char **argv)
{
	unsigned char *raw = NULL;
	char *camera_model, *ip_address, *port, *network_interface, *camera_name = NULL;
	camera_message message;
	int camera_id, number_of_images, model, width, height, image_index = 0, undistort = 1;
	static int first_time = 1;
	double resize_factor = 1.0, frame_rate = 9999, time_slot, time = 0.0;
	
	check_parameters(argc, argv);
    
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	read_parameters(argc, argv, camera_id, &camera_name, &camera_model, 
		number_of_images, frame_rate, undistort, &ip_address, &port, &network_interface, resize_factor);
	setup_message(&message, number_of_images);

	time_slot = (1 / frame_rate) * 0.9;

	model = find_camera_model(camera_model);

	while (1)
	{
		switch (model)
		{
		case udp_rtp_h264:
			raw = get_image_udp_rtp(cap, ip_address, port, network_interface, width, height);
			break;

		case ip_opencv:
			raw = get_image_ip_camera_with_opencv(ip_address, undistort, camera_name, width, height);
			break;

		case ip_ffmpeg:
			raw = get_image_ip_camera_with_ffmmpeg(ip_address, undistort, camera_name, av_ffmpeg_params, width, height);
			break;

		case usb_opencv:
			raw = get_image_usb_camera_with_opencv(camera_model, undistort, camera_name, width, height);
			break;

		default:
			break;
		}

		if (!raw)
			continue;
		
		if (first_time)
		{
			if (fabs(resize_factor - 1.0) <= 1e-5)
				resize_factor = 1.0;
			for (int i = 0; i < number_of_images; i++)
				setup_image_struct(width * resize_factor, height * resize_factor, 3, 1, &(message.images[i]));

			first_time = 0;
		}

		if (fabs(resize_factor - 1.0) > 1e-5)
		{
			cv::Mat image(height, width, CV_8UC3, raw, 0);
			cv::resize(image, image, Size(image.cols * resize_factor, image.rows * resize_factor));

			message.images[image_index].width = image.cols;
			message.images[image_index].height = image.rows;
			raw = image.data;		
		}

		message.images[image_index].raw_data = raw;
		if (undistort)
			message.undistorted = 1;

		if ((carmen_get_time() - time) > time_slot)
		{
			time = carmen_get_time();
			message.timestamp = time;
			publish_image_message(camera_id, &message);
		}
	}
}
