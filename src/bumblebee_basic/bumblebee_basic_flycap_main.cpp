/*
 * bumblebee_basic_flycap_main.c
 *
 *  Created on: 19/09/2013
 *      Author: romulo
 */

#include <stdio.h>
#include <flycapture/FlyCapture2.h>
#include <stdlib.h>
#include <unistd.h>
#include <libbee.hpp>
#include <dc1394/conversions.h>
#include <dc1394/register.h>
#include <signal.h>
#include "triclops.h"
#include <sys/time.h>
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <flycapture/Utilities.h>
#include <pthread.h>

#include <gtk/gtk.h>

typedef struct {
	FlyCapture2::PropertyType type;
	char* property_name;
	double range_max;
	double range_min;
	double increment;
	GtkWidget *scale_button;
	GtkWidget *check_button_auto;
} PropertyGTK;


PropertyGTK property_vector[50];
int property_vector_size;

/**
 * If not using format 7 pan select which sensor will send the image
 * PAN_VALUE_0 - bb2: right, xb3: right
 * PAN_VALUE_1 - bb2: left, xb3: center
 * PAN_VALUE_2 - bb2: none, xb3: left
 *
 * If using format7, mode3 and RAW16:
 * PAN_VALUE_0 - will retorn in xb3 right and left image
 * PAN_VALUE_1 - will retorn in xb3 righ and center image
 */
#define PAN_ADDRESS 0x884
#define PAN_VALUE_0 0x82000000
#define PAN_VALUE_1 0x82000001
#define PAN_VALUE_2 0x82000002

#define REG_CONFIG_LENGTH         		0x1FFC
#define REG_CONFIG_DATA           		0x2000
#define REG_UNIT_DIRECTORY_OFFSET      	0x0424

#define IMAGE_DATA_FORMAT_REGISTER		0x1048
#define IMAGE_DATA_FORMAT_BIG_ENDIAN 	0x80000001
#define IMAGE_DATA_FORMAT_LITTLE_ENDIAN 0x80000000



using namespace FlyCapture2;

static Camera cam;
static TriclopsContext triclops;
static int camera_id;

static carmen_bumblebee_basic_stereoimage_message bumblebee_msg;

static unsigned int pan_value = PAN_VALUE_0;
static int _rectify_image = 1;
static int use_configuration_gui = 0;

double fps;

void
handle_error(Error &error)
{
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		exit(-1);
	}
}

void
initialize_bumblebee_message(int width, int height, int bytes_per_pixel, int isRectified)
{
	bumblebee_msg.host = carmen_get_host();
	bumblebee_msg.image_size = width*height*bytes_per_pixel;
	bumblebee_msg.width = width;
	bumblebee_msg.isRectified = isRectified;
	bumblebee_msg.height = height;
}

void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, double timestamp, int camera)
{
	bumblebee_msg.raw_left = rawLeft;
	bumblebee_msg.raw_right = rawRight;
	bumblebee_msg.timestamp = timestamp;

	carmen_bumblebee_basic_publish_message(camera, &bumblebee_msg);
}

static void
print_camera_info( CameraInfo* pCamInfo )
{
	printf(
			"\n*** CAMERA INFORMATION ***\n"
			"Serial number - %u\n"
			"Camera model - %s\n"
			"Camera vendor - %s\n"
			"Sensor - %s\n"
			"Resolution - %s\n"
			"Firmware version - %s\n"
			"Firmware build time - %s\n\n",
			pCamInfo->serialNumber,
			pCamInfo->modelName,
			pCamInfo->vendorName,
			pCamInfo->sensorInfo,
			pCamInfo->sensorResolution,
			pCamInfo->firmwareVersion,
			pCamInfo->firmwareBuildTime );
}

static void
deinterlace_rgb_single( unsigned char* src,
		unsigned char* destR,
		unsigned char* destG,
		unsigned char* destB,
		unsigned int width,
		unsigned int height)
{
	register unsigned int i = 0;
	register unsigned int j = 0;

	while (i < (width * height)) {
		destR[j] = src[i++];
		destG[j] = src[i++];
		destB[j++] = src[i++];
	}
}

static void
interpolate_triclops_stereo_image(unsigned char* outImageLeft, TriclopsColorImage *inputImageLeft,
		unsigned char* outImageRight, TriclopsColorImage *inputImageRight,
		int width, int height)
{
	//#pragma omp parallel for shared(width, height, outImageLeft, outImageRight, inputImageLeft, inputImageRight)
	for (int i = 0; i < (width * height); i++)
	{
		outImageLeft[3*i+0] = inputImageLeft->red[i];
		outImageLeft[3*i+1] = inputImageLeft->green[i];
		outImageLeft[3*i+2] = inputImageLeft->blue[i];

		outImageRight[3*i+0] = inputImageRight->red[i];
		outImageRight[3*i+1] = inputImageRight->green[i];
		outImageRight[3*i+2] = inputImageRight->blue[i];
	}
}

static void
rectify_image(
		unsigned char *rgb_image,
		unsigned char *rectfied_rgb_right,
		unsigned char *rectfied_rgb_left)
{
	static unsigned char *r_vector = new unsigned char[1280*960*2];
	static unsigned char *g_vector = new unsigned char[1280*960*2];
	static unsigned char *b_vector = new unsigned char[1280*960*2];

	unsigned char * aux;
	TriclopsInput	input;
	TriclopsColorImage  img_right, img_left;

	deinterlace_rgb_single(rgb_image, r_vector, g_vector, b_vector, 1280, 6 * 960);

	//#pragma omp parallel sections private(input, aux) shared(img_right, img_left, triclops, r_vector, g_vector, b_vector)
	{
		//		#pragma omp section
		{
			input.inputType 	= TriInp_RGB;
			input.nrows	= 960;
			input.ncols	= 1280;
			input.rowinc	= 1280;
			input.u.rgb.red   	= r_vector;
			input.u.rgb.green 	= g_vector;
			input.u.rgb.blue  	= b_vector;

			triclopsRectifyColorImage( triclops, TriCam_RIGHT, &input, &img_right );
		}

		//		#pragma omp section
		{
			input.inputType 	= TriInp_RGB;
			input.nrows	= 960;
			input.ncols	= 1280;
			input.rowinc	= 1280;
			aux = (unsigned char*)r_vector;
			input.u.rgb.red = &aux[1280 * 960];
			aux = (unsigned char*)g_vector;
			input.u.rgb.green = &aux[1280 * 960];
			aux = (unsigned char*)b_vector;
			input.u.rgb.blue = &aux[1280 * 960];

			triclopsRectifyColorImage( triclops, TriCam_LEFT, &input, &img_left );
		}
	}

	interpolate_triclops_stereo_image(rectfied_rgb_left, &img_left, rectfied_rgb_right, &img_right, 1280, 960);
}

void
image_grabber_handler(FlyCapture2::Image* pImage)
{
	static unsigned char *deinterleaved = new unsigned char[1280*960*2];
	static unsigned char *rgb = new unsigned char[1280*960*2*3];

	static unsigned char *rectfied_rgb_right = new unsigned char[1280*960*3];
	static unsigned char *rectfied_rgb_left = new unsigned char[1280*960*3];

	double timestamp;

	timestamp = carmen_get_time();


	dc1394_deinterlace_stereo( pImage->GetData(),
			deinterleaved,
			1280,
			2*960 );

	dc1394_bayer_decoding_8bit( deinterleaved,
			rgb,
			1280,
			2*960,
			DC1394_COLOR_FILTER_GBRG,
			DC1394_BAYER_METHOD_NEAREST );

	if (_rectify_image)
	{
		rectify_image(rgb, rectfied_rgb_right, rectfied_rgb_left);
		carmen_bumblebee_publish_stereoimage_message(rectfied_rgb_left, rectfied_rgb_right, timestamp, camera_id);
	}
	else
	{
		carmen_bumblebee_publish_stereoimage_message(rgb, rgb + 1280 * 960 * 3, timestamp, camera_id);
	}
}

//1280×960×8  --- 2560
//total_bits   = package_size
// bits_per_package = 3840
// total_bits / bits_per_package = package_size

//5120

void
set_camera_PAN()
{
	cam.WriteRegister(PAN_ADDRESS, pan_value, true);
}

void
set_buffer_size(int num_buffer)
{
	FC2Config config;
	config.numBuffers = num_buffer;
	config.grabMode = DROP_FRAMES;
	config.highPerformanceRetrieveBuffer = true;

	cam.SetConfiguration(&config);
}

void
configure_format7()
{
	Format7ImageSettings format7;
	format7.mode = MODE_3;
	format7.pixelFormat = PIXEL_FORMAT_RAW16;
	format7.width = 1280;
	format7.height = 960;
	cam.SetFormat7Configuration(&format7, (float)100);
}

void
calculate_max_fps()
{
	Format7ImageSettings format7;
	unsigned int package_size;
	float porcentage;
	unsigned int package_ideal_size = 1280.0 * 960.0 * 16.0 / 3840.0;


	cam.GetFormat7Configuration(&format7, &package_size, &porcentage);

	printf("package_size=%d package_ideal_size=%d porcentage=%f\n", package_size, package_ideal_size, porcentage);
	fps = carmen_clamp(1.0, ((package_size * 16.0) / (double)package_ideal_size), 16.0);

	if (package_size < package_ideal_size)
	{
		fps = carmen_clamp(1.0, fps - 1.0, fps);
	}

	printf("fps = %f\n", fps);
}

void
print_camera_info()
{
	Error error;
	CameraInfo camInfo;
	error = cam.GetCameraInfo(&camInfo);

	handle_error(error);

	print_camera_info(&camInfo);
}

void
update_gui_property_values(bool update_all = true)
{
	Property prop;
	Error error;
	gdk_threads_enter();
	for (int i = 0; i < property_vector_size; i++)
	{
		prop.type = property_vector[i].type;

		if (!update_all && !gtk_toggle_button_get_active((GtkToggleButton*)property_vector[i].check_button_auto))
			continue;

		error = cam.GetProperty(&prop);
		handle_error(error);


		gtk_toggle_button_set_active((GtkToggleButton*)property_vector[i].check_button_auto, prop.autoManualMode);

		if (update_all || prop.autoManualMode)
		{
			if (prop.type == PAN)
				gtk_range_set_value((GtkRange *)property_vector[i].scale_button, prop.valueA);
			else
				gtk_range_set_value((GtkRange *)property_vector[i].scale_button, prop.absValue);
		}
	}
	gdk_threads_leave();
}

void
capture_image()
{
	FlyCapture2::Image image;
	double time_to_sleep, t1;
	Error error;

	error = cam.StartCapture();

	handle_error(error);

	int update_count = 0;


	while (true)
	{
		error = cam.RetrieveBuffer(&image);
		handle_error(error);

		t1 = carmen_get_time();
		image_grabber_handler(&image);

		update_count = (update_count + 1) % (int)fps;

		if (use_configuration_gui && update_count == 0)
			update_gui_property_values(false);

		time_to_sleep = (1.0 / fps) - (carmen_get_time() - t1);


		if (time_to_sleep >= 0)
			usleep(time_to_sleep * 1000000);
	}
}

void
connect_camera(PGRGuid guid)
{
	Error error;

	// Connect to a camera
	error = cam.Connect(&guid);

	handle_error(error);
}

int RunSingleCamera( PGRGuid guid )
{
	connect_camera(guid);

	cam.WriteRegister(IMAGE_DATA_FORMAT_REGISTER, IMAGE_DATA_FORMAT_LITTLE_ENDIAN, true);

	set_camera_PAN();

	set_buffer_size(2);

	configure_format7();

	calculate_max_fps();

	// Get the camera information
	print_camera_info();

	if (use_configuration_gui)
		update_gui_property_values();

	capture_image();

	return 0;
}

unsigned long long int CameraContextByGuid(int camera)
{
//readme_como_obter_guid.txt
	unsigned long long int guid = 0;

	switch(camera)
	{
	case 1:
		guid = 49712223527926590ll;
		break;

	case 2:
		guid = 49712223531755284ll;
		break;

	case 3:
		guid = 49712223533115246ll;
		break;

	case 4:
		guid = 49712223533115251ll;
		break;

	case 5:
		guid = 49712223533115245ll;
		break;

	case 6:
		guid = 49712223533115244ll;
		break;

	case 7:
		guid = 49712223533115250ll;
		break;

	case 8:
		guid = 49712223533115248ll;
		break;

	case 9:
		guid = 49712223532964068ll;
		break;

	default:
		guid = -1;
		break;
	}

	return guid;
}


static dc1394error_t
writeTriclopsConfigFromCameraToFile( dc1394camera_t* camera,
		const char* outputFile)
{
	dc1394error_t err;
	uint32_t 	ulQuadlet;

	err = dc1394_get_control_register( camera, REG_CONFIG_LENGTH, &ulQuadlet );
	if ( err != DC1394_SUCCESS )
	{
		fprintf(stderr, "dc1394_get_control_register(REG_CONFIG_LENGTH) failed\n");
		return err;
	}

	// the length of the config file
	unsigned long ulFileSizeBytes = ulQuadlet;
	if( ulFileSizeBytes == 0 )
	{
		fprintf( stderr, "File size == 0!\n" );
		return DC1394_FAILURE;
	}

	FILE* pfile = fopen( outputFile, "w" );
	if ( !pfile )
	{
		fprintf( stderr, "Can't open temporary file\n" );
		return DC1394_FAILURE;
	}

	// Read the config file, and save it to the output file,
	// while fixing endianness.
	for( unsigned long offset = 0 ; offset < ulFileSizeBytes; offset += 4 )
	{
		err = dc1394_get_control_register( camera,
				REG_CONFIG_DATA + offset,
				&ulQuadlet );

		if( err != DC1394_SUCCESS )
		{
			fprintf( stderr,
					"Can't get control register 0x%x\n",
					(int) (REG_CONFIG_DATA+offset) );
			fclose( pfile );
			return err;
		}


		for( int i = 24; i >= 0; i -= 8 )
		{
			fputc( ( (ulQuadlet>>i) & 0xFF ), pfile );
		}
	}

	fclose( pfile );

	return DC1394_SUCCESS;
}

static TriclopsError
get_triclops_context_from_camera( TriclopsContext* pTriclops )
{
	const char* tempFile = "/tmp/triclops.cal";
	TriclopsError  tErr;
	dc1394error_t err;
	dc1394camera_t*		camera;
	dc1394_t* 		d;

	d = dc1394_new ();
	camera = dc1394_camera_new(d, CameraContextByGuid(camera_id)/*49712223532964068ll*/);

	if (!camera)
	{
		fprintf(stderr, "Camera %d not found\n", camera_id);
		exit(1);
	}



	err = writeTriclopsConfigFromCameraToFile( camera, tempFile );

	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Couldn't write config data to file\n" );
		return TriclopsErrorSystemError;
	}

	tErr = triclopsGetDefaultContextFromFile( pTriclops, (char*) tempFile );

	if ( tErr != TriclopsErrorOk )
		fprintf( stderr, "triclopsGetDefaultContextFromFile failed!\n" );

	tErr = triclopsSetResolution( triclops, 960, 1280);

	tErr = triclopsSetRectImgQuality(triclops, TriRectQlty_STANDARD);

	tErr = triclopsSetSubpixelInterpolation( triclops, 1 );

	dc1394_camera_free(camera);
	dc1394_free(d);

	return tErr;
}

static void
shutdown_module(int signo)
{
	Error error;

	if (signo != SIGINT)
		return;

	// Stop capturing images
	error = cam.StopCapture();
	handle_error(error);

	// Disconnect the camera
	error = cam.Disconnect();
	handle_error(error);

	exit(0);
}

static void
read_parameters(int argc, char **argv)
{
	{
		carmen_param_t param_list[] = {
				{(char*)"command_line", (char*)"camera_id", CARMEN_PARAM_INT, &camera_id, 0, NULL},
		};

		carmen_param_install_params(argc, argv, param_list,  sizeof(param_list)/sizeof(param_list[0]));
	}

	{
		int use_extended_baseline = 1;

		_rectify_image = 1;


		carmen_param_t param_list[] = {
				{(char*)"command_line", (char*)"use_extended_baseline", CARMEN_PARAM_ONOFF, &use_extended_baseline, 0, NULL},
				{(char*)"command_line", (char*)"rectify_image", CARMEN_PARAM_ONOFF, &_rectify_image, 0, NULL},
				{(char*)"command_line", (char*)"use_configuration_gui", CARMEN_PARAM_ONOFF, &use_configuration_gui, 0, NULL},
		};


		carmen_param_allow_unfound_variables(true);

		carmen_param_install_params(argc, argv, param_list,  sizeof(param_list)/sizeof(param_list[0]));

		pan_value = use_extended_baseline ? PAN_VALUE_0 : PAN_VALUE_1;

		printf("PARAMS:\n"
				"\t-camera_id %d\n"
				"\t-use_extended_baseline %s\n"
				"\t-rectify_image %s\n"
				"\t-use_configuration_gui %s\n\n",
				camera_id,
				use_extended_baseline ? "on" : "off",
				_rectify_image ? "on" : "off",
				use_configuration_gui ? "on" : "off"
				);
	}
}



int
scale_changed_handler(GtkRange     *range,
		GtkScrollType scroll,
		gdouble       value,
		gpointer      user_data)
{
	if (scroll == GTK_SCROLL_NONE)
		return 0;

	if (value) {}

	if (cam.IsConnected())
	{
		Property prop;

		prop.type = ((PropertyGTK*)user_data)->type;

		if (prop.type == PAN)
		{
			pan_value = gtk_range_get_value(range) == 0.0 ? PAN_VALUE_0 : PAN_VALUE_1;
			set_camera_PAN();
			return 0;
		}

		prop.absValue = gtk_range_get_value(range);
		gtk_toggle_button_set_active((GtkToggleButton*)((PropertyGTK*)user_data)->check_button_auto, false);
		prop.absControl = true;
		prop.autoManualMode = false;
		prop.onOff = true;
		prop.onePush = false;
		prop.valueA = ceil(prop.absValue);
		cam.SetProperty(&prop, true);


	}

	return 0;
}

int
toggled_handler(GtkToggleButton *togglebutton,
		gpointer         user_data)
{

	if (cam.IsConnected())
	{
		Property prop;
		prop.type = ((PropertyGTK*)user_data)->type;
		prop.autoManualMode = gtk_toggle_button_get_active (togglebutton);
		prop.onOff = true;
		prop.absValue = gtk_range_get_value((GtkRange*)((PropertyGTK*)user_data)->scale_button);
		cam.SetProperty(&prop, true);
	}

	return 0;
}

GtkWidget *
create_property(PropertyGTK *property)
{
	GtkWidget *horizontal_container, *frame;

	frame = gtk_frame_new(property->property_name);
	horizontal_container =  gtk_hbox_new(1, 0);


	property->scale_button = gtk_hscale_new_with_range(property->range_min, property->range_max, property->increment);
	gtk_box_pack_start(GTK_BOX(horizontal_container),
			property->scale_button, TRUE, TRUE, 0);

	property->check_button_auto = gtk_check_button_new_with_label("Auto");
	gtk_box_pack_start(GTK_BOX(horizontal_container),
			property->check_button_auto, FALSE, FALSE, 0);

	gtk_widget_set_size_request(horizontal_container, 400, 65);


	gtk_container_add(GTK_CONTAINER(frame), horizontal_container);

	gtk_signal_connect(GTK_OBJECT(property->scale_button), "change-value",
			(GtkSignalFunc) scale_changed_handler, (void*)property);

	gtk_signal_connect(GTK_OBJECT(property->check_button_auto), "toggled",
			(GtkSignalFunc) toggled_handler, (void*)property);


	gtk_widget_show(frame);
	gtk_widget_show(horizontal_container);
	gtk_widget_show(property->check_button_auto);
	gtk_widget_show(property->scale_button);

	return frame;

}



void
create_gui(int argc, char **argv)
{
	GtkWidget *main_window;
	GtkWidget *vertical_container, *frame;

	gtk_init(&argc, &argv);
	main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

	gtk_window_set_title(GTK_WINDOW(main_window), (char*) "Bumblebee Configurations");

	vertical_container = gtk_vbox_new(0, 0);

	property_vector[0].property_name = (char*)"Brightness";
	property_vector[0].type = BRIGHTNESS;
	property_vector[0].range_min = 0.0;
	property_vector[0].range_max = 6.244;
	property_vector[0].increment = 0.001;

	property_vector[1].property_name = (char*)"Exposure";
	property_vector[1].type = AUTO_EXPOSURE;
	property_vector[1].range_min = -7.585;
	property_vector[1].range_max = 2.414;
	property_vector[1].increment = 0.001;

	property_vector[2].property_name = (char*)"Shutter";
	property_vector[2].type = SHUTTER;
	property_vector[2].range_min = 0.001;
	property_vector[2].range_max = 83.284;
	property_vector[2].increment = 0.001;

	property_vector[3].property_name = (char*)"Gain";
	property_vector[3].type = GAIN;
	property_vector[3].range_min = -2.815;
	property_vector[3].range_max = 24;
	property_vector[3].increment = 0.001;

	property_vector[4].property_name = (char*)"PAN";
	property_vector[4].type = PAN;
	property_vector[4].range_min = 0;
	property_vector[4].range_max = 1;
	property_vector[4].increment = 1;


	property_vector_size = 5;


	for (int i = 0; i < property_vector_size; i++)
	{
		frame = create_property(&property_vector[i]);
		gtk_box_pack_start(GTK_BOX(vertical_container),
				frame, 1, 0, 10);
	}

	gtk_container_add(GTK_CONTAINER(main_window), vertical_container);

	gtk_widget_show(vertical_container);

	gtk_widget_show(main_window);
}

int main(int argc, char **argv)
{

	Error error;
	BusManager busMgr;
	unsigned int numCameras;

	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	initialize_bumblebee_message(1280, 960, 3, 1);

	get_triclops_context_from_camera( &triclops );

	error = busMgr.GetNumOfCameras(&numCameras);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}

	printf( "Number of cameras detected: %u\n", numCameras );

	if (use_configuration_gui)
		create_gui(argc, argv);

#pragma omp parallel sections if(use_configuration_gui)
	{
#pragma omp section
		if (use_configuration_gui)
			gtk_main();

#pragma omp section
		for (unsigned int i = 0; i < numCameras; i++)
		{
			PGRGuid guid;
			error = busMgr.GetCameraFromIndex(i, &guid);

			handle_error(error);

			RunSingleCamera( guid );
		}
	}

	return 0;
}
