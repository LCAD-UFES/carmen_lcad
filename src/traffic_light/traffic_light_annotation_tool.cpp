/* TODO:
  * - Codigo para visualizar imagens a partir do momento acima (100m antes da annotação do semaforo)
 * 		-Receber annotacoes
 * 		-Receber globalpos
 * 		-Receber numero da mesagem do log
 * 		-Abrir visualizador
 * 		-Tecla para marcar o trecho (red-green-yellow-off)
 * 		-Salvar:
 *	  	  GT: image_0000_TimeInicio_TimeFim_pose_poseAno_Dist_message.png estado
 *             Predito: TimeInicio_TimeFim estado
 *            IOU = TimeInicioGT-TimeInicioP TimeFimGT-TimeFimP Estado
 *
 *		Para garantir Salvar 2 arquivos, 1 com a configuração acima e outro com todas as imagens dentro do periodo marcado.
image_0000_TimeStamp_pose_poseAno_Dist_message.png estado
 *
 *
 * */

/*********************************************************
---   Tool for traffic_light decision time annotation  ---
* This module will be used to generate the GT for the behavior selector decision process in traffic light

**********************************************************/


#include <stdio.h>
#include <iostream>
#include <string>

#include <carmen/carmen.h>
#include <carmen/readlog.h>
#include <carmen/rddf_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/bumblebee_basic_interface.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH 640
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3

int camera_side = 1; //0-left 1-right
char dataset_images_path[1028];
int save_images;


using namespace std;

string window_name;
string database_path;


void
save_image_and_detections()
{
//	static int counter = 0;
//
//	if (traffic_light_message.num_traffic_lights == 0)
//	{
//		const char *filename = "undetected.txt";
//		static char image_name[1024];
//		static char file_name_with_dir[2048];
//
//		sprintf(image_name, "%s/img/image_%04d_%lf_%lf_%lf.png",
//				database_path, counter, stereo_image->timestamp,
//				nearest_traffic_light_pose.x, nearest_traffic_light_pose.y);
//
//		sprintf(file_name_with_dir, "%s/%s", database_path, filename);
//
//		FILE *database_file = fopen(file_name_with_dir, "a");
//
//		if (database_file == NULL)
//			exit(printf("Error: Unable to open the output file '%s'\n", file_name_with_dir));
//
//		Mat *m = bumblebee_to_opencv(stereo_image);
//		imwrite(image_name, *m);
//
//		fprintf(database_file, "%s\n", image_name);
//
//		fclose(database_file);
//	}
}


static void
shutdown_traffic_light_tool(int x)
{
    if (x == SIGINT)
    {
        carmen_verbose("Disconnecting Traffic Light Tool Service.\n");
        carmen_ipc_disconnect();

        exit(1);
    }
}


void
process_event(char c)
{
	if (c == 'c' || c == 'q')
		shutdown_traffic_light_tool(SIGINT);

}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	double window_scale;
	int window_height;
	window_name = "Metric Annotation Tool";
	static int first = 1;
	static cv::Mat *resized_image = NULL;
	cv::Mat src_image(cv::Size(stereo_image->width, stereo_image->height), CV_8UC3);
	if (camera_side == 0)
		src_image.data = stereo_image->raw_left;
	else
		src_image.data = stereo_image->raw_right;

	window_scale = (double) BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH / (double) stereo_image->width;
	window_height = (int) (stereo_image->height * window_scale);

	//printf("window_scale: %lf window_height: %d width: %d msg: %d %d\n", window_scale, window_height, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH,
	//msg->height, msg->width);

	if (first)
	{
		resized_image= new cv::Mat(cv::Size(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, window_height), CV_8UC3);
		first = 0;
	}
	//resizing the image
	resize(src_image, *resized_image, resized_image->size());

	cv::cvtColor(*resized_image, *resized_image, CV_RGB2BGR);
	cv::imshow(window_name, *resized_image);

	char c = cv::waitKey(2);
	process_event(c);

}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *global_pos)
{

}


void
carmen_rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{

}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static int
read_parameters(int argc, char **argv)
{
	return 0;
}


void
subscribe_messages(int camera)
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) carmen_rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);

	if (argc < 3)
		carmen_die("%s: Wrong number of parameters. Traffic Light annotation tool requires 3 parameters and received %d parameter(s). \n If save images is active create a directory for images.\n"
				"Usage:\n %s <camera_number> <camera_side> <[0-1]save_images> <dataset_path>\n", argv[0], argc - 1, argv[0]);

	int camera = atoi(argv[1]);
	camera_side = atoi(argv[2]);
	database_path = "";
    database_path = argv[3];
    sprintf(dataset_images_path,"%s/images/", database_path);
    printf("Database: %s  Saving Images in: %s\n", database_path, dataset_images_path);

    carmen_param_check_version(argv[0]);

    signal(SIGINT, shutdown_traffic_light_tool);

    read_parameters(argc, argv);

    subscribe_messages(camera);

    carmen_ipc_dispatch();

    return (0);
}

