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
#include <vector>

#include <carmen/carmen.h>
#include <carmen/readlog.h>
#include <carmen/rddf_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/playback_interface.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH 640
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3
#define COLOR_RED 0
#define COLOR_GREEN 1
#define COLOR_YELLOW 2
#define COLOR_OFF 3
#define MAX_TRAFFIC_LIGHT_DISTANCE	110.0

typedef struct
{
	int img_number;
    double start_time;
    double end_time;
    carmen_point_t pose_robot;
    carmen_point_t pose_annotation;
    double distance;
    int color;
} gt_annotation;



int camera_side = 1; //0-left 1-right
char dataset_images_path[1028];
int save_images;
double begin_timestamp;
double end_timestamp;
int last_state;
using namespace std;

vector <carmen_point_t> annotation_list;
carmen_point_t nearest_traffic_light_pose;
gt_annotation currently_annotation;
vector<gt_annotation> set_of_annotations_by_state;
double last_timestamp = 0.0;
string window_name;
static char *database_path = NULL; ;
carmen_localize_ackerman_globalpos_message car_pose;
vector <carmen_playback_info_message> playback_info_list;
int last_color = -1;
int img_number = 0;


double
compute_distance_to_the_traffic_light()
{
    double nearest_traffic_light_distance = DBL_MAX;

    for (unsigned int i = 0; i < annotation_list.size(); i++)
    {
    		double distance = sqrt(pow(car_pose.globalpos.x - annotation_list[i].x, 2) +
							pow(car_pose.globalpos.y - annotation_list[i].y, 2));
    		if (distance < nearest_traffic_light_distance)
    		{
    			bool orientation_ok = fabs(carmen_radians_to_degrees(car_pose.globalpos.theta - annotation_list[i].theta)) < 10.0 ? 1 : 0;
    			bool behind = fabs(carmen_normalize_theta((atan2(car_pose.globalpos.y - annotation_list[i].y,
															car_pose.globalpos.x - annotation_list[i].x) - M_PI -
													 	 	car_pose.globalpos.theta))) > M_PI_2;

				if (distance <= MAX_TRAFFIC_LIGHT_DISTANCE && orientation_ok && behind == false)
				{
	    			nearest_traffic_light_distance = distance;
	    			nearest_traffic_light_pose = annotation_list[i];
				}
    		}
    }

    return (nearest_traffic_light_distance);
}


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
set_end_timestamp_and_save_lines()
{
	char filename[2048];
	sprintf(filename, "%s/gound_truth_images.txt", database_path);
	FILE *arquivo = fopen(filename, "a");
	for (unsigned int i = 0; i < set_of_annotations_by_state.size(); i++)
	{
		set_of_annotations_by_state.at(i).end_time = last_timestamp;

//		image_0000_TimeInicio_TimeFim_Dist.png estado
//		printf("image_%.4d_%lf_%lf_%lf %d \n",
//				set_of_annotations_by_state.at(i).img_number,
//				set_of_annotations_by_state.at(i).start_time,
//				set_of_annotations_by_state.at(i).end_time,
//				set_of_annotations_by_state.at(i).distance,
//				set_of_annotations_by_state.at(i).color);
		//TODO:
		fprintf(arquivo, "image_%.4d_%lf_%lf_%lf %d\n",
				set_of_annotations_by_state.at(i).img_number,
				set_of_annotations_by_state.at(i).start_time,
				set_of_annotations_by_state.at(i).end_time,
				set_of_annotations_by_state.at(i).distance,
				set_of_annotations_by_state.at(i).color);
	}
	set_of_annotations_by_state.clear();
	fclose(arquivo);
}


void
update_state(double timestamp_image, double distance, cv::Mat src_img, int state)
{
	char image_name[2048];
	carmen_point_t p;
	p.theta = 0.0;
	p.x = 0.0;
	p.y = 0.0;

	gt_annotation currently_annotation;
	currently_annotation.start_time = timestamp_image;
	currently_annotation.color = state;
	currently_annotation.distance = distance;
	currently_annotation.img_number = img_number;
	currently_annotation.pose_annotation = p;
	currently_annotation.pose_robot = car_pose.globalpos;
	set_of_annotations_by_state.push_back(currently_annotation);
	img_number++;

	if (save_images)
	{
		sprintf(image_name, "%s/images/image_%.4d_%lf_%lf_%d.png", database_path,
				currently_annotation.img_number,
				currently_annotation.start_time,
				currently_annotation.distance,
				currently_annotation.color);
		cv::imwrite(image_name, src_img);
	}
}


void
process_event(char key_pressed, double timestamp_image, cv::Mat src_img, double distance)
{
	static int first_time = 1;
	static char last_key_pressed = key_pressed;

	switch (key_pressed)
	{
	case 'q':
		if (last_color != -1)
			set_end_timestamp_and_save_lines();
		shutdown_traffic_light_tool(SIGINT);
		break;
	case 'r':
		if (!first_time && (strcmp(&last_key_pressed, &key_pressed))!=0)
		{
			set_end_timestamp_and_save_lines();
			update_state(timestamp_image, distance, src_img, COLOR_RED);
			last_key_pressed = key_pressed;
			last_color = COLOR_RED;
		}
		else
		{
			first_time = 0;
			update_state(timestamp_image, distance, src_img, COLOR_RED);
			last_color = COLOR_RED;
		}
		break;
	case 'g':
		if (!first_time && (strcmp(&last_key_pressed, &key_pressed)!=0))
		{
			set_end_timestamp_and_save_lines();
			update_state(timestamp_image, distance, src_img, COLOR_GREEN);
			last_key_pressed = key_pressed;
			last_color = COLOR_GREEN;
		}
		else
		{
			first_time = 0;
			update_state(timestamp_image, distance, src_img, COLOR_GREEN);
			last_color = COLOR_GREEN;
		}
		break;
	case 'y':
		if (!first_time && (strcmp(&last_key_pressed, &key_pressed)!=0))
		{
			set_end_timestamp_and_save_lines();
			update_state(timestamp_image, distance, src_img, COLOR_YELLOW);
			last_key_pressed = key_pressed;
			last_color = COLOR_YELLOW;
		}
		else
		{
			first_time = 0;
			update_state(timestamp_image, distance, src_img, COLOR_YELLOW);
			last_color = COLOR_YELLOW;
		}
		break;
	case 'o':
			if (!first_time && (strcmp(&last_key_pressed, &key_pressed)!=0))
			{
				set_end_timestamp_and_save_lines();
				update_state(timestamp_image, distance, src_img, COLOR_OFF);
				last_key_pressed = key_pressed;
				last_color = COLOR_OFF;
			}
			else
			{
				first_time = 0;
				update_state(timestamp_image, distance, src_img, COLOR_OFF);
				last_color = COLOR_OFF;
			}
			break;
	case 'e':
		set_end_timestamp_and_save_lines();
		last_key_pressed = key_pressed;
		last_color = -1;
		break;

	default:
		//printf("Comando desconhecido\n");
		if (last_color != -1)
			update_state(timestamp_image, distance, src_img, last_color);
		break;
	}
//	printf("Dist: %lf\n", distance);
	last_timestamp = timestamp_image;


}


void
write_state_and_distance_on_image(cv::Mat *resized_image, int color, double distance)
{
	char text[128];
	if (distance <= MAX_TRAFFIC_LIGHT_DISTANCE)
		sprintf(text,"Distance: %.2lf meters",distance);
	else
		sprintf(text,"Distance greater than: %.2lf meters",MAX_TRAFFIC_LIGHT_DISTANCE);

	if (resized_image->cols > 900)
		cv::putText(*resized_image, text, cv::Point(20, 880), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255,0,0), 2, 5);
	else
		cv::putText(*resized_image, text, cv::Point(20, 380), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 5);

	if (color == COLOR_RED)
		cv::circle(*resized_image, cv::Point(50, 130), 30, cv::Scalar(0, 0, 255), -1, 8);
	else if (color == COLOR_GREEN)
		cv::circle(*resized_image, cv::Point(50, 130), 30, cv::Scalar(0, 255, 0), -1, 8);
	else if (color == COLOR_YELLOW)
		cv::circle(*resized_image, cv::Point(50, 130), 30, cv::Scalar(0, 255, 255), -1, 8);
	else if (color == COLOR_OFF)
		cv::circle(*resized_image, cv::Point(50, 130), 30, cv::Scalar(0, 0, 0), -1, 8);
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
	cv::cvtColor(src_image, src_image, CV_RGB2BGR);
	if (first)
	{
		last_timestamp = stereo_image->timestamp;
		resized_image= new cv::Mat(cv::Size(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, window_height), CV_8UC3);
		first = 0;
	}
	//resizing the image
	resize(src_image, *resized_image, resized_image->size());

	double distance = compute_distance_to_the_traffic_light();
	write_state_and_distance_on_image(resized_image, last_color, distance);

	cv::imshow(window_name, *resized_image);

	char c = cv::waitKey(2);
	process_event(c, stereo_image->timestamp, src_image, distance);

}


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg_global_pos)
{
	car_pose = *msg_global_pos;

}


//void
//carmen_playback_info_message_handler(carmen_playback_info_message *msg_playback)
//{
//	carmen_playback_info_message info;
//	info = *msg_playback;
//	playback_info_list.push_back(info);
//
//}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


/*static int
read_parameters()
{
	return 0;
}*/


void
subscribe_messages(int camera)
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler,
    		CARMEN_SUBSCRIBE_LATEST);
//    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) carmen_rddf_annotation_message_handler,
//    		CARMEN_SUBSCRIBE_LATEST);
//    carmen_subscribe_playback_info_message (NULL, (carmen_handler_t) carmen_playback_info_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
load_traffic_light_annotation_file(char* path)
{
	carmen_point_t pose;
	FILE *annotation = fopen(path,"r");
	while ((fscanf(annotation, "%lf %lf %lf", &pose.x,&pose.y, &pose.theta) != EOF))
	{
		annotation_list.push_back(pose);
	}

	fclose(annotation);
}


///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	if (argc < 3)
		carmen_die("%s: Wrong number of parameters. Traffic Light annotation tool requires 3 parameters and received %d parameter(s). \n If save images is active create a directory for images.\n"
				"Usage:\n %s <camera_number> <[0-l 1-r]camera_side> <[0-1]save_images> <dataset_path> <annotation_file>\n", argv[0], argc - 1, argv[0]);

    carmen_param_check_version(argv[0]);

	int camera = atoi(argv[1]);
	camera_side = atoi(argv[2]);
	save_images = atoi(argv[3]);
    database_path = argv[4];
    sprintf(dataset_images_path,"%s/images/", database_path);
    printf("Database: %s  Saving Images in: %s\n", database_path, dataset_images_path);


    signal(SIGINT, shutdown_traffic_light_tool);

//    read_parameters(argc, argv);

    load_traffic_light_annotation_file(argv[5]);
    subscribe_messages(camera);

    carmen_ipc_dispatch();

    return (0);
}
