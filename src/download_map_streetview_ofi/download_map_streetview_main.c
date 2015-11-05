#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/carmen.h>
#include <carmen/carmen_gps_wrapper.h>
#include <carmen/download_map_streetview_interface.h>
#include <carmen/grid_mapping.h>

#include "download_map_streetview.h"
#include "math.h"

carmen_vector_3D_t pose_last_map_downloaded;
carmen_vector_3D_t position_before;
//static carmen_download_map_streetview_message download_map_streetview_message;


static int
	carmen_download_map_streetview_from_internet = 0,
	posVetorCoordenadas = 0,
	posVetYaw = 0,
	checador = 0,
	num_image = 1;

static double 
	vetorCoordenadas[10000],
	vetorYaw[10000];

FILE
	*data_output,
	*url_output;

static void
carmen_publish_download_map_streetview_message (carmen_download_map_streetview_message *download_map_streetview_message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData (CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_NAME, download_map_streetview_message);
	carmen_test_ipc_exit (err, "Could not publish", CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_FMT);
}



static int
get_angle_unnormalized (int angle)
{
	if(0 > angle && angle >= -180)
	{
		angle = angle + 360;
	}
	angle = angle%360;
	return angle;
}


static int
return_front_side_angle (int angle)
{
	int
		value_to_add = 0;

	if(angle < 360 && angle > 270)
	{
		if(fabs(angle - 270) < fabs(angle - 360))
		{
			value_to_add = 2*fabs(angle-270);
			angle = angle - value_to_add;
		}
		else
		{
			value_to_add = 2*fabs(angle-360);
			angle = angle + value_to_add;
		}

	}
	else if(angle < 270 && angle > 180)
	{
		if(fabs(angle - 180) < fabs(angle - 270))
		{
			value_to_add = 2*fabs(angle-180);
			angle = angle - value_to_add;

		}
		else
		{
			value_to_add = 2*fabs(angle-270);
			angle = angle + value_to_add;

		}

	}
	else if(angle < 180 && angle > 90)
	{
		if(fabs(angle - 90) < fabs(angle - 180))
		{
			value_to_add = 2*fabs(angle-90);
			angle = angle - value_to_add;

		}
		else
		{
			value_to_add = 2*fabs(angle-180);
			angle = angle + value_to_add;

		}


	}
	else if(angle < 90 && angle > 0){
		if(fabs(angle - 0) < fabs(angle - 90))
		{
			value_to_add = 2*fabs(angle);
			angle = angle - value_to_add;

		}
		else
		{
			value_to_add = 2*fabs(angle-90);
			angle = angle + value_to_add;

		}

	}


	return angle;
}

//ok
static IplImage*
download_map_streetview_from_google_maps (double latitude, double longitude, double yaw)
{
	IplImage 
		*img;

	int 
		map_width = 640,
		map_height = 480;

	int
		angle = carmen_radians_to_degrees(yaw); // convertido de radiano para grau.

	angle = get_angle_unnormalized(angle);
	angle = return_front_side_angle(angle);

	char 
		filename[128];
	

	sprintf(filename, "%03d.png", num_image);
	get_image_from_gps (url_output, latitude, longitude, map_width, map_height, filename, angle);
	img = cvLoadImage (filename, CV_LOAD_IMAGE_COLOR);

	num_image++;
	return img;
}

void
carmen_grid_mapping_get_map_origin2(carmen_point_t *global_pose, int *x_origin, int* y_origin)
{
	*x_origin = (floor(global_pose->x / 50.0) - 1) * 50;
	*y_origin = (floor(global_pose->y / 50.0) - 1) * 50;
}

void
get_map_origin (double x, double y, int *x_origin, int *y_origin)
{
	carmen_point_t pose;

	pose.x = x;
	pose.y = y;

	carmen_grid_mapping_get_map_origin2 (&pose, x_origin, y_origin);
}

void
format_map_path (double x_origin, double y_origin, char* map_filename)
{
	char *carmen_home = getenv("CARMEN_HOME");

	if (carmen_home == NULL)
		exit(printf("Could not get environment variable $CARMEN_HOME in create_texture()\n"));

	sprintf(map_filename,
			"%s/data/google_maps_streetview/m%f,%f.bmp",
			carmen_home,
			x_origin,
			y_origin);
}

void
save_map_image (IplImage *img, double origin_x, double origin_y)
{
	char map_filename [1024];

	format_map_path(origin_x, origin_y, map_filename);
	cvSaveImage(map_filename, img, NULL);
}

IplImage *
find_map_from_data (double origin_x, double origin_y)
{
	IplImage *map;
	char map_filename [1024];

	format_map_path(origin_x, origin_y, map_filename);
	map = cvLoadImage (map_filename, CV_LOAD_IMAGE_ANYCOLOR);

	return map;
}

int
checa_distancia_percorrida_maior_dez(carmen_vector_3D_t ant, carmen_vector_3D_t atual)
{
	carmen_world_point_t p1, p2;


	p1.pose.x = ant.x;
	p1.pose.y = ant.y;

	p2.pose.x = atual.x;
	p2.pose.y = atual.y;

	if(carmen_distance_world(&p1, &p2) > 10.0)
		return 1;

	return 0;

}

static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_vector_3D_t position;
	position.x = msg->globalpos.x;
	position.y = msg->globalpos.y;
	position.z = 0;
	static int image_number = 1;
	static int first_image = 0;

	if(checa_distancia_percorrida_maior_dez(position_before, position) == 1)
	{
		if(first_image == 1){
			vetorCoordenadas[posVetorCoordenadas] = position.x;
			vetorCoordenadas[posVetorCoordenadas+1] = position.y;
			position_before.x = position.x;
			position_before.y = position.y;
			posVetorCoordenadas += 2;
			fprintf(data_output,"%03d %f %f %f %f %f\n", image_number, position.x, position.y, msg->pose.orientation.pitch, msg->pose.orientation.roll, msg->pose.orientation.yaw);
			vetorYaw[posVetYaw] = msg->pose.orientation.yaw;

			printf("%f\n", vetorYaw[posVetYaw]);

			image_number++;
			posVetYaw++;
		}else{
			first_image = 1;
		}

	}

}

static void
save_maps_disk()
{
	double
		latitude,
		longitude,
		elevation,
		map_center_x,
		map_center_y;

	int
		x_origin,
		y_origin,
		i,
		yaw_pos;

	IplImage
		*map_img = NULL;

	for(i = 0, yaw_pos = 0; i < posVetorCoordenadas; i += 2,yaw_pos++)
	{
		double robot_real_pos_x = vetorCoordenadas[i],
			robot_real_pos_y = vetorCoordenadas[i+1];

		get_map_origin (robot_real_pos_x, robot_real_pos_y, &x_origin, &y_origin);

		// o mapa do google tem 153.6 (512 pixels * 0.3 metros por pixel) metros de lado
		// como a origem calculada eh no canto inferior esquerdo do mapa, preciso somar meio lado
		// para calcular as coordenadas do meio do mapa

		map_center_x = x_origin + 76.8;
		map_center_y = y_origin + 76.8;

		// os valores sao colocados invertidos por causa do sistema de coordenadas do gps

		double gps_position_x = -robot_real_pos_y;//-map_center_y;
		double gps_position_y = robot_real_pos_x;//map_center_x;

		carmen_Utm_Gdc3(gps_position_x, gps_position_y, 0, 24, 0, &latitude, &longitude, &elevation);

		//printf("real pos : %f, %f >><< origin : %d, %d <<>> map center : %f, %f <><> gps pos : %f, %f <><><><> lat-lon : %f, %f \n", robot_real_pos_x, robot_real_pos_y, x_origin, y_origin, map_center_x, map_center_y, gps_position_x, gps_position_y, latitude, longitude);

		map_img = download_map_streetview_from_google_maps (latitude, longitude, vetorYaw[yaw_pos]);
		carmen_ipc_sleep(2.0);
		save_map_image (map_img, latitude, longitude);

	}

	printf("Fim do download !\n");
	fclose(data_output);
	fclose(url_output);
	exit(1);
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("download_map_streetview: disconnected.\n");

		exit(0);
	}
}

//ok
static void
carmen_get_download_map_streetview_parameters (int argc, char** argv)
{
	carmen_param_t param_list[] = {
			{(char*) "download_map", (char*) "from_internet", CARMEN_PARAM_ONOFF, &carmen_download_map_streetview_from_internet, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}

//ok
void
check_vector(void *clientData,
				   unsigned long currentTime,
				   unsigned long scheduledTime)
{



	if(checador == 0)
	{
		checador = posVetorCoordenadas;
	}
	else if(checador == posVetorCoordenadas)
	{
		printf("\nTerminou o log, iniciando download dos mapas.\n");
		printf("\nTam vetor coordenadas: %d .\n", posVetorCoordenadas);
		//terminou o log.

		save_maps_disk();


	}
	else
	{
		checador = posVetorCoordenadas;

	}


}


int main(int argc, char **argv)
{
	srand(time(NULL));

	carmen_ipc_initialize(argc, argv);
	carmen_get_download_map_streetview_parameters (argc, argv);

	signal(SIGINT, shutdown_module);

	pose_last_map_downloaded.x = 9999999;
	pose_last_map_downloaded.y = 9999999;
	pose_last_map_downloaded.z = 9999999;

	position_before.x = 1.0;
	position_before.y = 1.0;
	position_before.z = 1.0;

	//sv_images.pos = 0;
	data_output = fopen("data.txt","w");
	url_output = fopen("url.txt","w");

	carmen_download_map_streetview_define_messages();
	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_addPeriodicTimer(35.0, check_vector, NULL);

	carmen_ipc_dispatch();
	return (0);
}
