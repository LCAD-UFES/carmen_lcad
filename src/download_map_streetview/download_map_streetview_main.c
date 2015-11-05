#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/carmen.h>
#include <carmen/carmen_gps_wrapper.h>
#include <carmen/download_map_streetview_interface.h>
#include <carmen/grid_mapping.h>

#include "download_map_streetview.h"

carmen_vector_3D_t pose_last_map_downloaded;
carmen_vector_3D_t position_before;
//static carmen_download_map_streetview_message download_map_streetview_message;
static int carmen_download_map_streetview_from_internet = 0;
static int posVetorCoordenadas = 0;
static double vetorCoordenadas[10000];
static int checador = 0;
static int num_image = 1;
FILE *file;

typedef struct info_streetview_image{

	double pos_x[10000];
	double pos_y[10000];
	double pos_z[10000];

	double yaw[10000];
	double pitch[10000];
	double roll[10000];

	int pos;

}info_streetview_image;

info_streetview_image sv_images;




//
// O mapa do google no zoom 19 possui 0.3 metros por pixel.
// Sendo assim, o mapa com dimensoes de 512x512 pixels, tera 153.6x153.6 metros.
//
// static double distance_to_download_new_map = 10.0;
// static double distance_to_download_new_map = 76.8; // 76.8 = (153.6 / 2.0)

static void
carmen_publish_download_map_streetview_message (carmen_download_map_streetview_message *download_map_streetview_message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData (CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_NAME, download_map_streetview_message);
	carmen_test_ipc_exit (err, "Could not publish", CARMEN_DOWNLOAD_MAP_STREETVIEW_MESSAGE_FMT);
}

//ok
static IplImage*
download_map_streetview_from_google_maps (double latitude, double longitude)
{
	IplImage *img;

	int i;
	int map_width = 640;
	int map_height = 480;


	char *key = "AIzaSyD4Ektt6lJgUjQOAXDc0M2tCByQwbrBENY";	// usar sua key da google API.
	char filename[128];

	for(i = 0; i < 8; i++)
	{
		int angle = i * 45;
		sprintf(filename, "%03d_%d.png", num_image, angle/45);
		//sprintf(filename, "streetview__image_%f,%f_%d.png", latitude, longitude, angle);
		get_image_from_gps (latitude, longitude, map_width, map_height, filename, key, angle);
		img = cvLoadImage (filename, CV_LOAD_IMAGE_COLOR);
	}

	num_image++;
	return img;
}



void
carmen_grid_mapping_get_map_origin2(carmen_point_t *global_pose, int *x_origin, int* y_origin)
{
	*x_origin = (floor(global_pose->x / 50.0) - 1) * 50;
	*y_origin = (floor(global_pose->y / 50.0) - 1) * 50;
}

//ok
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

//ok
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


//ok
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

//ok
static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_vector_3D_t position;
//	msg->pose.orientation.
	position.x = msg->globalpos.x;
	position.y = msg->globalpos.y;
	position.z = 0;

	static int image_number = 1;
	if(checa_distancia_percorrida_maior_dez(position_before, position) == 1)
	{
		vetorCoordenadas[posVetorCoordenadas] = position.x;
		vetorCoordenadas[posVetorCoordenadas+1] = position.y;
		//printf(" %f ==== %f\n", vetorCoordenadas[posVetorCoordenadas], vetorCoordenadas[posVetorCoordenadas+1]);
		position_before.x = position.x;
		position_before.y = position.y;
		posVetorCoordenadas += 2;

		int angle;
		for(angle = 0; angle < 8; angle++)
			fprintf(file,"%03d_%d %f %f %f %f %f\n",image_number, angle, position.x, position.y, msg->pose.orientation.pitch, msg->pose.orientation.roll, msg->pose.orientation.yaw);


		image_number++;
	}

}

//ok
static void
save_maps_disk()
{
	double latitude, longitude, elevation, map_center_x, map_center_y;
	int x_origin, y_origin, i;
	IplImage *map_img = NULL;

	for(i = 0; i < posVetorCoordenadas; i += 2)
	{
		double robot_real_pos_x = vetorCoordenadas[i];
		double robot_real_pos_y = vetorCoordenadas[i+1];

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

		printf("real pos : %f, %f >><< origin : %d, %d <<>> map center : %f, %f <><> gps pos : %f, %f <><><><> lat-lon : %f, %f \n", robot_real_pos_x, robot_real_pos_y, x_origin, y_origin, map_center_x, map_center_y, gps_position_x, gps_position_y, latitude, longitude);

		map_img = download_map_streetview_from_google_maps (latitude, longitude);
		carmen_ipc_sleep(2.0);
		save_map_image (map_img, latitude, longitude);
	}
	printf("Fim do download !\n");
	fclose(file);
	exit(1);

}

//ok
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
	file = fopen("data.txt","w");

	carmen_download_map_streetview_define_messages();
	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_addPeriodicTimer(35.0, check_vector, NULL);

	carmen_ipc_dispatch();
	return (0);
}
