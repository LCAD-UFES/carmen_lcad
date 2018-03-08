#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <libgen.h>
#include <unistd.h> //função sleep
#include "rddf_graph_utils.h"

using namespace std;

string g_window_name1 = "road map";
string g_window_name2 = "road center vertical";
string g_window_name3 = "road center horizontal";
string g_window_name4 = "blended images";

#define MAX_PROB (pow(2.0, 16) - 1.0)

bool g_ipc_required = false;
int g_img_channels = 3;
int g_class_bits = 0;
char *g_remission_map_dir = NULL;
int g_road_map_index = 1;

static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "[-c <img_channels>] [-b <class_bits>] [-r <remission_map_dir>] <road_map_1>.map [...]";
	for(int i = 1; i < argc; i++)
	{
		if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
			exit(printf("Usage:\n%s %s\n", argv[0], usage));
		else if(strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--img_channels") == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_img_channels = atoi(argv[i + 1]);
				i++, g_road_map_index++;
			}
			else
				printf("Image channels expected following -c option.\nUsage:\n%s %s\n", argv[0], usage);
		}
		else if(strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--class_bits") == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_class_bits = atoi(argv[i + 1]);
				i++, g_road_map_index++;
			}
			else
				printf("Class bits expected following -b option.\nUsage:\n%s %s\n", argv[0], usage);
		}
		else if(strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--remission_maps") == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_remission_map_dir = argv[i + 1];
				int n = strlen(g_remission_map_dir) - 1;
				if (n >= 0 && g_remission_map_dir[n] == '/')
					g_remission_map_dir[n] = 0;
				i++, g_road_map_index++;
			}
			else
				printf("Remission map directory expected following -r option.\nUsage:\n%s %s\n", argv[0], usage);
		}
		else
			break;
	}
	if (g_road_map_index == argc)
		exit(printf("At least one road map file expected\nUsage:\n%s %s\n", argv[0], usage));
	printf("Image channels set to %d.\n", g_img_channels);
	if (g_img_channels == 1)
		printf("Class bits set to %d.\n", g_class_bits);
	if (g_remission_map_dir)
		printf("Remission map directory: %s.\n", g_remission_map_dir);
}


static void
define_messages()
{
}

static void
register_handlers()
{
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (g_ipc_required)
			carmen_ipc_disconnect();
		exit(printf("road_mapper_display_map: disconnected.\n"));
	}
}

void
blend_images_vertical_and_horizontal(carmen_map_p road_map, cv::Mat *image_vertical, cv::Mat *image_horizontal)
{
	//source: https://docs.opencv.org/2.4/doc/tutorials/core/adding_images/adding_images.html

	double alpha = 0.5; double beta;
	cv::Mat blended_image;
	beta = 1-alpha;
	cv::addWeighted( *image_vertical, alpha, *image_horizontal, beta, 0.0, blended_image);
	cv::namedWindow(g_window_name4, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name4, (78*3) + 2*road_map->config.x_size, 50+road_map->config.y_size);
	cv::imshow(g_window_name4, blended_image);
	//cv::waitKey(0);
	while((cv::waitKey() & 0xff) != 27);


}

void
road_mapper_display_road_map(carmen_map_p road_map, int img_channels, int img_class_bits)
{
	road_prob *cell_prob;
	cv::namedWindow(g_window_name1, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name1, 78 + road_map->config.x_size, 10);
	cv::namedWindow(g_window_name2, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name2, 78 + 2*road_map->config.x_size, 10);
	cv::namedWindow(g_window_name3, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name3, 78 + 3*road_map->config.x_size, 10);

	cv::Mat image1;
	cv::Mat imageCenterVertical; //imagem que guarda apenas o ponto de centro de pista verticalmente
	cv::Mat imageCenterHorizontal; //imagem que guarda apenas o ponto de centro de pista verticalmente
	if (img_channels == 1)
	{
		image1 = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC1);
		road_map_to_image_black_and_white(road_map, &image1, img_class_bits);
	}
	else
	{
		image1 = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC3, cv::Scalar::all(0));
		imageCenterVertical = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC3, cv::Scalar::all(255));
		imageCenterHorizontal = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC3, cv::Scalar::all(255));
		road_map_to_image(road_map, &image1);
	}
	/*cv::imshow(g_window_name1, image1);
	if (g_remission_map_dir == NULL)
	{
		cout << "Press \"Esc\" key to continue...\n\n";
		while((cv::waitKey() & 0xff) != 27);
	}OLD WAY!*/
	printf("IMSIZE: %d X %d: \n",road_map->config.x_size, road_map->config.y_size);
	cv::Point p;
	cv::Point pAnt;
	cv::Point pPos;
	cv::Mat imgPaint = image1;
	cv::Vec3b pixelChannelAnt; //pegar cada canal de cor em separado
	cv::Vec3b pixelChannel; //pegar cada canal de cor em separado
	cv::Vec3b pixelChannelPos; //pegar cada canal de cor em separado

	cv::Point pAntV;
	cv::Point pPosV;
	cv::Vec3b pixelChannelAntV; //pegar cada canal de cor em separado
	cv::Vec3b pixelChannelPosV; //pegar cada canal de cor em separado
	int thickness = -1;
	int lineType = 8;
	for (int y = 1; y < road_map->config.y_size-1; y+=1)
		{
			for (int x = 1; x < road_map->config.x_size-1; x+=1)
			{
				cell_prob = road_mapper_double_to_prob(&road_map->map[x][road_map->config.y_size - 1 - y]);
				//printf("%d X %d %hu\n",x,y,cell_prob->lane_center);
				imgPaint = image1.clone();
				p.x = x;
				p.y = y;
				pAnt.x = x-1;
				pAnt.y = y;
				pPos.x = x+1;
				pPos.y = y;
				pixelChannelAnt = image1.at<cv::Vec3b>(pAnt);
				pixelChannel = image1.at<cv::Vec3b>(p);
				pixelChannelPos = image1.at<cv::Vec3b>(pPos);

				pAntV.x = x;
				pAntV.y = y-1;
				pPosV.x = x;
				pPosV.y = y+1;
				pixelChannelAntV = image1.at<cv::Vec3b>(pAntV);
				pixelChannelPosV = image1.at<cv::Vec3b>(pPosV);

				if(pixelChannel.val[0]==255 && pixelChannel.val[1]==255 && pixelChannel.val[2]==255)
					continue;
				else{
					cv::circle(imgPaint, p, 1,cv::Scalar( 0, 0, 0 ),thickness,lineType);
					if(((pixelChannel.val[1]>pixelChannelAnt.val[1])&&(pixelChannel.val[1]>pixelChannelPos.val[1]))||(pixelChannel.val[0]==0 && pixelChannel.val[1]==255 && pixelChannel.val[2]==0)){
						//printf("Pixel Color at %dX%d: b: %d g: %d r: %d %hu CENTER!!!\n",x,y,pixelChannel.val[0],pixelChannel.val[1],pixelChannel.val[2], cell_prob->lane_center);
						cv::circle(imageCenterVertical, p, 1,cv::Scalar( 255, 0, 0 ),thickness,lineType);
					}
					//else{
						//printf("Pixel Color at %dX%d: b: %d g: %d r: %d %hu\n",x,y,pixelChannel.val[0],pixelChannel.val[1],pixelChannel.val[2], cell_prob->lane_center);
					//}

					if(((pixelChannel.val[1]>pixelChannelAntV.val[1])&&(pixelChannel.val[1]>pixelChannelPosV.val[1]))||(pixelChannel.val[0]==0 && pixelChannel.val[1]==255 && pixelChannel.val[2]==0)){
						//printf("Pixel Color at %dX%d: b: %d g: %d r: %d  CENTER!!!\n",x,y,pixelChannel.val[0],pixelChannel.val[1],pixelChannel.val[2]);
						cv::circle(imageCenterHorizontal, p, 1,cv::Scalar( 0, 0, 255 ),thickness,lineType);
					}

					

				}


				cv::imshow(g_window_name1, imgPaint);
				cv::imshow(g_window_name2, imageCenterVertical);
				cv::imshow(g_window_name3, imageCenterHorizontal);
				//printf("Pixel Green Color at %dX%d: %d\n",x,y,image1.at<cv::Vec3b>(p)[1]);
				//printf("Pixel Color at %dX%d: b: %d g: %d r: %d\n",x,y,image1.at<cv::Vec3b>(p)[0],image1.at<cv::Vec3b>(p)[1],image1.at<cv::Vec3b>(p)[2]);
				//cout<<"Pixel Green Color at "<<x<<"X"<<y<< ": "<< imgPaint.at<cv::Vec3b>(p)[1]<<endl;
				//sleep(1000);
				//cout << "Press \"Esc\" key to continue...\n\n";
				cv::waitKey(1);
				//while((cv::waitKey() & 0xff) != 27);

			}
			//while((cv::waitKey() & 0xff) != 27);
			//system("clear");
		}
	blend_images_vertical_and_horizontal(road_map, &imageCenterVertical, &imageCenterHorizontal);
	//cv::waitKey(0);

}


int
main(int argc, char **argv)
{
	read_parameters(argc, argv);
	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	signal(SIGINT, shutdown_module);

	carmen_map_t road_map;
	for (int i = g_road_map_index; i < argc; i++)
	{
		char *road_map_filename = argv[i];
		bool valid_map_on_file = (carmen_map_read_gridmap_chunk(road_map_filename, &road_map) == 0);
		if (valid_map_on_file)
		{
			cout << "File " << string(road_map_filename) << " being displayed... ("
					<< (i - g_road_map_index + 1) << " of " << (argc - g_road_map_index) << ")" << endl;
			road_mapper_display_road_map(&road_map, g_img_channels, g_class_bits);
		}
		else
			cout << "road_mapper_display_map: could not read offline map from file named: " << road_map_filename << endl;
	}
	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	return 0;
}
