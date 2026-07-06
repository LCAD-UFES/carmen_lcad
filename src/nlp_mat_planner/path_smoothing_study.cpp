#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <carmen/carmen.h>
#include <carmen/collision_detection.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_linalg.h>
#include "nlp_mat_planner.h"

#define USE_CONTROL_SMOOTHING_VALUES 1

double distance_between_front_and_rear_axles = 2.625;
double model_predictive_planner_obstacles_safe_distance = 0.6;
double max_phi = 0.5237 * 0.6;

double smoothness_cost = 0.0;
double curvature_cost = 0.0;
double obstacle_cost = 0.0;
double theta_displacement_cost = 0.0;
double line_following_cost = 0.0;

int
get_key_non_blocking(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	return (ch);
}


int
control_smoothing_values()
{
	int ch = get_key_non_blocking();
//	double temp_value = 0.0;
	int signal_return = 0;

//Teclas:
// 1 - 4, altera individualmente os pesos. --- Não funciona, scanf esta bugado
//
//0 finaliza o programa

	switch (ch)
	{
		/*
		case '1':
			printf("Digite o valor de w1 (smoothness_cost), e pressione ENTER. O valor deve ser double (Ex: 1.0) \n");	

			scanf(" %lf", &temp_value);
			printf("temp_value = %lf\n", temp_value);
			signal_return = 1;

			smoothness_cost = temp_value;
			break;
		case '2':
			printf("Digite o valor de w2 (curvature_cost), e pressione ENTER. O valor deve ser double (Ex: 1.0) \n");	
			scanf(" %lf", &temp_value);
			signal_return = 1;
			curvature_cost = temp_value;
			break;
	
		case '3':
			printf("Digite o valor de w3 (obstacle_cost), e pressione ENTER. O valor deve ser double (Ex: 1.0) \n");	
			scanf(" %lf", &temp_value);
			signal_return = 1;
			obstacle_cost = temp_value;
			break;
		case '4':
			printf("Digite o valor de w4 (theta_displacement_cost), e pressione ENTER. O valor deve ser double (Ex: 1.0) \n");	
			scanf(" %lf", &temp_value);
			signal_return = 1;
			theta_displacement_cost = temp_value;
			break;
*/
		case 'q':
			smoothness_cost++;
			signal_return = 1;
			break;
		case 'a':
			smoothness_cost--;
			signal_return = 1;
			break;
		case 'w':
			curvature_cost++;
			signal_return = 1;
			break;
		case 's':
			curvature_cost--;
			signal_return = 1;
			break;
		case 'e':
			obstacle_cost++;
			signal_return = 1;
			break;
		case 'd':
			obstacle_cost--;
			signal_return = 1;
			break;
		case 'r':
			theta_displacement_cost++;
			signal_return = 1;
			break;
		case 'f':
			theta_displacement_cost--;
			signal_return = 1;
			break;
		case 't':
			line_following_cost++;
			signal_return = 1;
			break;
		case 'g':
			line_following_cost--;
			signal_return = 1;
			break;
		case '0':
			signal_return = -1;
			break;

		default:
			break;

	}

	return signal_return;

}	


static void
load_distance_map(carmen_obstacle_distance_mapper_map_message *obstacle_distance_map)
{
//	FILE *map_file = fopen("/home/lcad/carmen/bin/obstacle_distance_map.map", "r");
	FILE *map_file = fopen("obstacle_distance_map.map", "r");

	fread(&(obstacle_distance_map->config), sizeof(carmen_map_config_t), 1, map_file);
	fread(&(obstacle_distance_map->size), sizeof(int), 1, map_file);
	obstacle_distance_map->complete_x_offset = (short int *) malloc(obstacle_distance_map->size * sizeof(short int));
	obstacle_distance_map->complete_y_offset = (short int *) malloc(obstacle_distance_map->size * sizeof(short int));
	fread(obstacle_distance_map->complete_x_offset, sizeof(short int), obstacle_distance_map->size, map_file);
	fread(obstacle_distance_map->complete_y_offset, sizeof(short int), obstacle_distance_map->size, map_file);

	fclose(map_file);
}


FILE *gnuplot_pipe2;


void
plot_paths(carmen_robot_and_trailers_traj_point_t *path, carmen_robot_and_trailers_traj_point_t *path_smoothed, int path_size, int smoothed_size)
{
	static bool first_time = true;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipe2 = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe2, "set size square\n");
		fprintf(gnuplot_pipe2, "set size ratio -1\n");
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data2.txt", "w");
	for (int i = 0; i < path_size; i++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n",
				path[i].x, path[i].y,
				path_smoothed[i].x, path_smoothed[i].y);
	}
	fclose(gnuplot_data_file);

	gnuplot_data_file = fopen("gnuplot_data3.txt", "w");
	for (int i = 0; i < path_size; i++)
	{
		if (is_anchor_point(i, path, path_size))
			fprintf(gnuplot_data_file, "%lf %lf\n", path[i].x, path[i].y);
	}
	fclose(gnuplot_data_file);

	gnuplot_data_file = fopen("gnuplot_data4.txt", "w");
	for (int i = 0; i < smoothed_size; i++)
	{
		if (is_anchor_point(i, path_smoothed, smoothed_size))
			fprintf(gnuplot_data_file, "%lf %lf\n", path_smoothed[i].x, path_smoothed[i].y);
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe2, "plot "
			"'gnuplot_data2.txt' u 1:2 w linespoints t 'path', 'gnuplot_data2.txt' u 3:4 w linespoints t 'path_smoothed', "
			"'gnuplot_data3.txt' u 1:2 lc rgb 'red' t 'anchor points path', "
			"'gnuplot_data4.txt' u 1:2 lc rgb 'blue' t 'anchor points path_smoothed'\n");

	fflush(gnuplot_pipe2);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
//	FILE *carmen = fopen("/home/lcad/carmen/src/offroad_planner/caco.txt", "r");

	FILE *carmen = fopen("caco.txt", "r");

    char line[2048];
    double x, y, theta, v, d; //,phi;
    int i; //, xi, yi, thetai;
    carmen_robot_and_trailers_traj_point_t input_path[2048];
    carmen_robot_and_trailers_traj_point_t input_path_smoothed[2048];

    int size = 0;
    // x 7757913.83, y -363576.82, phi 0.00, theta -27.44, v 0.903672, displacement 1.00	 273, 122, 7
    while (fgets(line, 2047, carmen))
    {
    	printf("%s", line);
    	sscanf(line, " i %d, x %lf, y %lf, theta %lf, v %lf dist_2D [i]->[i+1] %lf",&i, &x, &y, &theta, &v, &d);
//    	printf("*i %d, x %lf, y %lf, phi %lf, theta %lf, v[i] %lf, dist_2D [i]->[i+1] %lf	 %d, %d, %d\n",
  //  			i, x, y, phi, theta, v, d, xi, yi, thetai);

	printf("i %d, x %lf, y %lf, theta %lf, dist_2D [i]->[i+1] %lf\n", i, x, y, theta, d);

    	printf("%lf %lf\n", x, y);

    	input_path[size].x = x;
    	input_path[size].y = y;
    	input_path[size].theta = M_PI * (theta / 180.0);
    	input_path[size].v = v;
    	input_path[size].phi = 0.0;
    	input_path_smoothed[size] = input_path[size];

    	size++;


    }

    int original_size = size;
    carmen_obstacle_distance_mapper_map_message obstacle_distance_map;
    load_distance_map(&obstacle_distance_map);

    smooth_path_using_conjugate_gradient_study((carmen_robot_and_trailers_traj_point_t *) input_path_smoothed, size, &obstacle_distance_map,
    		model_predictive_planner_obstacles_safe_distance, max_phi, smoothness_cost, curvature_cost, obstacle_cost, theta_displacement_cost, line_following_cost);

	plot_paths(input_path, input_path_smoothed, size, size);
	if (!USE_CONTROL_SMOOTHING_VALUES)
		getchar();
	else
		printf("Instruções para usar o controlador de parâmetros da suavização:\nq/a: Incrementa/Decrementa smoothness_cost\nw/s: Incrementa/Decrementa curvature_cost\ne/d: Incrementa/Decrementa obstacle_cost\nr/f: Incrementa/Decrementa theta_displacement_cost\nt/g: Incrementa/Decrementa line_following_cost\n0: Sair do programa\n");
	
	int signal_control;
	while (USE_CONTROL_SMOOTHING_VALUES)
	{
		signal_control = control_smoothing_values();
		
		if (signal_control == -1)
			break;
		
		else if (signal_control == 1)
		{
			memcpy(input_path_smoothed, input_path, sizeof(input_path_smoothed));

			printf("Suavização iniciada, aguarde.\n");
			smooth_path_using_conjugate_gradient_study((carmen_robot_and_trailers_traj_point_t *) input_path_smoothed, size, &obstacle_distance_map,
    		model_predictive_planner_obstacles_safe_distance, max_phi, smoothness_cost, curvature_cost, obstacle_cost, theta_displacement_cost, line_following_cost);
			plot_paths(input_path, input_path_smoothed, original_size, size);	
			for (int i = 0; i < size; i++)
			{
				printf("Novos pontos: %d %f %f %f %f\n", i, input_path_smoothed[i].x, input_path_smoothed[i].y, input_path_smoothed[i].theta, input_path_smoothed[i].v );
			}
			printf("Suavização terminada!\n");
				size = original_size;
		}

	}

	return (0);
}
