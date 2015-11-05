#include <vector>
#include <cmath>

#include <carmen/carmen.h>
#include <carmen/stereo_util.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

#define _IMAGE_WIDTH_ 1280
#define _IMAGE_HEIGHT_ 960

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// como estou usando as coordenadas da imagem normalizadas
// (entre [-1 e 1] ao inves de [0 a 1280]), na hora de fazer
// a correcao eu preciso normalizar o desvio padrao dos pixels
// tambem, senao o fator de correcao fica absurdamente grande
// (o que faz sentido, imagine uma camera com desvio padrao
// por pixel de meia imagem! Isso que significaria um desvio
// padrao de 1.0 em coordenadas normalizadas);
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//#define _PIXEL_AVGDEV_ 1.0
#define _PIXEL_AVGDEV_ 0.0015625 // 1.0 / halph_width
//#define _PIXEL_AVGDEV_ 0.000078125 // 0.05 / halph_width
#define _DISPARITY_AVGDEV_ (0.1 / 640)

typedef struct
{
	float left_x, left_y;
	float right_x, right_y;

}MatchingPixel;

int num_points_x, num_points_y;
double baseline, focal_length;
vector<MatchingPixel> matchings;

int
parse_arguments(int argc, char **argv)
{
	if (argc < 6)
		return 0;

	num_points_y = atoi(argv[2]);
	num_points_x = atoi(argv[3]);

	focal_length = atof(argv[4]);
	baseline = atof(argv[5]);

	char image_filename[256];

	FILE *fileptr = fopen(argv[1], "r");

	while(!feof(fileptr))
	{
		fscanf(fileptr, "%s", image_filename);

		for(int i = 0; i < (num_points_x * num_points_y); i++)
		{
			MatchingPixel m;
			fscanf(fileptr, " %f %f ", &(m.left_x), &(m.left_y));
			matchings.push_back(m);
		}

		fscanf(fileptr, "\n%s", image_filename);

		for(int i = 0; i < (num_points_x * num_points_y); i++)
			fscanf(fileptr, " %f %f ", &(matchings[i].right_x), &(matchings[i].right_y));
	}

	fclose(fileptr);
	return 1;
}


double
dist(carmen_vector_3D_p p, carmen_vector_3D_p q)
{
	return sqrt(pow(p->x - q->x, 2) + pow(p->y - q->y, 2) + pow(p->z - q->z, 2));
}


carmen_position_t
normalize_point_coordinates(carmen_position_t point, int width, int height)
{
	carmen_position_t normalized_point;

	double halph_width = (double) width / 2.0;
	double halph_height = (double) height / 2.0;

	normalized_point.x = ((double) point.x - halph_width) / halph_width;
	normalized_point.y = ((double) point.y - halph_height) / halph_height;

	return normalized_point;
}


void
calculate_distance(MatchingPixel &m, MatchingPixel &n)
{
	carmen_position_t left_point_m, left_point_n;
	carmen_position_t right_point_m, right_point_n;

	carmen_vector_3D_p point3D_with_bias_m, point3D_with_bias_n;
	carmen_vector_3D_p point3D_without_bias_m, point3D_without_bias_n;

	// inicializa os pontos
	left_point_m.x = m.left_x;
	left_point_m.y = m.left_y;
	right_point_m.x = m.right_x;
	right_point_m.y = m.right_y;

	left_point_n.x = n.left_x;
	left_point_n.y = n.left_y;
	right_point_n.x = n.right_x;
	right_point_n.y = n.right_y;

	// coloca em coordenadas normalizadas
	left_point_m = normalize_point_coordinates(left_point_m, _IMAGE_WIDTH_, _IMAGE_HEIGHT_);
	right_point_m = normalize_point_coordinates(right_point_m, _IMAGE_WIDTH_, _IMAGE_HEIGHT_);
	left_point_n = normalize_point_coordinates(left_point_n, _IMAGE_WIDTH_, _IMAGE_HEIGHT_);
	right_point_n = normalize_point_coordinates(right_point_n, _IMAGE_WIDTH_, _IMAGE_HEIGHT_);

	// calcular a distancia usando a medida com bias
	point3D_with_bias_m = reproject_single_point_to_3D_in_left_camera_frame(left_point_m, right_point_m, baseline, focal_length);
	point3D_with_bias_n = reproject_single_point_to_3D_in_left_camera_frame(left_point_n, right_point_n, baseline, focal_length);

	// calcular a distancia usando a medida sem bias
	//point3D_without_bias_m = reproject_single_point_to_3D_in_left_camera_frame_with_bias_reduction(left_point_m, right_point_m, baseline, focal_length, _PIXEL_AVGDEV_);
	//point3D_without_bias_n = reproject_single_point_to_3D_in_left_camera_frame_with_bias_reduction(left_point_n, right_point_n, baseline, focal_length, _PIXEL_AVGDEV_);

	point3D_without_bias_m = (carmen_vector_3D_p) calloc (1, sizeof(carmen_vector_3D_t));
	point3D_without_bias_n = (carmen_vector_3D_p) calloc (1, sizeof(carmen_vector_3D_t));
	(*point3D_without_bias_m) = (*point3D_with_bias_m);
	(*point3D_without_bias_n) = (*point3D_with_bias_n);
	// from paper Bias Reduction and Filter Convergence for Long Range Stereo (G. Sibley et al.)
	point3D_without_bias_m->z = point3D_without_bias_m->z +
		((_DISPARITY_AVGDEV_ * baseline * focal_length) / pow(left_point_m.x - right_point_m.x, 3));
	point3D_without_bias_n->z = point3D_without_bias_n->z +
		((_DISPARITY_AVGDEV_ * baseline * focal_length) / pow(left_point_n.x - right_point_n.x, 3));

	printf("%f %f %f %f %f %f %f %f ",
		left_point_m.x, left_point_m.y,
		right_point_m.x, right_point_m.y,
		left_point_n.x, left_point_n.y,
		right_point_n.x, right_point_n.y
	);

	// imprimir m, n e as distancias com e sem bias
	printf("%f %f %f %f %f %f %f ",
		point3D_with_bias_m->x, point3D_with_bias_m->y, point3D_with_bias_m->z,
		point3D_with_bias_n->x, point3D_with_bias_n->y, point3D_with_bias_n->z,
		(float) dist(point3D_with_bias_m, point3D_with_bias_n)
	);

	printf("%f %f %f %f %f %f %f",
		point3D_without_bias_m->x, point3D_without_bias_m->y, point3D_without_bias_m->z,
		point3D_without_bias_n->x, point3D_without_bias_n->y, point3D_without_bias_n->z,
		dist(point3D_without_bias_m, point3D_without_bias_n)
	);

	printf("\n");
}


void
perform_test()
{
	for(unsigned int i = 0; i < (matchings.size() - 1); i++)
		calculate_distance(matchings[i], matchings[i + 1]);
}


void
show_usage_information_and_exit(char *program_name)
{
	printf("\n");
	printf("\nUse %s <detection-files-l-and-r-in-sequency> <num-corners-y> <num-corners-x> <focal length> <baseline>\n", program_name);
	printf("\n");

	exit(-1);
}


int
main(int argc, char **argv)
{
	if (!parse_arguments(argc, argv))
		show_usage_information_and_exit(argv[0]);

	printf("lm_x lmy rmx rmy lnx lny rnx rny ");
	printf("m3D_x m3D_y m3D_z n3D_x n3D_y n3D_z dist_m3D_n3D ");
	printf("m3D_corrected_x m3D_corrected_y m3D_corrected_z n3D_corrected_x n3D_corrected_y n3D_corrected_z dist_m3D_corrected_n3D_corrected\n");

	perform_test();

	return 0;
}


