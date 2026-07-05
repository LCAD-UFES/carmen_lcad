#include "nlp_mat_planner_path_planner_astar.h"
#include <carmen/model_predictive_planner_optimizer.h>
#include <carmen/command.h>
#ifdef _REMOVE_BOOST
// #include <boost/math/special_functions/ellint_1.hpp>
// #include <boost/math/special_functions/ellint_2.hpp>
// #include <boost/math/special_functions/jacobi_elliptic.hpp>
#else
#include <boost/math/special_functions/ellint_1.hpp>
#include <boost/math/special_functions/ellint_2.hpp>
#include <boost/math/special_functions/jacobi_elliptic.hpp>
#endif
#include "carmen/global.h"
#include "g2o/types/slam2d/se2.h"

#include <time.h>
#include <carmen/mapper_interface.h>
#include <carmen/global_graphics.h>

#define DIFF2D(x1, x2) {x1.x - x2.x, x1.y - x2.y, 0.0, 0, {0.0, 0.0, 0.0, 0.0, 0.0}, 0.0, 0.0}
#define THETA_DIFF(x1, x2) (carmen_normalize_theta((carmen_normalize_theta((x1).theta) - carmen_normalize_theta((x2).theta))))
#define BETA_DIFF(x1, x2) (carmen_normalize_theta((carmen_normalize_theta((x1).trailer_theta[0]) - carmen_normalize_theta((x2).trailer_theta[0]))))

#define DIRECTION(x) x > 0.0 ? 1 : 0;

#define ASTAR_TIMEOUT (30.0) // seconds

// #define SEND_VIRTUAL_LASER 1

#define PRINT_NODES 0

// Salvar arquivos de expansao dos nos - adicionar o tempo no nome do arquivo se quiser
//#define DRAW_EXPANSION_TREE
//#define PRINT_TIME
//#define DRAW_PATHS_AND_POSES
//#define DRAW_LINES
//#define DRAW_BLOCKS

cv::Mat map_image;
cv::Mat map_image_2;
cv::Mat map_image_3;

static int expanded_nodes_cont = 0;

double astar_timeout = 0.0;
double curr_time = 0.0;

extern int time_mult;
extern carmen_robot_ackerman_config_t robot_config;
extern carmen_semi_trailers_config_t semi_trailer_config;
extern carmen_path_planner_astar_t astar_config;

extern carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map;
extern carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map_print_map;


// extern nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map;
extern double *nonholonomic_heuristic_cost_map;
extern int nonholonomic_heuristic_cost_map_loaded;
extern int nonholonomic_heuristic_cost_map_trailer_loaded;
extern double path_smoothing_obstacles_safe_distance;

extern double max_phi_multiplier;
extern int semi_trailer_solver_type;

extern carmen_behavior_selector_task_t *behavior_selector_task;

int grid_state_map_x_size;
int grid_state_map_y_size;
extern int number_of_trailers_for_path_planning;

carmen_route_planner_road_network_message *current_road_network_message = NULL;

int GlobalState::reverse_planning = 0;
int use_unity_simulator = 0;
carmen_robot_and_trailers_pose_t *GlobalState::localizer_pose = NULL;
int GlobalState::eliminate_path_follower = 1;
int backwards_finisher_on = 0;
int count_2 = 0;
// Tabela do trailer
// nonholonomic_heuristic_cost_trailer_p *****cost_map_trailer;
nonholonomic_heuristic_cost_trailer_p cost_map_trailer;

const int boost_arity = 2;

char expansion_tree_file_name[] = "Expansion_illustration_0.png";

extern carmen_mapper_virtual_laser_message virtual_laser_message;

extern double param_offroad_max_reed_shepp_distance;



static const int MAP_SCALE = 4;

void publish_simulated_objects()
{
	virtual_laser_message.host = carmen_get_host();
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
	virtual_laser_message.num_positions = 0;
}

void publish_larger_laser_point(double x, double y, double size, int color)
{
	double current_x = x - size;
	double current_y = y - size;

	static double step = 0.05;
	int num_points = (int)(2 * size / step) + 1;

	for (int i = 0; i < num_points; i++)
	{
		for (int j = 0; j < num_points; j++)
		{
			double point_x = current_x + i * step;
			double point_y = current_y + j * step;

			virtual_laser_message.positions[virtual_laser_message.num_positions].x = point_x;
			virtual_laser_message.positions[virtual_laser_message.num_positions].y = point_y;
			virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
			virtual_laser_message.num_positions++;
		}
	}
}

void publish_line_between_points(double x1, double y1, double x2, double y2, int color)
{
	static double step = 0.01;

	double dx = x2 - x1;
	double dy = y2 - y1;
	double distance = sqrt(dx * dx + dy * dy);

	int num_points = (int)(distance / step);

	for (int i = 0; i <= num_points; i++)
	{
		double t = (double)i / num_points;
		double point_x = x1 + t * dx;
		double point_y = y1 + t * dy;

		virtual_laser_message.positions[virtual_laser_message.num_positions].x = point_x;
		virtual_laser_message.positions[virtual_laser_message.num_positions].y = point_y;
		virtual_laser_message.colors[virtual_laser_message.num_positions] = color;
		virtual_laser_message.num_positions++;
	}
}

void draw_map(carmen_obstacle_distance_mapper_map_message *distance_map, cv::Mat &map_image)
{
	if (distance_map->complete_x_offset == NULL)
		return;

	unsigned int width = distance_map->config.x_size;
	unsigned int height = distance_map->config.y_size;
	unsigned int size = width * height;
	unsigned char map[3 * size];

	for (unsigned int i = 0; i < size; ++i)
	{
		unsigned int row = (height - 1) - i % height;

		unsigned int col = i / height;

		unsigned int index = row * width + col;

		if (0.0 == distance_map->complete_x_offset[i] && 0.0 == distance_map->complete_y_offset[i])
		{
			map[index * 3] = 0;
			map[index * 3 + 1] = 0;
			map[index * 3 + 2] = 0;
		}
		else
		{
			map[index * 3] = 255;
			map[index * 3 + 1] = 255;
			map[index * 3 + 2] = 255;
		}
	}

	//cv::Mat img(width, height, CV_8UC3, map);
	cv::Mat img(height, width, CV_8UC3, map);
	map_image = img.clone();

	//map_image_2 = img.clone();
	//map_image_3 = img.clone();
	
    cv::resize(img, map_image_2, cv::Size(width * MAP_SCALE, height * MAP_SCALE), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, map_image_3, cv::Size(width * MAP_SCALE, height * MAP_SCALE), 0, 0, cv::INTER_NEAREST);
}



void draw_state_in_opencv_image(state_node *current, carmen_map_config_t config, cv::Scalar color, cv::Mat map_image)
{
	if (current->parent == NULL)
		return;

	int img_x = (double)(current->pose.x - config.x_origin) / config.resolution;
	int img_y = (double)(current->pose.y - config.y_origin) / config.resolution;
	int parent_x = (double)(current->parent->pose.x - config.x_origin) / config.resolution;
	int parent_y = (double)(current->parent->pose.y - config.y_origin) / config.resolution;

	cv::line(map_image, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), color, 1, cv::LINE_8);
}

void draw_point_on_map_img(double x, double y, carmen_map_config_t config, cv::Scalar color, cv::Mat map_image)
{
	int img_x = (double)(x - config.x_origin) / config.resolution;
	int img_y = (double)(y - config.y_origin) / config.resolution;
	cv::circle(map_image, cv::Point(img_x, config.y_size - 1 - img_y), 4, color, -1, 8);
}

void draw_point_in_opencv_image(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t parent, carmen_map_config_t config, cv::Scalar color, int size = 1)
{
	int img_x = (double)(current.x - config.x_origin) / config.resolution;
	int img_y = (double)(current.y - config.y_origin) / config.resolution;
	int parent_x = (double)(parent.x - config.x_origin) / config.resolution;
	int parent_y = (double)(parent.y - config.y_origin) / config.resolution;

	cv::line(map_image, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), color, 1, cv::LINE_8);
}

// ######################################   TODO  #########################################################



void draw_text_on_image(cv::Mat &image, const std::string &text, int x, int y, int font_face, double font_scale, cv::Scalar color, int thickness)
{
	cv::putText(image, text,
				cv::Point(x, y), // posição do “baseline” do texto
				font_face,		 // ex.: cv::FONT_HERSHEY_SIMPLEX
				font_scale,		 // escala do texto
				color,			 // cor BGR
				thickness,		 // espessura das linhas
				cv::LINE_AA		 // tipo de linha (antialiasing)
	);
}


static double
wrap_pi(double a)
{
	// Use a sua carmen_normalize_theta se preferir
	while (a <= -M_PI) a += 2.0 * M_PI;
	while (a >  M_PI) a -= 2.0 * M_PI;
	return(a);
}


void
draw_solid_rectangle(carmen_point_t pose, carmen_map_config_t config, cv::Scalar color)
{
    // Calcula as posições da caixa com base nas coordenadas
    int img_x = (int)floor((pose.x - config.x_origin) / config.resolution);
    int img_y = (int)floor((pose.y - config.y_origin) / config.resolution);

    int width = 10; // Largura do retângulo (em pixels)
    int height = 10; // Altura do retângulo (em pixels)

    // Cria um retângulo sólido na imagem
    cv::rectangle(map_image, cv::Point(img_x - width / 2, config.y_size - 1 - img_y - height / 2), 
                  cv::Point(img_x + width / 2, config.y_size - 1 - img_y + height / 2), color, cv::FILLED);
}


void draw_truck_and_trailers_paths_and_poses(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t parent, size_t path_size, carmen_map_config_t config, cv::Scalar color, int size = 1)
{

	carmen_robot_and_trailers_traj_point_t current_robot_pose = current;
	carmen_robot_and_trailers_traj_point_t parent_robot_pose = parent;

	carmen_point_t current_semi_trailer_pose, parent_semi_trailer_pose;
	carmen_robot_and_trailers_path_point_t current_king_pin_pose, parent_king_pin_pose;

	carmen_point_t current_semi_trailers_poses[MAX_NUM_TRAILERS];
	carmen_point_t parent_semi_trailers_poses[MAX_NUM_TRAILERS];


			/*
		Espaço de cores:

		Nome		B	G	R
		Azul		255	0	0
		Verde		0	255	0
		Vermelho	0	0	255
		Ciano		255	255	0
		Magenta		255	0	255
		Amarelo		0	255	255
		Laranja		0	128	255
		Roxo		128	0	128
		Cinza		128	128	128

		*/


	std::vector<cv::Scalar> colors = {  cv::Scalar(0,0,255), 
										cv::Scalar(255,0,0), 
										cv::Scalar(0,128,255), 
										cv::Scalar(0,255,0), 
										cv::Scalar(128,	128,128)};

	int mod_print = 10;

	// print the truck line

	int img_x = (double)(current.x - config.x_origin) / config.resolution;
	int img_y = (double)(current.y - config.y_origin) / config.resolution;
	int parent_x = (double)(parent.x - config.x_origin) / config.resolution;
	int parent_y = (double)(parent.y - config.y_origin) / config.resolution;

	img_x *= MAP_SCALE;
    img_y *= MAP_SCALE;
    parent_x *= MAP_SCALE;
    parent_y *= MAP_SCALE;

    cv::line(map_image_2,
             cv::Point(img_x, MAP_SCALE * (config.y_size - 1 - img_y / MAP_SCALE)),
             cv::Point(parent_x, MAP_SCALE * (config.y_size - 1 - parent_y / MAP_SCALE)),
             colors[0],
             size * (MAP_SCALE-1),
             cv::LINE_AA);

	//cv::line(map_image_2, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), colors[0], 1, cv::LINE_8);

	for (int i = 0; i < semi_trailer_config.num_semi_trailers; i++)
	{
		double d = semi_trailer_config.semi_trailers[i].d;
		double M = semi_trailer_config.semi_trailers[i].M;

		if (i == 0)
		{

			current_semi_trailers_poses[i].x = current_robot_pose.x - M * cos(current_robot_pose.theta) - d * cos(current_robot_pose.trailer_theta[i]);
			current_semi_trailers_poses[i].y = current_robot_pose.y - M * sin(current_robot_pose.theta) - d * sin(current_robot_pose.trailer_theta[i]);
			current_semi_trailers_poses[i].theta = current_robot_pose.trailer_theta[i];

			parent_semi_trailers_poses[i].x = parent_robot_pose.x - M * cos(parent_robot_pose.theta) - d * cos(parent_robot_pose.trailer_theta[i]);
			parent_semi_trailers_poses[i].y = parent_robot_pose.y - M * sin(parent_robot_pose.theta) - d * sin(parent_robot_pose.trailer_theta[i]);
			parent_semi_trailers_poses[i].theta = parent_robot_pose.trailer_theta[i];
		}
		else
		{

			current_semi_trailers_poses[i].x = current_semi_trailers_poses[i - 1].x - M * cos(carmen_normalize_theta(current_robot_pose.trailer_theta[i-1])) - d * cos(carmen_normalize_theta(current_robot_pose.trailer_theta[i]));
			current_semi_trailers_poses[i].y = current_semi_trailers_poses[i - 1].y - M * sin(carmen_normalize_theta(current_robot_pose.trailer_theta[i-1])) - d * sin(carmen_normalize_theta(current_robot_pose.trailer_theta[i]));
			current_semi_trailers_poses[i].theta = current_robot_pose.trailer_theta[i];

			parent_semi_trailers_poses[i].x = parent_semi_trailers_poses[i - 1].x - M * cos(carmen_normalize_theta(parent_robot_pose.trailer_theta[i-1])) - d * cos(carmen_normalize_theta(parent_robot_pose.trailer_theta[i]));
			parent_semi_trailers_poses[i].y = parent_semi_trailers_poses[i - 1].y - M * sin(carmen_normalize_theta(parent_robot_pose.trailer_theta[i-1])) - d * sin(carmen_normalize_theta(parent_robot_pose.trailer_theta[i]));
			parent_semi_trailers_poses[i].theta = parent_robot_pose.trailer_theta[i];
		}

		int img_x = (int)floor((current_semi_trailers_poses[i].x - config.x_origin) / config.resolution);
		int img_y = (int)floor((current_semi_trailers_poses[i].y - config.y_origin) / config.resolution);
		int parent_x = (int)floor((parent_semi_trailers_poses[i].x - config.x_origin) / config.resolution);
		int parent_y = (int)floor((parent_semi_trailers_poses[i].y - config.y_origin) / config.resolution);

		img_x *= MAP_SCALE;
    	img_y *= MAP_SCALE;
    	parent_x *= MAP_SCALE;
    	parent_y *= MAP_SCALE;


#ifdef DRAW_LINES

		//cv::line(map_image_2, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), colors[i + 1], 1, cv::LINE_8);
		  cv::line(map_image_2,
             cv::Point(img_x, MAP_SCALE * (config.y_size - 1 - img_y / MAP_SCALE)),
             cv::Point(parent_x, MAP_SCALE * (config.y_size - 1 - parent_y / MAP_SCALE)),
             colors[i + 1],
             size * (MAP_SCALE-1),
             cv::LINE_AA);

#endif
	}

// desenha o tractor
#ifdef DRAW_BLOCKS

	 cv::Mat overlay = map_image_3.clone();

	float wheelbase = robot_config.length;
	float dlon = fabs(wheelbase / 2 - robot_config.distance_between_front_and_rear_axles);
	float dlat = 0;

	// rotaciona o vetor (dlon, dlat) por theta
	float dx = dlon * std::cos(current_robot_pose.theta) - dlat * std::sin(current_robot_pose.theta);
	float dy = dlon * std::sin(current_robot_pose.theta) + dlat * std::cos(current_robot_pose.theta);

	// centro real = ponto referencial - esse deslocamento

	float cx = current_robot_pose.x + dx;
	float cy = current_robot_pose.y + dy;

	// converte pra graus
	float th_deg = current_robot_pose.theta * 180.0f / CV_PI;

	cv::RotatedRect box(cv::Point2f(cx, cy), cv::Size2f(wheelbase, robot_config.width), th_deg);

	cv::Point2f verts[4];
	box.points(verts);

	cv::Point pts_pix[4];

	for (int j = 0; j < 4; j++)
	{
			int px = (double)(verts[j].x - config.x_origin) / config.resolution;

			int py = (double)(verts[j].y - config.y_origin) / config.resolution;

			// inverte Y de acordo com o sistema de imagem
            py = (double)config.y_size - 1.0 - py;

            /* aplica escala da imagem ampliada */
            px *= (double)MAP_SCALE;
            py *= (double)MAP_SCALE;

            pts_pix[j] = cv::Point(
                (int)std::round(px),
                (int)std::round(py));
	}

	if (!(path_size % mod_print) || (path_size == 0))
	{

		for (int j = 0; j < 4; j++)
		{
			//cv::line(map_image_3, pts_pix[j], pts_pix[(j + 1) % 4], colors[0], MAP_SCALE, cv::LINE_AA);
			cv::line(overlay, pts_pix[j], pts_pix[(j + 1) % 4], colors[0], MAP_SCALE, cv::LINE_AA);
		}
	}

#endif
// desenha os trailers


#ifdef DRAW_BLOCKS

	for (int i = 0; i < semi_trailer_config.num_semi_trailers; i++)
	{

		wheelbase = semi_trailer_config.semi_trailers[i].distance_between_axle_and_front + semi_trailer_config.semi_trailers[i].distance_between_axle_and_back;
		dlon = wheelbase / 2 - semi_trailer_config.semi_trailers[i].distance_between_axle_and_back;
		dlat = 0;

		// rotaciona o vetor (dlon, dlat) por theta
		dx = dlon * std::cos(current_semi_trailers_poses[i].theta) - dlat * std::sin(current_semi_trailers_poses[i].theta);
		dy = dlon * std::sin(current_semi_trailers_poses[i].theta) + dlat * std::cos(current_semi_trailers_poses[i].theta);

		// centro real = ponto referencial - esse deslocamento

		cx = current_semi_trailers_poses[i].x + dx;
		cy = current_semi_trailers_poses[i].y + dy;

		// converte pra graus
		th_deg = current_semi_trailers_poses[i].theta * 180.0f / CV_PI;

		cv::RotatedRect trailer_box(cv::Point2f(cx, cy), cv::Size2f(wheelbase, semi_trailer_config.semi_trailers[i].width), th_deg);

		cv::Point2f verts_2[4];
		box.points(verts_2);

		trailer_box.points(verts_2);

		for (int j = 0; j < 4; j++)
		{
			int px = (double)(verts_2[j].x - config.x_origin) / config.resolution;

			int py = (double)(verts_2[j].y - config.y_origin) / config.resolution;

			// inverte Y de acordo com o sistema de imagem
            py = (double)config.y_size - 1.0 - py;

            /* aplica escala da imagem ampliada */
            px *= (double)MAP_SCALE;
            py *= (double)MAP_SCALE;

            pts_pix[j] = cv::Point(
                (int)std::round(px),
                (int)std::round(py));
		}

		if (!(path_size % mod_print) || (path_size == 0))
		{

			for (int j = 0; j < 4; j++)
			{
				cv::line(overlay, pts_pix[j], pts_pix[(j + 1) % 4], colors[i+1], MAP_SCALE, cv::LINE_AA);
			}
		}
	}

    /* mistura overlay com o mapa para dar transparencia tipo figura do artigo */
    double alpha = 0.25;  /* quanto menor, mais transparente */
    cv::addWeighted(overlay, alpha, map_image_3, 1.0 - alpha, 0.0, map_image_3);

	if (semi_trailer_config.num_semi_trailers > 1)
	{

		carmen_point_t hitch_curr[MAX_NUM_TRAILERS], hitch_parent[MAX_NUM_TRAILERS];

		hitch_curr[0] = current_semi_trailer_pose;
		hitch_parent[0] = parent_semi_trailer_pose;
		current_semi_trailers_poses[0] = current_semi_trailer_pose;
		parent_semi_trailers_poses[0] = parent_semi_trailer_pose;

		for (int i = 0; i < semi_trailer_config.num_semi_trailers; i++)
		{

			double M_i = semi_trailer_config.semi_trailers[i].M;
			double d_i = semi_trailer_config.semi_trailers[i].d;

			double theta_prev_curr = current_robot_pose.trailer_theta[i - 1];
			double theta_prev_par = parent_robot_pose.trailer_theta[i - 1];
			double theta_curr = current_robot_pose.trailer_theta[i];
			double theta_par = parent_robot_pose.trailer_theta[i];

			// 1) acha o ponto de engate deste trailer
			hitch_curr[i].x = hitch_curr[i - 1].x - M_i * cos(theta_prev_curr);
			hitch_curr[i].y = hitch_curr[i - 1].y - M_i * sin(theta_prev_curr);

			hitch_parent[i].x = hitch_parent[i - 1].x - M_i * cos(theta_prev_par);
			hitch_parent[i].y = hitch_parent[i - 1].y - M_i * sin(theta_prev_par);

			// 2) a partir do engate, vai até o eixo de rodas
			current_semi_trailers_poses[i].x = hitch_curr[i].x - d_i * cos(theta_curr);
			current_semi_trailers_poses[i].y = hitch_curr[i].y - d_i * sin(theta_curr);
			current_semi_trailers_poses[i].theta = theta_curr;

			parent_semi_trailers_poses[i].x = hitch_parent[i].x - d_i * cos(theta_par);
			parent_semi_trailers_poses[i].y = hitch_parent[i].y - d_i * sin(theta_par);
			parent_semi_trailers_poses[i].theta = theta_par;

			int img_x = (double)(current_semi_trailers_poses[i].x - config.x_origin) / config.resolution;
			int img_y = (double)(current_semi_trailers_poses[i].y - config.y_origin) / config.resolution;
			int parent_x = (double)(parent_semi_trailers_poses[i].x - config.x_origin) / config.resolution;
			int parent_y = (double)(parent_semi_trailers_poses[i].y - config.y_origin) / config.resolution;

			/*
			current_semi_trailers_poses[i].x = current_semi_trailers_poses[i - 1].x - semi_trailer_config.semi_trailers[i].M * cos(current_robot_pose.trailer_theta[i - 1]) - semi_trailer_config.semi_trailers[i].d * cos(current_robot_pose.trailer_theta[i]);
			current_semi_trailers_poses[i].y = current_semi_trailers_poses[i - 1].y - semi_trailer_config.semi_trailers[i].M * sin(current_robot_pose.trailer_theta[i - 1]) - semi_trailer_config.semi_trailers[i].d * sin(current_robot_pose.trailer_theta[i]);
			current_semi_trailers_poses[i].theta = current_robot_pose.trailer_theta[i];

			parent_semi_trailers_poses[i].x = parent_semi_trailers_poses[i - 1].x - semi_trailer_config.semi_trailers[i].M * cos(parent_robot_pose.trailer_theta[i - 1]) - semi_trailer_config.semi_trailers[i].d * cos(parent_robot_pose.trailer_theta[i]);
			parent_semi_trailers_poses[i].y = parent_semi_trailers_poses[i - 1].y - semi_trailer_config.semi_trailers[i].M * sin(parent_robot_pose.trailer_theta[i - 1]) - semi_trailer_config.semi_trailers[i].d * sin(parent_robot_pose.trailer_theta[i]);
			parent_semi_trailers_poses[i].theta = parent_robot_pose.trailer_theta[i];

			int img_x = (double)(current_semi_trailers_poses[i].x - config.x_origin) / config.resolution;
			int img_y = (double)(current_semi_trailers_poses[i].y - config.y_origin) / config.resolution;
			int parent_x = (double)(parent_semi_trailers_poses[i].x - config.x_origin) / config.resolution;
			int parent_y = (double)(parent_semi_trailers_poses[i].y - config.y_origin) / config.resolution;

			*/
#endif

#ifdef DRAW_LINES

			// cv::line(map_image_2, cv::Point(img_x, config.y_size - 1 - img_y), cv::Point(parent_x, config.y_size - 1 - parent_y), colors[i], size, cv::LINE_AA);

#endif

#ifdef DRAW_BLOCKS

			float wheelbase = semi_trailer_config.semi_trailers[i].distance_between_axle_and_front + semi_trailer_config.semi_trailers[i].distance_between_axle_and_back;
			float center_offset = (semi_trailer_config.semi_trailers[i].distance_between_axle_and_front - semi_trailer_config.semi_trailers[i].distance_between_axle_and_back) / 2.0f;
			float dlat = 0.0f;

			float th = current_semi_trailers_poses[i].theta;

			// rotaciona o vetor (center_offset, dlat) por theta
			float dx = center_offset * std::cos(th) - dlat * std::sin(th);
			float dy = center_offset * std::sin(th) + dlat * std::cos(th);

			// centro real em metros
			float cx_m = current_semi_trailers_poses[i].x + dx;
			float cy_m = current_semi_trailers_poses[i].y + dy;

			// converte para pixel
			int cx = (cx_m - config.x_origin) / config.resolution;
			int cy = (cy_m - config.y_origin) / config.resolution;
			// inverte Y de acordo com o sistema de imagem
			cy = config.y_size - 1 - cy;

			// converte ângulo para graus CW (OpenCV)
			float th_deg = th * 180.0f / CV_PI;

			// dimensões em pixel
			float wheelbase_px = wheelbase / config.resolution;

			float width_px = semi_trailer_config.semi_trailers[i].width / config.resolution;

			// aplica escala para o mapa ampliado
			float cx_scaled = (float)cx * MAP_SCALE;
			float cy_scaled = (float)cy * MAP_SCALE;
			float wheelbase_px_scaled = wheelbase_px * MAP_SCALE;
			float width_px_scaled = width_px * MAP_SCALE;


			cv::RotatedRect box(cv::Point2f(cx_scaled, cy_scaled),cv::Size2f(wheelbase_px_scaled, width_px_scaled),-th_deg);

			cv::Point2f verts[4];
			box.points(verts);

			cv::Point pts_pix[4];
			for (int j = 0; j < 4; j++)
			{
				int px = (int)std::round(verts[j].x);
				int py = (int)std::round(verts[j].y);
				pts_pix[j] = cv::Point(px, py);

			}

			if (!(path_size % mod_print))
			{
				for (int j = 0; j < 4; j++)
				{
						cv::line(overlay ,pts_pix[j],pts_pix[(j + 1) % 4],colors[i+1], MAP_SCALE, cv::LINE_AA);
				}
			}

		}
	}
#endif

}




	

static int
sign(double x)
{
	if (x >= 0.0)
		return (1);
	else
		return (-1);
}

inline int
get_grid_state_map_index(index_node current_index)
{

	//	return ((current_index.direction * astar_config.state_map_beta_resolution * astar_config.state_map_theta_resolution * grid_state_map_y_size * grid_state_map_x_size) +
	//			(current_index.trailer_theta[0] * astar_config.state_map_theta_resolution * grid_state_map_y_size * grid_state_map_x_size) +
	//			(current_index.theta * grid_state_map_y_size * grid_state_map_x_size) +
	//			(current_index.y * grid_state_map_x_size) + current_index.x);

	int index =
		((current_index.direction * astar_config.state_map_theta_resolution * grid_state_map_y_size * grid_state_map_x_size) +
		 (current_index.theta * grid_state_map_y_size * grid_state_map_x_size) +
		 (current_index.y * grid_state_map_x_size) + current_index.x);

	for (int ii = 0; ii < number_of_trailers_for_path_planning; ii++)
	{
		index += current_index.trailer_theta[ii] * pow(astar_config.state_map_beta_resolution, ii) * 2 * astar_config.state_map_theta_resolution * grid_state_map_y_size * grid_state_map_x_size; // 2 é a dimensão da direção
	}

	return index;
}

inline int
get_non_holonomic_car_index(int x, int y, int theta)
{
	int x_and_y_size = astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution;
	return ((theta * x_and_y_size * x_and_y_size) +
			(y * x_and_y_size) + x);
}

inline int
get_trailer_table_map_index(int x, int y, int theta, int beta, int beta2)
{
	int x_and_y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	return ((beta2 * astar_config.precomputed_cost_trailer_beta_size * astar_config.precomputed_cost_trailer_theta_size * x_and_y_size * x_and_y_size) +
			(beta * astar_config.precomputed_cost_trailer_theta_size * x_and_y_size * x_and_y_size) +
			(theta * x_and_y_size * x_and_y_size) +
			(y * x_and_y_size) + x);
}

double
get_distance_to_nearest_lane(carmen_route_planner_road_network_message *road_network_message, carmen_robot_and_trailers_traj_point_t current)
{
	if (road_network_message->offroad_planner_request == PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE)
		return (1000.0);

	double distance_to_nearest_edge = 1000.0;
	for (int pose_index = 0; pose_index < (road_network_message->nearby_lanes_size - 1); pose_index++)
	{
		double distance;
		if (fabs(carmen_normalize_theta(road_network_message->nearby_lanes[pose_index].theta - current.theta)) > carmen_degrees_to_radians(110.0))
			distance = 1000.0;
		else
		{
			distance = DIST2D(road_network_message->nearby_lanes[pose_index], current);
			if (distance < 5.0)
			{
				int status;
				double temp_distance = DIST2D(carmen_get_point_nearest_to_trajectory(&status, road_network_message->nearby_lanes[pose_index], road_network_message->nearby_lanes[pose_index + 1], current, 0.1), current);
				if (status == POINT_WITHIN_SEGMENT)
					distance = temp_distance;

				if (distance < distance_to_nearest_edge)
					distance_to_nearest_edge = distance;
			}
		}
	}

	return (distance_to_nearest_edge);
}

bool near_enough(double distance, double theta_diff, double beta_diff)
{
	if ((distance < astar_config.goal_constraints.goal_achieved_distance) &&
		(fabs(theta_diff) < astar_config.goal_constraints.goal_achieved_theta) &&
		(number_of_trailers_for_path_planning == 0 || (fabs(beta_diff) < astar_config.goal_constraints.goal_achieved_trailer_theta)))
		return (true);
	else
		return (false);
}

bool near_enough_multi_trailer(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal)
{
	double distance = (DIST2D(current, goal));
	double theta_diff = (THETA_DIFF(current, goal));

	if (distance < astar_config.goal_constraints.goal_achieved_distance && fabs(theta_diff) < astar_config.goal_constraints.goal_achieved_theta)
	{
		// printf("near enough!\n %lf %lf [%lf %lf] - %lf %lf %lf\n", distance, theta_diff, astar_config.goal_constraints.goal_achieved_distance, astar_config.goal_constraints.goal_achieved_theta, current.x, current.y, current.theta);
		if (number_of_trailers_for_path_planning == 0 || (*behavior_selector_task == BEHAVIOR_SELECTOR_PARK))
		{
			return (true);
		}
		else
		{
			for (int i = 0; i < number_of_trailers_for_path_planning; i++)
			{
				if (fabs(carmen_normalize_theta(carmen_normalize_theta(current.trailer_theta[i]) - carmen_normalize_theta(goal.trailer_theta[i]))) > MAX_POLY_BETA_DIFF)
				{
					return (false);
				}
			}
			return (true);
		}
	}
	else
	{
		return (false);
	}
}

int get_astar_map_theta(double theta, int precomputed_cost_theta_size)
{
	double up_to_2pi_theta = (theta < 0.0) ? theta + 2.0 * M_PI : theta;
	int z = round((up_to_2pi_theta * (double)(precomputed_cost_theta_size - 1)) / (2.0 * M_PI));

	return (z);
}

int get_cost_map_int_theta(double theta, int precomputed_cost_theta_size)
{
	int z;

	theta = carmen_normalize_theta(theta);

	if (theta >= 0.0)
	{
		z = round((theta * (double)(precomputed_cost_theta_size - 1)) / (1.0 * M_PI));
		if (z > precomputed_cost_theta_size / 2)
			z = precomputed_cost_theta_size / 2;
	}
	else
	{
		theta = -theta;
		z = round((theta * (double)(precomputed_cost_theta_size - 1)) / (1.0 * M_PI));
		if (z > precomputed_cost_theta_size / 2)
			z = precomputed_cost_theta_size / 2;

		z = precomputed_cost_theta_size - z;

		if (z > precomputed_cost_theta_size - 1)
			z = 0;
	}

	return (z);
}

int get_astar_map_beta(double theta, double theta1, int map_beta_resolution)
{

	if (number_of_trailers_for_path_planning == 0)
		return 0;

	double beta = convert_theta1_to_beta(theta, theta1); // os cálculos foram feitos para o beta relativo, então transformo o beta em relativo nessa linha
	int z;

	beta = carmen_normalize_theta(beta);

	if (beta >= 0.0)
	{
		z = round((beta * (double)(map_beta_resolution - 1)) / (0.25 * M_PI));
		if (z > map_beta_resolution / 2)
			z = map_beta_resolution / 2;
	}
	else
	{
		beta = -beta;
		z = round((beta * (double)(map_beta_resolution - 1)) / (0.25 * M_PI));
		if (z > map_beta_resolution / 2)
			z = map_beta_resolution / 2;

		z = map_beta_resolution - z;

		if (z > map_beta_resolution - 1)
			z = 0;
	}

	return (z);
}

int get_grid_state_map_x(double x, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	return (round((double)((x - obstacle_distance_grid_map->config.x_origin) / astar_config.state_map_resolution)));
}

int get_grid_state_map_y(double y, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	return (round((double)((y - obstacle_distance_grid_map->config.y_origin) / astar_config.state_map_resolution)));
}

double
get_distance_map_x(int x, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	return ((double)(x * astar_config.state_map_resolution) + obstacle_distance_grid_map->config.x_origin);
}

double
get_distance_map_y(int y, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	return ((double)(y * astar_config.state_map_resolution) + obstacle_distance_grid_map->config.y_origin);
}

void
// get_current_pos_in_grid_state_map(state_node *current_node, int &x, int &y, int &theta, int &beta, int &direction,
get_current_pos_in_grid_state_map(state_node *current_node, index_node &current_index,
								  carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	current_index.x = get_grid_state_map_x(current_node->pose.x, obstacle_distance_grid_map);
	current_index.y = get_grid_state_map_y(current_node->pose.y, obstacle_distance_grid_map);
	current_index.theta = get_astar_map_theta(current_node->pose.theta, astar_config.state_map_theta_resolution);
	current_index.direction = DIRECTION(current_node->pose.v);
	current_index.trailer_theta[0] = get_astar_map_beta(current_node->pose.theta, current_node->pose.trailer_theta[0], astar_config.state_map_beta_resolution);

	if (number_of_trailers_for_path_planning > 1)
	{
		for (int i = 1; i < number_of_trailers_for_path_planning; i++)
			current_index.trailer_theta[i] = get_astar_map_beta(current_node->pose.trailer_theta[i - 1], current_node->pose.trailer_theta[i], astar_config.state_map_beta_resolution);
	}
}

carmen_map_t *
copy_grid_mapping_to_map(carmen_map_t *map, carmen_mapper_map_message *grid_map)
{
	int i;

	if (!map)
	{
		map = (carmen_map_t *)malloc(sizeof(carmen_map_t));
		map->map = (double **)malloc(grid_map->config.x_size * sizeof(double *));
	}

	map->config = grid_map->config;
	map->complete_map = grid_map->complete_map;
	for (i = 0; i < map->config.x_size; i++)
		map->map[i] = map->complete_map + i * map->config.y_size;

	return (map);
}

void save_map(char *file_name, double *map, int x_size, int y_size)
{
	carmen_map_t out_map;

	double *map_copy = (double *)malloc(x_size * y_size * sizeof(double));
	memcpy((void *)map_copy, (void *)map, x_size * y_size);

	double min_value, max_value;
	min_value = 100000000.0;
	max_value = -100000000.0;
	for (int i = 0; i < x_size * y_size; i++)
	{
		double value = map[i];
		if (value != 10000.0)
		{
			//			printf("Value = %f\n", value);
			if (value < min_value)
				min_value = value;
			if (value > max_value)
				max_value = value;
		}
		//		else
		//			printf("x %d, y %d, value %lf, x_size %d, y_size %d\n", i % x_size, i / x_size, value, x_size, y_size);
	}

	//	printf("map name %s, min_value %lf, max_value %lf\n", file_name, min_value, max_value);
	for (int i = 0; i < x_size * y_size; i++)
	{
		double value = map[i];
		if (value == 1000.0)
			value = 250.0;
		//		if (value != -1.0)
		//		{
		//			value = (value - min_value) / (max_value - min_value);
		//			if (value < 0.8)
		//				value = 0.8;
		//			value = (value - 0.8) / (1.0 - 0.8);
		//		}
		map_copy[i] = value;
	}
	out_map.complete_map = map_copy;
	out_map.config.x_size = x_size;
	out_map.config.y_size = y_size;
	out_map.config.resolution = 0.2;
	out_map.config.map_name = (char *)"test";
	out_map.config.x_origin = 2000000.0;
	out_map.config.y_origin = 2000000.0;

	out_map.map = (double **)malloc(sizeof(double *) * x_size);
	carmen_test_alloc(out_map.map);
	for (int x = 0; x < out_map.config.x_size; x++)
		out_map.map[x] = &(out_map.complete_map[x * out_map.config.y_size]);

	carmen_grid_mapping_save_map(file_name, &out_map);

	free(out_map.map);
	free(map_copy);
}

void copy_map(double *cost_map, std::vector<std::vector<double>> map, int x_size, int y_size)
{
	for (int x = 0; x < x_size; x++)
		for (int y = 0; y < y_size; y++)
			cost_map[x + y * x_size] = map[x][y];
}

void copy_map(std::vector<std::vector<double>> &map, double *cost_map, int x_size, int y_size)
{
	for (int x = 0; x < x_size; x++)
		for (int y = 0; y < y_size; y++)
			map[x][y] = cost_map[x + y * x_size];
}

double *
get_goal_distance_map(double *goal_distance_map, carmen_point_t *goal_pose, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	//	printf("Computando mapa da heurística com obstáculos\n");
	grid_state_map_x_size = round((obstacle_distance_grid_map->config.x_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);
	grid_state_map_y_size = round((obstacle_distance_grid_map->config.y_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);

	int x_size = grid_state_map_x_size;
	int y_size = grid_state_map_y_size;
	double *utility_map;
	if (!goal_distance_map)
		goal_distance_map = utility_map = (double *)calloc(x_size * y_size, sizeof(double));
	else
		utility_map = goal_distance_map;

	double *cost_map = (double *)calloc(x_size * y_size, sizeof(double));
	std::fill_n(cost_map, x_size * y_size, -1.0);
	for (int x = 0; x < x_size; x++)
	{
		for (int y = 0; y < y_size; y++)
		{
			carmen_position_t global_point = {get_distance_map_x(x, obstacle_distance_grid_map), get_distance_map_y(y, obstacle_distance_grid_map)};
			if (carmen_obstacle_avoider_distance_from_global_point_to_obstacle(&global_point, obstacle_distance_grid_map) < robot_config.model_predictive_planner_obstacles_safe_distance + 2.0 * obstacle_distance_grid_map->config.resolution)
				cost_map[y + x * y_size] = 1.0; // Espaco ocupado eh representado como 1.0
		}
	}

	std::vector<std::vector<double>> map(x_size, std::vector<double>(y_size));
	copy_map(map, cost_map, x_size, y_size);
	Planning exact_euclidean_distance_to_goal;
	exact_euclidean_distance_to_goal.setMap(map);
	exact_euclidean_distance_to_goal.expandObstacles(0.1);
	copy_map(cost_map, exact_euclidean_distance_to_goal.getExpandedMap(), x_size, y_size);
	int goal_x = get_grid_state_map_x(goal_pose->x, obstacle_distance_grid_map);
	int goal_y = get_grid_state_map_y(goal_pose->y, obstacle_distance_grid_map);
	copy_map(utility_map, exact_euclidean_distance_to_goal.pathDR(goal_y, goal_x), x_size, y_size);

	for (int i = 0; i < x_size * y_size; i++)
		if (utility_map[i] >= 50000.0) // O infinito de distacia eh representado como 50000.0, assim como o espaco ocupado.
			utility_map[i] = 1000.0;

	save_map((char *)"obstacle_heuristic.map", utility_map, x_size, y_size);
	//	printf("Mapa da heurística com obstáculos carregado!\n");
	free(cost_map);

	return (utility_map);
}

grid_state_p
alloc_grid_state_map()
{
	int theta_size = astar_config.state_map_theta_resolution;
	int beta_size = astar_config.state_map_beta_resolution;

	int size_of_grid = grid_state_map_x_size * grid_state_map_y_size * theta_size * 2 * pow(beta_size, number_of_trailers_for_path_planning);
	grid_state_p grid_state_map = (grid_state_p)malloc(sizeof(grid_state) * size_of_grid);
	carmen_test_alloc(grid_state_map);

	// double first = carmen_get_time();

	// Dessa forma é mais eficiente do que a de baixo
	memset(grid_state_map, 0, sizeof(grid_state) * size_of_grid);

	// int temp_index;
	// for (int i = 0; i < grid_state_map_x_size; i++)
	// {
	//     for (int j = 0; j < grid_state_map_y_size; j++)
	//     {
	//         for (int k = 0; k < theta_size; k++)
	//         {
	//             for (int m = 0; m < 2; m++)
	//             {
	//                 for (int l = 0; l < pow(beta_size, number_of_trailers_for_path_planning); l++)
	//                 {
	//                     index_node current_index = {i, j, k, {0, 0, 0, 0, 0}, m};
	//                     int temp_value = l;
	//
	//                     for (int ii = 0; ii < number_of_trailers_for_path_planning; ii++)
	//                     {
	//                         current_index.trailer_theta[ii] = temp_value % beta_size;
	//                         temp_value /= beta_size;
	//                     }
	//
	//                     temp_index = get_grid_state_map_index(current_index);
	//                     grid_state_map[temp_index].state = Not_visited;
	//                     grid_state_map[temp_index].g = 0.0;
	//                 }
	//             }
	//         }
	//     }
	// }

	// printf("First: %lf for %d\n", carmen_get_time() - first, size_of_grid);
	return (grid_state_map);
}

void clear_grid_state_map(grid_state_p astar_map)
{
	free(astar_map);
}

static int
load_cost_map()
{
	FILE *fp;
	int result;

	fp = fopen(astar_config.precomputed_cost_file_name, "r");
	printf("Carregando mapa de heurística sem obstáculos para IARA. Arquivo: %s\n", astar_config.precomputed_cost_file_name);
	if (fp == NULL)
	{
		printf("Houve um erro ao abrir a matriz e por isso será usado o Reed-Shepp como heurística.\n");
		printf("Verifique se o arquivo %s existe. Caso não exista, execute 'make update' em src/offroad_planner\n", astar_config.precomputed_cost_file_name);
		nonholonomic_heuristic_cost_map_loaded = 0;
		return 1;
	}

	for (int i = 0; i < astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution; i++)
	{
		for (int j = 0; j < astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution; j++)
		{
			for (int k = 0; k < astar_config.precomputed_cost_theta_size; k++)
			{
				int x, y, z;
				result = fscanf(fp, "%d %d %d %lf\n", &x, &y, &z, &nonholonomic_heuristic_cost_map[get_non_holonomic_car_index(i, j, k)]);
				if (result != 4)
				{
					printf("Erro in load_cost_map()\n");
					printf("É possível que os parâmetros offroad_planner_precomputed_cost estão incorretos ou a tabela está desatualizada.\n Execute um 'make update' em src/offroad_planner, e altere os parâmetros offroad_planner_precomputed_cost no .ini utilizado\n");
					printf("Será usado o Reed-Shepp como heurística...\n");
					nonholonomic_heuristic_cost_map_loaded = 0;
					fclose(fp);

					return (1);
				}
			}
		}
	}

	fclose(fp);
	nonholonomic_heuristic_cost_map_loaded = 1;
	printf("Mapa da heurística sem obstáculos carregado!\n");

	return (0);
}

void alloc_cost_map()
{
	// get_non_holonomic_car_index
	int x_size = round(astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution);
	int y_size = round(astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution);
	int theta_size = astar_config.precomputed_cost_theta_size;
	nonholonomic_heuristic_cost_map = (double *)calloc(x_size * y_size * theta_size, sizeof(double));
	carmen_test_alloc(nonholonomic_heuristic_cost_map);

	load_cost_map();
}

std::vector<carmen_robot_and_trailers_traj_point_t>
get_path_from_file(int indice)
{

	std::vector<carmen_robot_and_trailers_traj_point_t> path;

	if (indice == -1)
		return path;

	FILE *paths_file;
	char binary_file_type[1024];
	sprintf(binary_file_type, "%s_paths", astar_config.precomputed_cost_trailer_file_name);
	paths_file = fopen(binary_file_type, "rb");
	if (paths_file == NULL)
	{
		printf("Houve um erro ao abrir o arquivo %s.\n", binary_file_type);
		printf("Verifique se o arquivo %s existe. Caso não exista, execute 'make update' em src/offroad_planner\n", binary_file_type);
		nonholonomic_heuristic_cost_map_trailer_loaded = 0;
		exit(1);
	}

	// Valor 80 porque os paths sempre vão possuir 80 poses
	fseek(paths_file, indice * 80 * sizeof(carmen_robot_and_trailers_traj_point_t), SEEK_SET);
	carmen_robot_and_trailers_traj_point_t current_pose;
	carmen_robot_and_trailers_traj_point_t *alloc_path = NULL;
	alloc_path = (carmen_robot_and_trailers_traj_point_t *)calloc(80, sizeof(carmen_robot_and_trailers_traj_point_t));

	fread(alloc_path, sizeof(carmen_robot_and_trailers_traj_point_t), 80, paths_file);
	// Tenho quase certeza que isso vai quebrar por conta da mudança de size do carmen_robot_and_trailers_traj_point_t

	for (int i = 0; i < 80; i++)
	{
		//		int teste =	fread(&current_pose, sizeof(carmen_robot_and_trailer_traj_point_t), 1, paths_file);
		//		fread(&alloc_path[i], sizeof(carmen_robot_and_trailer_traj_point_t), 1, paths_file);
		current_pose.x = alloc_path[i].x;
		current_pose.y = alloc_path[i].y;
		current_pose.theta = alloc_path[i].theta;
		current_pose.num_trailers = alloc_path[i].num_trailers;
		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			current_pose.trailer_theta[z] = alloc_path[i].trailer_theta[z];
		current_pose.v = alloc_path[i].v;
		current_pose.phi = alloc_path[i].phi;
		path.push_back(current_pose);
		//		printf("Indice: %d %f %f %f %f %f %f\n", indice, current_pose.x, current_pose.y, current_pose.theta, current_pose.beta, current_pose.v, current_pose.phi);
		//		perror("?");
	}

	free(alloc_path);
	fclose(paths_file);

	return path;
}

#define NEW_COST_MAP 0

static int
open_trailer_cost_map()
{
	FILE *fp;
	printf("Carregando mapa de heurística sem obstáculos para o robô mais semi-trailer. Arquivo: %s\n", astar_config.precomputed_cost_trailer_file_name);
	fp = fopen(astar_config.precomputed_cost_trailer_file_name, "r");
	if (fp == NULL)
	{
		printf("Houve um erro ao abrir o mapa para robô e trailer...\n");
		printf("Verifique se o arquivo %s existe. Caso não exista, execute 'make update' em src/offroad_planner\n", astar_config.precomputed_cost_trailer_file_name);

		nonholonomic_heuristic_cost_map_trailer_loaded = 0;
		return (1);
	}
#if NEW_COST_MAP
	// Precisa obter novamente  os parametros da primeira linha para o loop de construção começar a partir da segunda
	// ATENÇÂO: Se algum mapa foi gerado pelo cost_matrix_trailer2 após o dia 18/03/2022, NEW_COST_MAP PRECISA ter valor 1
	int result_param = fscanf(fp, "%lf %d %d %lf\n", &(astar_config.precomputed_cost_trailer_size), &(astar_config.precomputed_cost_trailer_theta_size), &(astar_config.precomputed_cost_trailer_beta_size), &(astar_config.precomputed_cost_trailer_resolution));
	if (result_param != 4)
	{
		printf("Erro in open_trailer_cost_map(), line %d erro = %d\n", __LINE__, result_param);
		printf("Os parâmetros do mapa obtidos pela primeira linha do arquivo não foram lidos. Verifique se a tabela está atualizada executando um 'make update' em src_offroad_planner\n");
		exit(1);
	}
#endif

	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			for (int k = 0; k < astar_config.precomputed_cost_trailer_theta_size; k++)
			{
				for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
				{
					for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
					{
						int d, e, f, g, h, h_forward_valid, h_backward_valid;
						int temp_index = get_trailer_table_map_index(i, j, k, a, b);
#if NEW_COST_MAP
						int result = fscanf(fp, "%d %d %d %d %d %lf %lf %d %d %d\n", &d, &e, &f, &g, &h,
											//								&(cost_map_trailer[i][j][k][a][b]->h_forward), &(cost_map_trailer[i][j][k][a][b]->h_backward),
											&(cost_map_trailer[temp_index].h_forward), &(cost_map_trailer[temp_index].h_backward),
											&h_forward_valid, &h_backward_valid, &(cost_map_trailer[temp_index]->indice));

						//						get_path_from_file(cost_map_trailer[temp_index]->indice);
						if (result != 10)
						{
							printf("Erro in open_trailer_cost_map()\n");
							printf("É possível que os parâmetros offroad_planner_precomputed_cost estão incorretos ou a tabela está desatualizada.\n Execute um 'make update' em src/offroad_planner, e altere os parâmetros offroad_planner_precomputed_cost no .ini utilizado\n");
							exit(1);
						}
#else
						// get_trailer_table_map_index
						int result = fscanf(fp, "%d %d %d %d %d %lf %lf %d %d\n", &d, &e, &f, &g, &h,
											&(cost_map_trailer[temp_index].h_forward), &(cost_map_trailer[temp_index].h_backward),
											&h_forward_valid, &h_backward_valid);
						if (result != 9)
						{
							printf("Erro in open_trailer_cost_map()\n");
							printf("É possível que os parâmetros offroad_planner_precomputed_cost estão incorretos ou a tabela está desatualizada.\n Execute um 'make update' em src/offroad_planner, e altere os parâmetros offroad_planner_precomputed_cost no .ini utilizado\n");
							exit(1);
						}
#endif
						cost_map_trailer[temp_index].h_forward_valid = h_forward_valid;
						cost_map_trailer[temp_index].h_backward_valid = h_backward_valid;
					}
				}
			}
		}
	}

	fclose(fp);
	nonholonomic_heuristic_cost_map_trailer_loaded = 1;
	printf("Mapa da heurística sem obstáculos para TRAILER carregado!\n");

	return (0);
}

void alloc_trailer_cost_map()
{
#if NEW_COST_MAP
	FILE *fp;
	fp = fopen(astar_config.precomputed_cost_trailer_file_name, "r");
	if (fp == NULL)
	{
		printf("Houve um erro ao abrir o mapa para robô e trailer...\n");
		printf("Verifique se o arquivo %s existe. Caso não exista, execute 'make update' em src/offroad_planner\n", astar_config.precomputed_cost_trailer_file_name);

		nonholonomic_heuristic_cost_map_trailer_loaded = 0;
		exit(1);
	}

	FILE *paths_file;
	char binary_file_type[1024];
	sprintf(binary_file_type, "%s_paths", astar_config.precomputed_cost_trailer_file_name);
	paths_file = fopen(binary_file_type, "rb");
	if (paths_file == NULL)
	{
		printf("Houve um erro ao abrir o arquivo %s.\n", binary_file_type);
		printf("Verifique se o arquivo %s existe. Caso não exista, execute 'make update' em src/offroad_planner\n", binary_file_type);
		nonholonomic_heuristic_cost_map_trailer_loaded = 0;
		exit(1);
	}
	// Apenas para verificar se o arquivo existe. Os paths não serão lidos nesse método
	fclose(paths_file);

	// Obter parâmetros do map na primeira linha do arquivo
	int result_param = fscanf(fp, "%lf %d %d %lf\n", &(astar_config.precomputed_cost_trailer_size), &(astar_config.precomputed_cost_trailer_theta_size), &(astar_config.precomputed_cost_trailer_beta_size), &(astar_config.precomputed_cost_trailer_resolution));

	if (result_param != 4)
	{
		printf("Erro in open_trailer_cost_map(), line %d erro = %d\n", __LINE__, result_param);
		printf("Os parâmetros do mapa obtidos pela primeira linha do arquivo não foram lidos. Verifique se a tabela está atualizada executando um 'make update' em src_offroad_planner\n");
		exit(1);
	}
	fclose(fp);
#endif

	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	cost_map_trailer = (nonholonomic_heuristic_cost_trailer_p)calloc(x_size * y_size * astar_config.precomputed_cost_trailer_theta_size * astar_config.precomputed_cost_trailer_beta_size * astar_config.precomputed_cost_trailer_beta_size, sizeof(nonholonomic_heuristic_cost_trailer));
	carmen_test_alloc(cost_map_trailer);
	open_trailer_cost_map();
}

void
#ifdef _REMOVE_BOOST
clear_astar_search(priority_queue<state_node *, vector<state_node *>, StateNodePtrComparator> &FH,
				   grid_state_p astar_map, state_node *goal_node)
#else
clear_astar_search(boost::heap::d_ary_heap<state_node *, boost::heap::arity<boost_arity>, boost::heap::compare<StateNodePtrComparator>> &FH,
				   grid_state_p astar_map, state_node *goal_node)
#endif
{
	while (!FH.empty())
	{
		state_node *temp = FH.top();
		FH.pop();

		// Condição para impedir o seg fault quando a initial ou goal pose estão próximas de obstáculos. A condição abaixo só vai ser ativada quando o nó
		// temp for igual ao primeiro nó que entrou na lista, o initial node
		if (temp->parent == NULL)
		{
			free(temp);
			continue;
		}
		// Precisa verificar os parent de cada nó dentro da heap, caso contrário acontece memory leak
		temp->parent->branches--;
		state_node *n = temp->parent;
		free(temp);
		while (n != NULL && n->branches == 0)
		{
			state_node *temp_n;
			temp_n = n;
			n = n->parent;
			free(temp_n);
			if (n != NULL)
				n->branches--;
		}
		//
	}
#ifdef _REMOVE_BOOST
	priority_queue<state_node *, vector<state_node *>, StateNodePtrComparator> empty_FH;
	FH = move(empty_FH);
#else
	FH.clear();
#endif
	clear_grid_state_map(astar_map);
	free(goal_node);
}

state_node *
new_state_node(carmen_robot_and_trailers_traj_point_t *pose)
{
	state_node *new_state = (state_node *)malloc(sizeof(state_node));

	new_state->pose.x = pose->x;
	new_state->pose.y = pose->y;
	new_state->pose.theta = pose->theta;
	new_state->pose.num_trailers = pose->num_trailers;

	for (size_t ii = 0; ii < MAX_NUM_TRAILERS; ii++)
	{
		new_state->pose.trailer_theta[ii] = pose->trailer_theta[ii];
	}

	new_state->pose.phi = 0.0;
	new_state->pose.v = EXPAND_NODES_V;
	new_state->it_since_change_in_direction = 10000;
	new_state->number_of_change_in_direction = 0;
	new_state->pose_of_direction_changed = new_state->pose;

	new_state->g = 0.0;
	new_state->h = 0.0;
	new_state->f = 0.0;
	new_state->branches = 0;
	new_state->parent = NULL;

	return (new_state);
}


static int check_inside_goal_radius(double robot_x, double robot_y, double goal_x, double goal_y, double radius_param)
{
    if (radius_param <= 0.0) return 0;
    
    // hypot faz o cálculo de distância 2D: sqrt((dx^2) + (dy^2))
    double distance = hypot(robot_x - goal_x, robot_y - goal_y);
    
    return (distance < radius_param) ? 1 : 0;
}


std::vector<carmen_robot_and_trailers_traj_point_t>
reed_shepp_path(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal_state,
				int &check_collision, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double length_since_change_of_direction)
{
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	carmen_robot_and_trailers_traj_point_t rs_points[6]; // Por alguma razão, com o valor 5 acontece stack smashing às vezes quando o rs_pathl == 5
	double v_step;
	std::vector<carmen_robot_and_trailers_traj_point_t> rs_path_nodes;

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	reed_shepp(current, goal_state, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);

	for (int i = rs_pathl; i > 0; i--)
	{
		carmen_robot_and_trailers_traj_point_t point = rs_points[i];
		carmen_robot_and_trailers_traj_point_t next_point = rs_points[i - 1];
		if (point.v < 0.0)
			v_step = EXPAND_NODES_V;
		else
			v_step = -EXPAND_NODES_V;

		point.v = v_step;
		while ((DIST2D(point, next_point) > obstacle_distance_grid_map->config.resolution) ||
			   (fabs(carmen_radians_to_degrees(carmen_normalize_theta(point.theta - next_point.theta))) > 3.0))
		{
			double distance_traveled = 0.0;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, point.phi, 0.1, &distance_traveled, DELTA_T,
														  robot_config, semi_trailer_config);
			carmen_robot_and_trailers_traj_point_t new_state = point;
			// Como o Reed Shepp realiza o caminho do goal para um ponto, ele está andando de ré. Por isso precisa-se inverter o sinal de v
			new_state.v = -new_state.v;
			new_state.theta = carmen_normalize_theta(new_state.theta);

			rs_path_nodes.push_back(new_state);
		}
	}

	if (rs_path_nodes.size() == 0)
		return (rs_path_nodes);

	std::reverse(rs_path_nodes.begin(), rs_path_nodes.end());

	//	double ant_direction = rs_path_nodes[0].v;

	rs_path_nodes[0].trailer_theta[0] = current.trailer_theta[0];
	//	printf("first beta = %f\n", carmen_radians_to_degrees(rs_path_nodes[0].beta));

	if (number_of_trailers_for_path_planning == 0)
	{
		// Find the first pose that achieves goal constraint conditions to avoid unecessary adjustments
		int pose_found = 0;
		carmen_robot_and_trailers_traj_point_t nearest_goal_pose;
		int last_relevant_index = -1;

		for (unsigned int i = 1; i < rs_path_nodes.size(); i++)
		{
			if (pose_found == 1)
			{
				double current_distance = DIST2D(goal_state, rs_path_nodes[i]);
				double previous_distance = DIST2D(goal_state, nearest_goal_pose);
				if (current_distance < previous_distance && (near_enough_multi_trailer(rs_path_nodes[i], goal_state) == true))
				{
					nearest_goal_pose = rs_path_nodes[i];
					last_relevant_index = i;
				}
				else
				{
					// printf("Last index found: %d from %d [%lf %lf %lf]\n", last_relevant_index, rs_path_nodes.size(), nearest_goal_pose.x, nearest_goal_pose.y, nearest_goal_pose.theta);
					break;
				}
			}

			if (near_enough_multi_trailer(rs_path_nodes[i], goal_state) == true)
			{
				pose_found = 1;
				nearest_goal_pose = rs_path_nodes[i];
			}
		}

		if (last_relevant_index >= 0)
		{
			rs_path_nodes.erase(rs_path_nodes.begin() + last_relevant_index, rs_path_nodes.begin() + rs_path_nodes.size());
		}
	}

	// Find the last 10 meters of the reed shepp path, to use smaller collision there
	double last_path_length = 0.0;
	unsigned int last_path_index = rs_path_nodes.size();
	for (unsigned int i = (rs_path_nodes.size() - 1); i > 0; i--)
	{
		last_path_length += DIST2D(rs_path_nodes[i], rs_path_nodes[i - 1]);
		last_path_index = i;
		if (last_path_length > 10.0) // Last 10 meters of the path
		{
			break;
		}
	}

	if (astar_config.max_reed_shepp_distance_backwards > 0.0)
	{
		double current_distance_backwards = astar_config.max_reed_shepp_distance_backwards;

		// if (curr_time > (ASTAR_TIMEOUT / 2.0))
		// {
		//     current_distance_backwards = current_distance_backwards * 2.0;
		// }

		double distance_in_backwards = 0.0;
		for (unsigned int i = 1; i < rs_path_nodes.size(); i++)
		{
			if (rs_path_nodes[i].v < 0.0)
			{
				distance_in_backwards += DIST2D(rs_path_nodes[i], rs_path_nodes[i - 1]);
				if (distance_in_backwards > current_distance_backwards) // 5 meters in backwards allowed
				{
					check_collision = 1;
					return (rs_path_nodes);
				}
			}
		}
	}

	// para encontrar o length de todos os segmentos e não permitir que exista um segmento muito pequeno por problemas do sistema de seguir um plano pequeno
	double min_segment_distance = length_since_change_of_direction;

	int change_of_directions = 0;
	if (sign(rs_path_nodes[0].v) != sign(current.v))
	{
		change_of_directions = 1;
	}

	for (unsigned int i = 1; i < rs_path_nodes.size(); i++)
	{
		min_segment_distance += DIST2D(rs_path_nodes[i], rs_path_nodes[i - 1]);

		if (sign(rs_path_nodes[i - 1].v) != sign(rs_path_nodes[i].v))
		{
			change_of_directions++;
		}

		if ((sign(rs_path_nodes[i - 1].v) != sign(rs_path_nodes[i].v)) || (i == (rs_path_nodes.size() - 1)))
		{

			// printf("segment: %lf :: %lf %lf\n", min_segment_distance, rs_path_nodes[i - 1].v, rs_path_nodes[i].v);

			// if (min_segment_distance < (MAX(2.0, (robot_config.distance_between_front_and_rear_axles * 2.0/3.0))))
			if (min_segment_distance < (astar_config.min_dist_motion_change) || ((astar_config.max_reed_shepp_change_direction > 0) && (change_of_directions > astar_config.max_reed_shepp_change_direction)))
			{
				// printf("discard path\n");
				check_collision = 1;
				return (rs_path_nodes);
			}
			else
			{
				min_segment_distance = 0.0;
			}
		}
	}

	double path_distance = 0.0;
	for (unsigned int i = 1; i < rs_path_nodes.size(); i++)
	{
		path_distance += DIST2D(rs_path_nodes[i - 1], rs_path_nodes[i]);

		if (param_offroad_max_reed_shepp_distance > 0.0 && path_distance > param_offroad_max_reed_shepp_distance)
		{
			check_collision = 1;
			return (rs_path_nodes);
		}

		if (number_of_trailers_for_path_planning != 0)
		{
			for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			{
				rs_path_nodes[i].trailer_theta[z] = carmen_normalize_theta(compute_semi_trailer_thetas(rs_path_nodes[i - 1], 0.1, robot_config, semi_trailer_config, z));
			}
		}

		//		printf("beta = %f\n", carmen_radians_to_degrees(rs_path_nodes[i].beta));

		double current_safe_distance = robot_config.model_predictive_planner_obstacles_safe_distance * 1.5;
		if (i > last_path_index)
		{
			current_safe_distance = robot_config.model_predictive_planner_obstacles_safe_distance;
		}

		if ((carmen_obstacle_avoider_car_distance_to_nearest_obstacle(rs_path_nodes[i], obstacle_distance_grid_map) < current_safe_distance))
		{
			if (astar_config.radius_circle_to_ignore_obstacles_from_final_goal > 0.0 && (i > last_path_index)) // Apenas considera o radius do final goal se estiver no segmneto final do reed_shepp
			{
				int pose_is_inside_radius_circle = check_inside_goal_radius(rs_path_nodes[i].x, rs_path_nodes[i].y, goal_state.x, goal_state.y, astar_config.radius_circle_to_ignore_obstacles_from_final_goal);
				if (pose_is_inside_radius_circle == 0)
				{
					check_collision = 1;
					return (rs_path_nodes);
				}
			}
			else
			{
				check_collision = 1;
				return (rs_path_nodes);
			}
		}

		//		ant_direction = rs_path_nodes[i].v;
	}

	return (rs_path_nodes);
}

double
reed_shepp_cost(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal)
{
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	carmen_robot_and_trailers_traj_point_t rs_points[6]; // Por alguma razão, com o valor 5 acontece stack smashing às vezes quando o rs_pathl == 5
	double v_step;
	double path_cost = 0.0;

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current, goal, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);
	for (int i = rs_pathl; i > 0; i--)
	{
		carmen_robot_and_trailers_traj_point_t point = rs_points[i];
		carmen_robot_and_trailers_traj_point_t next_point = rs_points[i - 1];
		if (point.v < 0.0)
		{
			v_step = EXPAND_NODES_V;
		}
		else
		{
			v_step = -EXPAND_NODES_V;
		}

		point.v = v_step;
		while ((DIST2D(point, next_point) > 0.1) ||
			   (fabs(carmen_radians_to_degrees(carmen_normalize_theta(point.theta - next_point.theta))) > 3.0))
		{
			double distance_traveled = 0.0;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, point.phi,
														  0.1, &distance_traveled, 0.01, robot_config, semi_trailer_config);
			path_cost += distance_traveled;
		}
	}

	return (rs_length);
}

state_node
change_pose_to_relative_coordinates(state_node *reference_pose, state_node *pose)
{
	g2o::SE2 reference_pose_se2(reference_pose->pose.x, reference_pose->pose.y, reference_pose->pose.theta);
	g2o::SE2 pose_se2(pose->pose.x, pose->pose.y, pose->pose.theta);
	g2o::SE2 pose_in_relative_coordinates_se2 = reference_pose_se2.inverse() * pose_se2;

	state_node pose_in_relative_coordinates = *pose;
	pose_in_relative_coordinates.pose.x = pose_in_relative_coordinates_se2[0];
	pose_in_relative_coordinates.pose.y = pose_in_relative_coordinates_se2[1];
	pose_in_relative_coordinates.pose.theta = pose_in_relative_coordinates_se2[2];

	return (pose_in_relative_coordinates);
}

void get_cost_map_trailer_indexes(int &int_x, int &int_y, int &int_beta_goal, int &int_theta, int &int_beta_origem, double &half_map,
								  state_node *goal_pose, state_node *current_pose)
{
	state_node pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, goal_pose);

	int_theta = get_cost_map_int_theta(carmen_normalize_theta(pose_in_relative_coordinates.pose.theta),
									   astar_config.precomputed_cost_trailer_theta_size);

	double x = pose_in_relative_coordinates.pose.x / astar_config.precomputed_cost_trailer_resolution;
	double y = pose_in_relative_coordinates.pose.y / astar_config.precomputed_cost_trailer_resolution;

	half_map = (astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution) / 2.0;

	int_x = (int)(round(x + half_map));
	int_y = (int)(round(y + half_map));

	int_beta_goal = get_astar_map_beta(goal_pose->pose.theta, goal_pose->pose.trailer_theta[0], astar_config.precomputed_cost_trailer_beta_size);
	int_beta_origem = get_astar_map_beta(current_pose->pose.theta, current_pose->pose.trailer_theta[0], astar_config.precomputed_cost_trailer_beta_size);
}

void get_nonholonomic_truck_and_trailer_costs(double &nh_f, double &nh_b, state_node *goal_pose, state_node *current_pose)
{
	int int_x, int_y, int_beta_goal, int_theta, int_beta_origem;
	double half_map;
	get_cost_map_trailer_indexes(int_x, int_y, int_beta_goal, int_theta, int_beta_origem, half_map, goal_pose, current_pose);

	if (int_x >= 0 && int_y >= 0 && int_x < 2.0 * half_map && int_y < 2.0 * half_map)
	{
		int temp_index = get_trailer_table_map_index(int_x, int_y, int_theta, int_beta_goal, int_beta_origem);
		nh_f = cost_map_trailer[temp_index].h_forward;
		nh_b = cost_map_trailer[temp_index].h_backward;
	}
}

void get_nonholonomic_truck_and_trailer_valid(bool &nh_fv, bool &nh_bv, state_node *goal_pose, state_node *current_pose)
{
	int int_x, int_y, int_beta_goal, int_theta, int_beta_origem;
	double half_map;
	get_cost_map_trailer_indexes(int_x, int_y, int_beta_goal, int_theta, int_beta_origem, half_map, goal_pose, current_pose);

	if (int_x >= 0 && int_y >= 0 && int_x < 2.0 * half_map && int_y < 2.0 * half_map)
	{
		int temp_index = get_trailer_table_map_index(int_x, int_y, int_theta, int_beta_goal, int_beta_origem);
		nh_fv = cost_map_trailer[temp_index].h_forward_valid;
		nh_bv = cost_map_trailer[temp_index].h_backward_valid;
	}
}

double
get_nonholonomic_cost(double nh, state_node *goal_pose, state_node *current_pose, double *nonholonomic_heuristic_cost_map)
{
	state_node pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, goal_pose);

	int int_theta = get_cost_map_int_theta(pose_in_relative_coordinates.pose.theta,
										   astar_config.precomputed_cost_theta_size);

	double x = pose_in_relative_coordinates.pose.x / astar_config.precomputed_cost_resolution;
	double y = pose_in_relative_coordinates.pose.y / astar_config.precomputed_cost_resolution;

	double half_map = (astar_config.precomputed_cost_size / astar_config.precomputed_cost_resolution) / 2.0;

	int int_x = (int)(round(x + half_map));
	int int_y = (int)(round(y + half_map));

	if (int_x >= 0 && int_y >= 0 && int_x < 2.0 * half_map && int_y < 2.0 * half_map)
	{
		int temp_index = get_non_holonomic_car_index(int_x, int_y, int_theta);
		nh = nonholonomic_heuristic_cost_map[temp_index];
	}
	return (nh);
}

double
calculate_continuous_theta(int z, int precomputed_cost_theta_size)
{
	// precomputed_cost_theta_size_trailer tem que ser impar.
	double theta = (double)z * ((2.0 * M_PI) / (double)precomputed_cost_theta_size);
	theta = carmen_normalize_theta(theta) / 2.0;

	return (theta);
}

double
h(state_node *current_pose, state_node *goal_pose, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
  double *goal_distance_map, double *nonholonomic_heuristic_cost_map)
{
	index_node current_index;
	get_current_pos_in_grid_state_map(current_pose, current_index, obstacle_distance_grid_map);

	double ho = goal_distance_map[current_index.y + current_index.x * grid_state_map_y_size];

	double nh = -10.0;	 // @@@ Alberto: Por que nao incializar com zero?
	double nh_f = -10.0; // @@@ Alberto: Por que nao incializar com zero?
	double nh_b = -10.0; // @@@ Alberto: Por que nao incializar com zero?

	if (astar_config.use_matrix_cost_heuristic && nonholonomic_heuristic_cost_map_loaded &&
		((number_of_trailers_for_path_planning == 0))) //||
													   //		 (*behavior_selector_task == BEHAVIOR_SELECTOR_PARK) ||
													   //		 (*behavior_selector_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)))
	{
		nh = get_nonholonomic_cost(nh, goal_pose, current_pose, nonholonomic_heuristic_cost_map);
	}
	else if (astar_config.use_matrix_cost_trailer_heuristic && nonholonomic_heuristic_cost_map_trailer_loaded && number_of_trailers_for_path_planning > 0)
	{
		get_nonholonomic_truck_and_trailer_costs(nh_f, nh_b, goal_pose, current_pose);

		if ((nh_b != -1.0) && (nh_f != -1.0))
		{
			if (nh_b < nh_f)
				nh = nh_b;
			else
				nh = nh_f;
		}
		else if (nh_b != -1.0)
			nh = nh_b;
		else
			nh = nh_f;

		if ((nh < 0.0) || (nh > (2000.0 + ho)))
			nh = 2000.0 + ho;
		//		else
		//			nh *= 0.5;

		if (astar_config.use_matrix_cost_trailer_heuristic && nonholonomic_heuristic_cost_map_trailer_loaded && nh < 0.0) // Não há caminho encontrado pela expansão analítica do trailer
			nh = 100.0 + ho;																							  // Um custo alto somado com o custo holonomic para guiar o algoritmo mesmo se todas as expansões resultem em caminhos não encontrados pela expansão analítica do trailer
	}
	// else
	// {
	//     nh = reed_shepp_cost(current_pose->pose, goal_pose->pose);
	// }
	// Teste de simetria com o cálculo do theta
#if 0
	for (int z = 0; z < astar_config.precomputed_cost_theta_size; z++)
	{
		double continuous = carmen_normalize_theta(calculate_continuous_theta(z, astar_config.precomputed_cost_theta_size));
		int int_theta = get_cost_map_int_theta(continuous,	astar_config.precomputed_cost_theta_size);
		printf("z = %d, continuous = %f, int_theta = %d\n", z, continuous, int_theta);
	}
	exit(1);
#endif

	// printf("nh = %f ho = %f\n", nh, ho);

	// if (DIST2D(current_pose->pose, goal_pose->pose) > 25.0)
	// {
	// return (ho + (fabs(THETA_DIFF(current_pose->pose, goal_pose->pose)) * 2 * M_PI));
	// }

	return (std::max(nh, ho));
}

carmen_robot_and_trailers_traj_point_t
convert_state_node_to_trajectory_pose(state_node *n)
{
	carmen_robot_and_trailers_traj_point_t trajectory_pose = {n->pose.x, n->pose.y, n->pose.theta, n->pose.num_trailers, {0.0}, n->pose.v, n->pose.phi};
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		trajectory_pose.trailer_theta[z] = n->pose.trailer_theta[z];

	return trajectory_pose;
}

bool check_collision(vector<carmen_robot_and_trailers_traj_point_t> trajectory)
{
	for (unsigned int i = 1; i < trajectory.size(); i++)
	{
		if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory[i], obstacle_distance_grid_map) < robot_config.model_predictive_planner_obstacles_safe_distance)
			return (true);
	}

	return (false);
}

carmen_robot_and_trailers_traj_point_t
carmen_path_planner_astar_ackerman_kinematic(carmen_robot_and_trailers_traj_point_t point, double phi, double v)
{
	double full_time_interval = 1.0;
	double delta_t = 0.05;
	double distance_traveled;

	point = carmen_libcarmodel_recalc_pos_ackerman(point, v, phi, full_time_interval, &distance_traveled, delta_t,
												  robot_config, semi_trailer_config);

	return (point);
}

typedef struct rrt_state
{
	carmen_robot_and_trailers_traj_point_t pose;
	rrt_state *parent;
	int num_adjacent;
	rrt_state **adjacent;
	double cost;
	int expanded_since_start;
	int direction_changes;
	int failed_expansion;
	int success_expansion;
} rrt_state;

#define MAX_RRT_ADJACENT 10

void initialize_rrt_state(rrt_state *current_state)
{
	current_state->pose = {};
	current_state->parent = NULL;
	current_state->num_adjacent = 0;
	current_state->adjacent = (rrt_state **)malloc(MAX_RRT_ADJACENT * sizeof(rrt_state *));
	current_state->cost = 0.0;
	current_state->expanded_since_start = 0;
	current_state->direction_changes = 0;
	current_state->failed_expansion = 0;
	current_state->success_expansion = 0;
}

rrt_state *
rrt_expansion(rrt_state *current_node, carmen_point_t direction_point)
{
	int phi_possible_states = 5;
	int v_possible_states = 2;

	double target_phi[phi_possible_states] = {robot_config.max_phi, -robot_config.max_phi, 0.0, robot_config.max_phi / 2.0, -robot_config.max_phi / 2.0};
	double target_v[v_possible_states] = {-1.0, -1.0};

	//    int i = carmen_int_random(v_possible_states);
	//    int j = carmen_int_random(phi_possible_states);

	double min_distance = DBL_MAX;

	rrt_state *chosen_state;
	chosen_state = NULL;
	for (int i = 0; i < v_possible_states; i++)
	{
		for (int j = 0; j < phi_possible_states; j++)
		{
			rrt_state *new_state = (rrt_state *)malloc(sizeof(state_node));
			carmen_test_alloc(new_state);
			initialize_rrt_state(new_state);
			new_state->pose = carmen_path_planner_astar_ackerman_kinematic(current_node->pose, target_phi[j], target_v[i]);
			new_state->pose.v = target_v[i];

			if (trajectory_pose_hit_obstacle(new_state->pose, robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config))
			{
				free(new_state->adjacent);
				new_state->adjacent = NULL;
				free(new_state);
				new_state = NULL;
				continue;
			}

			int exit_loop = 0;
			for (int z = 0; z < number_of_trailers_for_path_planning; z++)
			{
				if (z == 0)
				{
					if (fabs(convert_theta1_to_beta(new_state->pose.theta, new_state->pose.trailer_theta[z])) > semi_trailer_config.semi_trailers[z].max_beta * 0.9)
					{
						exit_loop = 1;
						break;
					}
				}
				else
				{
					if (fabs(convert_theta1_to_beta(new_state->pose.trailer_theta[z - 1], new_state->pose.trailer_theta[z])) > semi_trailer_config.semi_trailers[z].max_beta * 0.9)
					{
						exit_loop = 1;
						break;
					}
				}
			}

			if (exit_loop == 1)
			{
				free(new_state->adjacent);
				new_state->adjacent = NULL;
				free(new_state);
				new_state = NULL;
				continue;
			}

			double current_distance = DIST2D(new_state->pose, direction_point);
			if (current_distance < min_distance)
			{
				chosen_state = new_state;
				min_distance = current_distance;
			}
			else
			{
				free(new_state);
			}
		}
	}

	if (chosen_state != NULL)
	{
		current_node->success_expansion++;
		return (chosen_state);
	}

	current_node->failed_expansion++;
	return (NULL);
}

rrt_state *
get_nearest_node_to_point(std::vector<rrt_state *> all_nodes, carmen_point_t random_point)
{
	double min_distance = DBL_MAX;
	int min_distance_index = -1;
	for (unsigned int i = 0; i < all_nodes.size(); i++)
	{
		double current_distance = DIST2D(all_nodes[i]->pose, random_point);
		if (all_nodes[i]->num_adjacent >= MAX_RRT_ADJACENT)
		{
			continue;
		}

		if (all_nodes[i]->direction_changes > 4)
		{
			continue;
		}
		if (all_nodes[i]->cost > 40.0)
		{
			continue;
		}

		if (all_nodes[i]->failed_expansion > 5 /*|| all_nodes[i]->success_expansion > 5*/)
		{
			continue;
		}

		if (current_distance < min_distance)
		{
			min_distance = current_distance;
			min_distance_index = i;
		}
	}

	if (min_distance_index > -1)
	{
		return (all_nodes[min_distance_index]);
	}

	return (NULL);
}

#define MAX_VIRTUAL_LASER_SAMPLES 1000000

vector<carmen_robot_and_trailers_traj_point_t>
rrt_trailer_expansion(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose)
{

	std::vector<carmen_robot_and_trailers_traj_point_t> path = {};
	rrt_state *first_node = (rrt_state *)malloc(sizeof(rrt_state));
	initialize_rrt_state(first_node);
	first_node->pose = current_pose;

	std::vector<rrt_state *> all_nodes = {first_node};
	double max_rand = 150.0;
	double min_rand = -149.0; // Para nao dividir por 0;
	carmen_point_t random_point;
	random_point.x = first_node->pose.x + carmen_uniform_random(min_rand, max_rand);
	random_point.y = first_node->pose.y + carmen_uniform_random(min_rand, max_rand);

	rrt_state *current_node = get_nearest_node_to_point(all_nodes, random_point);

	int goal_found = 0;
	while (goal_found == 0)
	{

		if (current_node->num_adjacent < MAX_RRT_ADJACENT)
		{
			publish_larger_laser_point(random_point.x, random_point.y, 0.3, CARMEN_RED);
			rrt_state *n = rrt_expansion(current_node, random_point);
			if (n != NULL)
			{
				current_node->adjacent[current_node->num_adjacent] = n;
				current_node->num_adjacent++;
				n->parent = current_node;
				n->expanded_since_start = current_node->expanded_since_start + 1;
				n->cost = current_node->cost + DIST2D(n->pose, current_node->pose);
				n->num_adjacent = 0;
				n->direction_changes = current_node->direction_changes;
				n->failed_expansion = 0;
				n->success_expansion = 0;

				if (n->pose.v != n->parent->pose.v)
				{
					n->direction_changes++;
				}

				rrt_state *t;
				t = n;

				while (t != NULL)
				{
					publish_larger_laser_point(t->pose.x, t->pose.y, 0.1, CARMEN_BLUE);
					//                	printf("full_path: %lf %lf\n", t->pose.x, t->pose.y);
					t = t->parent;
				}

				publish_simulated_objects();
				//                printf("Point: %lf %lf\n", n->pose.x, n->pose.y);
				all_nodes.push_back(n);

				if ((DIST2D(n->pose, goal_pose) < 0.5) && (fabs(carmen_normalize_theta(n->pose.theta - goal_pose.theta) < astar_config.goal_constraints.goal_achieved_theta)))
				{
					printf("Path found\n");
					while (n != NULL)
					{
						carmen_robot_and_trailers_traj_point_t current;
						current = n->pose;
						path.push_back(current);
						n = n->parent;
					}

					for (unsigned int i = 0; i < all_nodes.size(); i++)
					{
						free(all_nodes[i]->adjacent);
						all_nodes[i]->adjacent = NULL;
						free(all_nodes[i]);
						all_nodes[i] = NULL;
					}

					break;
				}
			}
		}

		int bias = carmen_int_random(100);
		if (bias <= 50)
		{
			random_point.x = goal_pose.x;
			random_point.y = goal_pose.y;
		}
		else
		{
			random_point.x = current_node->pose.x + carmen_uniform_random(min_rand, max_rand);
			random_point.y = current_node->pose.y + carmen_uniform_random(min_rand, max_rand);
		}
		current_node = get_nearest_node_to_point(all_nodes, random_point);

		if (current_node == NULL)
		{
			carmen_die("current_node is null, failure\n");
		}

		//        printf("rand: %lf %lf: %lf %lf %d %lf %d\n", random_point.x, random_point.y, current_node->pose.x, current_node->pose.y, current_node->num_adjacent, DIST2D(current_node->pose, goal_pose), all_nodes.size());
	}

	//    printf("Over\n");

	std::reverse(path.begin(), path.end());
	return (path);
}

bool analytic_expansion_with_trailer(state_node **n, state_node *goal_node)
{
	// memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	// virtual_laser_message.positions = (carmen_position_t *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	// virtual_laser_message.colors = (char *) calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	// virtual_laser_message.host = carmen_get_host();
	// virtual_laser_message.num_positions = 0;

#ifdef USE_VALID_CELLS_IN_NONHOLONOMIC_COST_TABLE_ONLY
	bool nh_fv, nh_bv;
	get_nonholonomic_truck_and_trailer_valid(nh_fv, nh_bv, goal_node, *n);
	bool valid_entry_in_table = false;
	if ((((*n)->pose.v > 0.0) && nh_fv) ||
		(((*n)->pose.v < 0.0) && nh_bv))
		valid_entry_in_table = true;
	if (!valid_entry_in_table)
		return (false);
#endif

	//	double nh_f, nh_b;
	//	get_nonholonomic_truck_and_trailer_costs(nh_f, nh_b, goal_node, *n);
	//	bool valid_entry_in_table = false;
	//	if ((((*n)->pose.r == Forward) && (nh_f > 0.0) && (nh_f < 10000.0)) ||
	//		(((*n)->pose.r == Backward) && (nh_b > 0.0) && (nh_b < 10000.0)))
	//		valid_entry_in_table = true;
	//	if (!valid_entry_in_table)
	//		return (false);

	vector<carmen_robot_and_trailers_traj_point_t> expanded_path;
	pair<bool, vector<carmen_robot_and_trailers_traj_point_t>> analytical_expansion_result;

	analytical_expansion_result = trailer_nlp_analytical_expansion((*n)->pose, goal_node->pose, robot_config, semi_trailer_config, obstacle_distance_grid_map, max_phi_multiplier, true, false, astar_config.goal_constraints, semi_trailer_solver_type);
	expanded_path = analytical_expansion_result.second;
	// expanded_path = rrt_trailer_expansion((*n)->pose, goal_node->pose);

	if ((analytical_expansion_result.first) && (expanded_path.size() > 0))
	{
		state_node_p ant_state = (state_node_p)malloc(sizeof(state_node));
		ant_state = (*n);

		for (unsigned int i = 0; i < expanded_path.size(); i++)
		{
			state_node_p current_state = (state_node_p)malloc(sizeof(state_node));

			current_state->pose.x = expanded_path[i].x;
			current_state->pose.y = expanded_path[i].y;
			current_state->pose.theta = expanded_path[i].theta;
			//			for (size_t ii = 0; ii < MAX_NUM_TRAILERS; ii++)
			//				current_state->pose.trailer_theta[ii] = expanded_path[i].trailer_theta[ii];
			memcpy(current_state->pose.trailer_theta, expanded_path[i].trailer_theta, sizeof(expanded_path[i].trailer_theta));

			current_state->pose.phi = expanded_path[i].phi;
			current_state->pose.v = expanded_path[i].v;
			current_state->parent = ant_state;

			*n = current_state;
			ant_state = current_state;

			//			printf("[] %lf %lf %lf %lf %lf\n", current_state->pose.x, current_state->pose.y, current_state->pose.theta, current_state->pose.trailer_theta[0], current_state->pose.trailer_theta[1]);
		}
		// publish_simulated_objects();
		return (true);
	}
	else
	{

#ifdef DRAW_EXPANSION_TREE
		for (size_t i = 1; i < expanded_path.size(); i++)
			draw_point_in_opencv_image(expanded_path[i], expanded_path[i - 1], obstacle_distance_grid_map_print_map->config, cv::Scalar(160, 160, 160), 1);
#endif

		return (false);
	}
}

bool analytic_expansion_without_trailer(state_node **n, state_node *goal_node)
{

#ifndef USE_REED_SHEPP_AS_ANALYTICAL_EXPANSION
	return (false);
#endif

	if (astar_config.use_reed_shepp == 0)
	{
		return (false);
	}

	int check_collision = 0;
	std::vector<carmen_robot_and_trailers_traj_point_t> rs_path = reed_shepp_path((*n)->pose, goal_node->pose, check_collision, obstacle_distance_grid_map, (*n)->it_since_change_in_direction * EXPAND_NODES_V);

#if SEND_VIRTUAL_LASER
	for (unsigned int i = 0; i < rs_path.size(); i++)
	{
		if (check_collision == 0)
		{
			publish_larger_laser_point(rs_path[i].x, rs_path[i].y, 0.1, CARMEN_HOTPINK);
		}
		else
		{
			publish_larger_laser_point(rs_path[i].x, rs_path[i].y, 0.1, CARMEN_RED);
		}
	}
#endif

	if ((rs_path.size() > 0) && (check_collision == 0) /*&& ((sign(rs_path[0].v) == sign((*n)->pose.v)))*/)
	{
		//            double distance_path = 0.0;
		state_node_p ant_state = (state_node_p)malloc(sizeof(state_node));
		ant_state = (*n);

		for (unsigned int i = 0; i < rs_path.size(); i++)
		{
			state_node_p current_state = (state_node_p)malloc(sizeof(state_node));

			current_state->pose.x = rs_path[i].x;
			current_state->pose.y = rs_path[i].y;
			current_state->pose.theta = rs_path[i].theta;
			for (size_t ii = 0; ii < MAX_NUM_TRAILERS; ii++)
				current_state->pose.trailer_theta[ii] = rs_path[i].trailer_theta[ii];

			current_state->pose.phi = rs_path[i].phi;
			current_state->pose.v = rs_path[i].v;
			current_state->parent = ant_state;

			*n = current_state;
			ant_state = current_state;
		}
		// printf("distance_path: %lf\n", distance_path);
		// for (unsigned int i = 0; i < rs_path.size(); i++)
		// {
		//     printf("rs[%d]: %lf %lf %lf %lf\n", i, rs_path[i].x, rs_path[i].y, rs_path[i].theta, rs_path[i].v);
		// }

		return (true);
	}
	else
		return (false);
}

bool analytic_expansion(state_node **n, state_node *goal_node)
{
	if ((number_of_trailers_for_path_planning == 0) || (astar_config.use_rs_with_trailer) || (*behavior_selector_task == BEHAVIOR_SELECTOR_PARK))
	{
		bool aewt_condition = false;
		aewt_condition = analytic_expansion_without_trailer(n, goal_node);

		// O código abaixo servia para aplicar um ruído no reed_shepp, mas não teve melhoras significativas. Apesar disso, mantive o código abaixo comentado apenas para caso seja possível evoluir em algum momento
		/*
		if (aewt_condition == false)
		{
			double maximum_distance_noise = astar_config.goal_constraints.goal_achieved_distance / 2.0;
			double maximum_theta_noise = astar_config.goal_constraints.goal_achieved_theta / 2.0;
			int index_distance_noise = (int) (maximum_distance_noise / 0.2);
			int index_theta_noise = (int) (maximum_theta_noise / 0.2);

			for (int x = -index_distance_noise; x < index_distance_noise; x++)
			{
				for (int y = -index_distance_noise; y < index_distance_noise; y++)
				{
					for (int theta = -index_theta_noise; theta < index_theta_noise; theta++)
					{
						// Cria uma cópia temporária de goal_node
						state_node temp_goal_node = *goal_node;

						// Adiciona ruído às coordenadas e ao ângulo da cópia
						temp_goal_node.pose.x += x * 0.1; // 0.1 é o incremento para o ruído
						temp_goal_node.pose.y += y * 0.1; // Ajuste conforme necessário
						temp_goal_node.pose.theta += theta * 0.1; // Ajuste o incremento para o ruído de theta

						// Agora chame analytic_expansion_without_trailer com temp_goal_node
						bool result = analytic_expansion_without_trailer(n, &temp_goal_node);

						// Se o resultado for verdadeiro, você pode interromper o loop aqui
						if (result == true)
						{
							return (true);
						}

					}

				}

			}

		}
*/
		return (aewt_condition);
	}
	else
	{
		return (analytic_expansion_with_trailer(n, goal_node));
	}
}

/*
bool
is_goal(state_node **current_node, state_node *goal_node)
{
	double min_distance_to_try_analytic_expansion;

	if (semi_trailer_config.num_semi_trailers == 0)
	{
		min_distance_to_try_analytic_expansion = 4.5 * robot_config.length;
		for (size_t ii = 0; ii < MAX_NUM_TRAILERS; ii++)
		{
			(*current_node)->pose.trailer_theta[ii] = 0.0;
			goal_node->pose.trailer_theta[ii] 		= 0.0;
		}
	}
	else
		min_distance_to_try_analytic_expansion = 3.0 * (robot_config.length + semi_trailer_config.semi_trailers[0].M + semi_trailer_config.semi_trailers[0].d);
//
//	if (near_enough(DIST2D((*current_node)->pose, goal_node->pose),
//					THETA_DIFF((*current_node)->pose, goal_node->pose),
//					BETA_DIFF((*current_node)->pose, goal_node->pose)))
	if(near_enough_multi_trailer((*current_node)->pose, goal_node->pose))
		return (true);
	else if (DIST2D((*current_node)->pose, goal_node->pose) < min_distance_to_try_analytic_expansion)
		return (analytic_expansion(current_node, goal_node));
	else
		return (false);
}
*/

bool is_goal(state_node **current_node, state_node *goal_node)
{
	double min_distance_to_try_analytic_expansion;
	min_distance_to_try_analytic_expansion = (number_of_trailers_for_path_planning == 0) ? 4.5 * robot_config.length : 3.0 * (robot_config.length + semi_trailer_config.semi_trailers[0].M + semi_trailer_config.semi_trailers[0].d);

	if (near_enough_multi_trailer((*current_node)->pose, goal_node->pose))
	{
		return (true);
	}
	else if (DIST2D((*current_node)->pose, goal_node->pose) < min_distance_to_try_analytic_expansion)
	{
		return (analytic_expansion(current_node, goal_node));
	}

	return (false);
}

double
movement_cost(state_node *current_node, state_node *new_node)
{
	return (DIST2D(current_node->pose, new_node->pose));
}

double
penalties(state_node *current_node, state_node *goal_node __attribute__((unused)), carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	//	carmen_robot_and_trailers_pose_t trajectory_pose;
	//	trajectory_pose.x = current_node->pose.x;
	//	trajectory_pose.y = current_node->pose.y;
	//	trajectory_pose.theta = current_node->pose.theta;
	//	for (size_t ii = 0; ii < MAX_NUM_TRAILERS; ii++)
	//		trajectory_pose.trailer_theta[ii] = current_node->pose.trailer_theta[ii];

	//	carmen_robot_and_trailers_pose_t not_used = {};
	// double obstacle_cost = carmen_obstacle_avoider_proximity_to_obstacles(&trajectory_pose, not_used, obstacle_distance_grid_map, robot_config.model_predictive_planner_obstacles_safe_distance);
	carmen_robot_and_trailers_traj_point_t cp = {current_node->pose.x, current_node->pose.y, current_node->pose.theta, 0, {0.0}, 0.0, 0.0};
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
	{
		cp.trailer_theta[z] = current_node->pose.trailer_theta[z];
	}

	double obstacle_cost = carmen_obstacle_avoider_car_distance_to_nearest_obstacle(cp, obstacle_distance_grid_map);
	if (obstacle_cost < robot_config.model_predictive_planner_obstacles_safe_distance)
	{
		obstacle_cost = 1000.0; // A collision happened here
	}
	else
	{
		obstacle_cost = 1.0 / obstacle_cost; // bigger distance should return a smaller cost
	}

	// double distance_to_lane = astar_config.penalties_w4 * std::min(astar_config.min_distance_to_lane, get_distance_to_nearest_lane(current_road_network_message, current_node->pose));
	// if (distance_to_lane > 8.0)
	// 	distance_to_lane = 8.0;

	int direction = current_node->pose.v < 0.0 ? 1 : 0; // 1 if is backwards, in which it enters w1 penalties
														// int old_direction = current_node->parent->pose.v < 0.0 ? 1 : 0;

	// New treatment for previous_direction; 1 for backwards, 0 for forward; -1 for pose 0.0 (initial pose)

	int old_direction = -1;
	if (current_node->parent->pose.v < 0.0)
	{
		old_direction = 1;
	}
	else if (current_node->parent->pose.v > 0.0)
	{
		old_direction = 0;
	}

	// if (obstacle_cost != 0.0)
	// {
	// printf("at %lf %lf %lf %lf\n", current_node->pose.x, current_node->pose.y, current_node->pose.theta, current_node->pose.phi);
	// printf("costs: %lf %lf %lf\n", astar_config.penalties_w5, (double) (current_node->pose.phi != 0.0), astar_config.penalties_w5 * (double) (current_node->pose.phi != 0.0));
	// }

	double reverse_movement_penalty = astar_config.penalties_w1;
	double change_direction_penalty = astar_config.penalties_w2;
	double obstacle_proximity_penalty = astar_config.penalties_w3;
	// double phi_penalty = astar_config.penalties_w5; // está completamente instável, gera efeito "estrela" na mosaic

	if (old_direction == -1)
	{
		change_direction_penalty = 0.0; // Zera porque a pose anterior é a pose inicial
	}

	double previous_movement_cost = movement_cost(current_node, current_node->parent);
	if (direction == 1)
	{
		previous_movement_cost = previous_movement_cost * reverse_movement_penalty;
	}

	// if (DIST2D(current_node->pose, goal_node->pose) < 15.0 && (fabs(THETA_DIFF(current_node->pose, goal_node->pose))) < carmen_degrees_to_radians(90.0))
	// {
	//     reverse_movement_penalty = 0.0;
	//     // change_direction_penalty = 0.0;
	//     obstacle_proximity_penalty = 0.0;
	// }

	return (previous_movement_cost +
			obstacle_proximity_penalty * obstacle_cost +
			change_direction_penalty * (double)(direction != old_direction) /*+
		   phi_penalty * (current_node->pose.phi != 0.0)*/
	);

	//	return (30.0 * obstacle_cost +
	//			PENALTIES_W1 * current_node->pose.r * movement_cost(current_node, current_node->parent) +
	//			PENALTIES_W2 * (current_node->pose.r != current_node->parent->pose.r));
}

//
// int
// is_valid_state(state_node *state, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
//{
//	if (state->pose.x < obstacle_distance_grid_map->config.x_origin || state->pose.y < obstacle_distance_grid_map->config.y_origin)
//		return 0;
//
//	index_node current_index;
//	get_current_pos_in_grid_state_map(state, current_index, obstacle_distance_grid_map);
//
//	if (current_index.x >= grid_state_map_x_size || current_index.y >= grid_state_map_y_size || current_index.x < 0 || current_index.y < 0 ||
//		trajectory_pose_hit_obstacle(state->pose, robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config) ||
//		((semi_trailer_config.num_semi_trailers > 0) && fabs(convert_theta1_to_beta(state->pose.theta, state->pose.trailer_theta[0])) > semi_trailer_config.semi_trailers[0].max_beta * 0.9)) // Impedir a criação de estados com o beta invalido
//	{
//		return (0);
//	}
//
//	return (1);
//}

int is_valid_state(state_node *state, state_node *goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	if (state->pose.x < obstacle_distance_grid_map->config.x_origin || state->pose.y < obstacle_distance_grid_map->config.y_origin)
	{
		return 0;
	}

	index_node current_index;
	get_current_pos_in_grid_state_map(state, current_index, obstacle_distance_grid_map);

	if (current_index.x >= grid_state_map_x_size || current_index.y >= grid_state_map_y_size || current_index.x < 0 || current_index.y < 0)
	{
		return (0);
	}

	if (trajectory_pose_hit_obstacle(state->pose, robot_config.model_predictive_planner_obstacles_safe_distance, obstacle_distance_grid_map, &robot_config))
	{
		// Desativado, pois essa checagem é feita no reed shepp, mas deixei aqui caso seja necessário no futuro

		if (astar_config.radius_circle_to_ignore_obstacles_from_final_goal > 0.0)
		{
			int pose_is_inside_radius_circle = check_inside_goal_radius(state->pose.x, state->pose.y, goal_node->pose.x, goal_node->pose.y, astar_config.radius_circle_to_ignore_obstacles_from_final_goal);
			if (pose_is_inside_radius_circle == 0)
			{
				return (0);
			}
		}
		else
		{
			return (0);
		}
	}

	// Verificação de angulo entre a ultima pose reversa e a atual (quando troca de direção) para evitar manobras que causam mudanças mínimas de ângulo
	// pose_of_direction_changed

	// if (state->parent != NULL && (sign(state->parent->pose.v) != sign(state->pose.v)) && (sign(state->pose.v) >= 0))
	// {
	//     if (fabs(THETA_DIFF(state->pose, state->parent->pose_of_direction_changed)) <= carmen_degrees_to_radians(30.0))
	//     {
	//         // printf("Found condition! %lf %lf %lf %lf, %lf %lf %lf %lf = %lf\n", state->pose.x, state->pose.y, state->pose.theta, state->pose.v, state->parent->pose_of_direction_changed.x, state->parent->pose_of_direction_changed.y, state->parent->pose_of_direction_changed.theta, state->parent->pose_of_direction_changed.v, DIST2D(state->pose, state->parent->pose_of_direction_changed));
	//         return (0);
	//     }
	// }

	//

	for (int i = 0; i < number_of_trailers_for_path_planning; i++)
	{
		if (i == 0)
		{
			if (fabs(convert_theta1_to_beta(state->pose.theta, state->pose.trailer_theta[i])) > semi_trailer_config.semi_trailers[i].max_beta * 0.9)
			{
				return (0);
			}
		}
		else
		{
			if (fabs(convert_theta1_to_beta(state->pose.trailer_theta[i - 1], state->pose.trailer_theta[i])) > semi_trailer_config.semi_trailers[i].max_beta * 0.9)
			{
				return (0);
			}
		}
	}

	return (1);
}

void get_astar_path(state_node *n, std::vector<carmen_robot_and_trailers_traj_point_t> &path_result)
{
	if (n == NULL)
		return;

	carmen_robot_and_trailers_traj_point_t last_state;

	path_result.push_back(n->pose);
	last_state = n->pose;
	n = n->parent;

	while (n != NULL)
	{
		if ((DIST2D(n->pose, last_state) >= 0.1) || ((sign(n->pose.v) != sign(last_state.v))))
		{
			path_result.push_back(n->pose);
			last_state = n->pose;
		}
		n = n->parent;
	}

	//	 @Anderson: pra que serve isso? Está dando segfault. O loop acima só termina quando n for null, então a comparação de baixo é estranha
	// O path tem que incluir a initial_pose
	//	if (DIST2D(n->pose, path_result[path_result.size() - 1]) > 0.0)
	//		path_result.push_back(n->pose);

	std::reverse(path_result.begin(), path_result.end());
}

double
return_dist_to_obstacle_following_expansion(carmen_robot_and_trailers_traj_point_t pose, double v, double phi, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	static double max_radius = 2 * (robot_config.distance_between_front_and_rear_axles / (sin(robot_config.max_phi) * 2));

	double result = 0.0;
	carmen_robot_and_trailers_traj_point_t new_pose = pose;
	new_pose.v = v;
	new_pose.phi = phi;
	int state_to_check_non_obstacle = 0;

	while (1)
	{
		carmen_robot_and_trailers_traj_point_t previous_pose = new_pose;
		new_pose = carmen_path_planner_astar_ackerman_kinematic(new_pose, phi, v);
		result += DIST2D(new_pose, previous_pose);

		// printf("current in new_pose: %lf %lf %lf %lf for: %lf \n", new_pose.x, new_pose.y, new_pose.theta, carmen_obstacle_avoider_car_distance_to_nearest_obstacle(new_pose, obstacle_distance_grid_map), phi);
		// printf("current in previous_pose: %lf %lf %lf %lf for: %lf \n", previous_pose.x, previous_pose.y, previous_pose.theta, carmen_obstacle_avoider_car_distance_to_nearest_obstacle(previous_pose, obstacle_distance_grid_map), phi);

		if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(new_pose, obstacle_distance_grid_map) < robot_config.model_predictive_planner_obstacles_safe_distance)
		{
			publish_larger_laser_point(new_pose.x, new_pose.y, 0.1, CARMEN_GREEN);
			// printf("found in new_pose: %lf %lf %lf %lf\n", new_pose.x, new_pose.y, new_pose.theta, carmen_obstacle_avoider_car_distance_to_nearest_obstacle(new_pose, obstacle_distance_grid_map));
			return (result);
		}

		double dist_to_initial_pose = DIST2D(new_pose, pose);
		// printf("dist: %lf %lf %d\n", dist_to_initial_pose, result, state_to_check_non_obstacle);
		if (state_to_check_non_obstacle == 1 && ((dist_to_initial_pose < 0.3) || (dist_to_initial_pose > (max_radius + 0.1))))
		{
			return (-1.0); // Não encontrou obstáculos, a expansão deu uma volta ou é uma expansão reta
		}

		if (dist_to_initial_pose > 1.0)
		{
			state_to_check_non_obstacle = 1;
		}
	}

	return (result);
}

void check_backward_finisher_scenario(state_node *goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	// Expand the node from backwards
	int phi_possible_states = 3;
	int v_possible_states = 1;

	double target_phi[phi_possible_states] = {robot_config.max_phi, -robot_config.max_phi, 0.0};
	double target_v[v_possible_states] = {-EXPAND_NODES_V};
	int counter = 0;

	for (int i = 0; i < v_possible_states; i++)
	{
		for (int j = 0; j < phi_possible_states; j++)
		{
			double dist_to_obstacle = return_dist_to_obstacle_following_expansion(goal_node->pose, target_v[i], target_phi[j], obstacle_distance_grid_map);
			if (dist_to_obstacle >= 0.0 && (dist_to_obstacle < robot_config.distance_between_front_and_rear_axles))
			{
				counter++;
			}
		}
	}

	publish_simulated_objects();

	backwards_finisher_on = 0;
	if (counter >= phi_possible_states)
	{
		backwards_finisher_on = 1;
	}
}

carmen_offroad_planner_feedback_t
check_initial_nodes(state_node *initial_node, state_node *goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
#if PRINT_NODES
	printf("Initial_pose = %f %f %f\n", initial_node->pose.x, initial_node->pose.y, initial_node->pose.theta);
	printf("Goal_pose = %f %f %f\n", goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta);
#endif

	if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(initial_node->pose, obstacle_distance_grid_map) < robot_config.model_predictive_planner_obstacles_safe_distance)
	{
		printf("Robot_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(initial_node->pose, obstacle_distance_grid_map));
		return (START_OVER_OBSTACLE);
	}

	if (astar_config.radius_circle_to_ignore_obstacles_from_final_goal > 0.0)
	{
		// Don't check for collision to final goal because of the parameter that allows final goal to be inserted in obstacle
		return (PLAN_OK);
	}

	if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(goal_node->pose, obstacle_distance_grid_map) < robot_config.model_predictive_planner_obstacles_safe_distance)
	{
		printf("Goal_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(goal_node->pose, obstacle_distance_grid_map));
		return (GOAL_OVER_OBSTACLE);
	}

	return (PLAN_OK);
}

std::vector<state_node *>
expand_node(state_node *n, state_node *goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	std::vector<state_node *> neighbors;

	int phi_possible_states = 3;
	int v_possible_states = 2;

	neighbors.reserve(phi_possible_states * v_possible_states);

	double target_phi[phi_possible_states] = {robot_config.max_phi, -robot_config.max_phi, 0.0};
	double target_v[v_possible_states] = {EXPAND_NODES_V, -EXPAND_NODES_V};
	//    double target_v[1]   = {EXPAND_NODES_V};

	// printf("%lf %lf %lf\n", n->pose.x, n->pose.y, n->pose.theta);
	for (int i = 0; i < v_possible_states; i++)
	{
		for (int j = 0; j < phi_possible_states; j++)
		{
			state_node_p new_state = (state_node_p)malloc(sizeof(state_node));
			carmen_test_alloc(new_state);
			new_state->pose = carmen_path_planner_astar_ackerman_kinematic(n->pose, target_phi[j], target_v[i]);
			new_state->pose.v = target_v[i];
			new_state->parent = n;

			if (new_state->pose.v != n->pose.v)
			{
				new_state->it_since_change_in_direction = 1;
				new_state->pose_of_direction_changed = new_state->pose;
				new_state->number_of_change_in_direction = n->number_of_change_in_direction + 1;
			}
			else
			{
				new_state->number_of_change_in_direction = n->number_of_change_in_direction;
				new_state->it_since_change_in_direction = n->it_since_change_in_direction + 1;
				new_state->pose_of_direction_changed = n->pose_of_direction_changed;
			}

			if (is_valid_state(new_state, goal_node, obstacle_distance_grid_map) == 1 && ((astar_config.max_change_direction < 0) || (new_state->number_of_change_in_direction <= astar_config.max_change_direction)) &&
				((new_state->pose.v == n->pose.v) || (n->parent == NULL || ((new_state->pose.v != n->pose.v) && (n->it_since_change_in_direction * EXPAND_NODES_V) > astar_config.min_dist_motion_change))))
			{
				// printf("All conditions: %d %d %d %lf %lf\n", is_valid_state(new_state, goal_node, obstacle_distance_grid_map), new_state->number_of_change_in_direction, n->it_since_change_in_direction, n->it_since_change_in_direction * EXPAND_NODES_V, astar_config.min_dist_motion_change);
				// printf("%lf %lf %lf = %lf\n", new_state->pose.x, new_state->pose.y, new_state->pose.theta, DIST2D(new_state->pose, n->pose));
				neighbors.push_back(new_state);
			}
			else
			{
				// printf("Ald conditions: %d %d %d %lf %lf\n", is_valid_state(new_state, goal_node, obstacle_distance_grid_map), new_state->number_of_change_in_direction, n->it_since_change_in_direction, n->it_since_change_in_direction * EXPAND_NODES_V, astar_config.min_dist_motion_change);
				// printf("%lf %lf %lf = %lf\n", new_state->pose.x, new_state->pose.y, new_state->pose.theta, DIST2D(new_state->pose, n->pose));
				free(new_state);
			}
		}
	}

	return (neighbors);
}

void publish_intermediate_plan(state_node *n_original)
{
	if (n_original == NULL)
		return;

	state_node *n = n_original;

	std::vector<carmen_robot_and_trailers_traj_point_t> path_result;
	path_result.push_back(n->pose);

	carmen_robot_and_trailers_traj_point_t last_state = n->pose;
	n = n->parent;
	while (n != NULL)
	{
		if ((DIST2D(n->pose, last_state) >= 0.4) || (sign(n->pose.v) != sign(last_state.v)))
		{
			path_result.push_back(n->pose);
			last_state = n->pose;
		}
		n = n->parent;
	}

	// O path tem que incluir a initial_pose
	if (DIST2D(n->pose, path_result[path_result.size() - 1]) > 0.0)
		path_result.push_back(n->pose);

	std::reverse(path_result.begin(), path_result.end());

	if (path_result.size() > 1)
	{
		// Consertar o v 0.0 do initial pose e final pose
		path_result[0].v = path_result[1].v;
		path_result[path_result.size() - 1].v = path_result[path_result.size() - 2].v;

		offroad_planner_plan_t plan;
		plan.robot = path_result[0];
		plan.goal = path_result[path_result.size()];
		plan.goal_set = 1;

		plan.path.length = path_result.size();
		plan.path.capacity = plan.path.length;
		plan.path.points = (carmen_robot_and_trailers_traj_point_t *)malloc(plan.path.length * sizeof(carmen_robot_and_trailers_traj_point_t));
		for (int i = 0; i < plan.path.length; i++)
			plan.path.points[i] = path_result[i];

		// publish
		carmen_offroad_planner_plan_message plan_m;
		plan_m.offroad_planner_feedback = OFFROAD_PLANNER_TEST;
		plan_m.number_of_poses = plan.path.length;
		plan_m.poses = plan.path.points;
		plan_m.pose_id = -1;
		plan_m.transition_pose = path_result[path_result.size() - 1];
		if (plan.path.points)
			plan_m.goal_pose = plan.path.points[plan.path.length - 1];
		else
			plan_m.goal_pose = path_result[path_result.size() - 1];

		carmen_offroad_planner_publish_plan(&plan_m);
		free(plan.path.points);
	}
}

void remove_dead_branch(state_node **n)
{
	while ((n != NULL) && ((*n) != NULL) && (*n)->branches == 0)
	{
		state_node *temp_n;
		temp_n = (*n);
		(*n) = (*n)->parent;
		free(temp_n);
		if ((*n) != NULL)
			(*n)->branches--;
	}
}

char *
convert_offroad_request_to_string(offroad_planner_request_t v)
{
	switch (v)
	{
	case PLAN_FROM_POSE_TO_LANE:
		return (char *)"PLAN_FROM_POSE_TO_LANE";
	case PLAN_FROM_LANE_TO_FINAL_POSE:
		return (char *)"PLAN_FROM_LANE_TO_FINAL_POSE";
	case PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT:
		return (char *)"PLAN_FROM_LANE_TO_RECTLINEAR_ROUTE_SEGMENT";
	case PLAN_FROM_CURRENT_POSE_TO_RECTLINEAR_ROUTE_SEGMENT:
		return (char *)"PLAN_FROM_CURRENT_POSE_TO_RECTLINEAR_ROUTE_SEGMENT";
	case PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE:
		return (char *)"PLAN_FROM_CURRENT_POSE_TO_ENGAGE_POSE";
	case PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE:
		return (char *)"PLAN_FROM_CURRENT_POSE_TO_FINAL_POSE";
	default:
		return (char *)"AAAAAAAAA";
	}
}

int cont = 0;

std::vector<carmen_robot_and_trailers_traj_point_t>
carmen_path_planner_astar_search(carmen_offroad_planner_feedback_t &feedback,
								carmen_robot_and_trailers_traj_point_t *initial_pose, carmen_robot_and_trailers_traj_point_t *goal_pose,
								carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double *goal_distance_map,
								double *nonholonomic_heuristic_cost_map,
								carmen_route_planner_road_network_message *road_network_message = NULL)
{

	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.positions = (carmen_position_t *)calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *)calloc(MAX_VIRTUAL_LASER_SAMPLES, sizeof(char));
	virtual_laser_message.host = carmen_get_host();
	virtual_laser_message.num_positions = 0;

#ifdef DRAW_EXPANSION_TREE
	char *str_offroad_planner_request = convert_offroad_request_to_string(road_network_message->offroad_planner_request);
	draw_map(obstacle_distance_grid_map_print_map, map_image);
#endif

#ifdef DRAW_PATHS_AND_POSES
	char *str_offroad_planner_request_2 = convert_offroad_request_to_string(road_network_message->offroad_planner_request);
	char *str_offroad_planner_request_3 = convert_offroad_request_to_string(road_network_message->offroad_planner_request);
	draw_map(obstacle_distance_grid_map_print_map, map_image);
#endif
	double start_time = carmen_get_time();

	current_road_network_message = road_network_message;

	std::vector<carmen_robot_and_trailers_traj_point_t> path_result;

	grid_state_p grid_state_map = alloc_grid_state_map();
	state_node *initial_node = new_state_node(initial_pose);
	state_node *goal_node = new_state_node(goal_pose);
	//	boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> FH;

#ifdef _REMOVE_BOOST
	priority_queue<state_node *, vector<state_node *>, StateNodePtrComparator> FH;
	FH.push(initial_node);
#else
	boost::heap::d_ary_heap<state_node *, boost::heap::arity<boost_arity>, boost::heap::compare<StateNodePtrComparator>> FH;
	FH.push(initial_node);
#endif

	feedback = check_initial_nodes(initial_node, goal_node, obstacle_distance_grid_map);
	if (feedback != PLAN_OK)
	{
		clear_astar_search(FH, grid_state_map, goal_node);
		return {};
	}
	// check_backward_finisher_scenario(goal_node, obstacle_distance_grid_map);

	index_node current_index;
	get_current_pos_in_grid_state_map(initial_node, current_index, obstacle_distance_grid_map);
	int temp_index = get_grid_state_map_index(current_index);
	grid_state_map[temp_index].state = Open;
	grid_state_map[temp_index].g = 0.0;

	state_node *n;
	int cont_rs_nodes = 0;

	astar_timeout = time_mult ? (time_mult * ASTAR_TIMEOUT) : ASTAR_TIMEOUT;
	curr_time = carmen_get_time() - start_time;

	while ((!FH.empty()) && (curr_time < astar_timeout))
	{
		n = FH.top();
		FH.pop();

		get_current_pos_in_grid_state_map(n, current_index, obstacle_distance_grid_map);
		temp_index = get_grid_state_map_index(current_index);
		if (grid_state_map[temp_index].state != Closed)
		{
#if PRINT_NODES
			printf("Current_node = %f %f %f %f %f %f %f %f %f\n", n->pose.x, n->pose.y, n->pose.theta, n->pose.trailer_theta[0], n->pose.trailer_theta[1], n->pose.trailer_theta[2], n->pose.trailer_theta[3], n->pose.trailer_theta[4], n->pose.v);
			printf("node_cost: %lf %lf %lf\n", n->g, n->h, n->f);
			printf("costs: %d %d\n", n->number_of_change_in_direction, n->it_since_change_in_direction);
#endif

#if SEND_VIRTUAL_LASER

			state_node *n_temp = n;
			while (n_temp != NULL)
			{
				if (n_temp->pose.v < 0.0)
				{
					publish_larger_laser_point(n_temp->pose.x, n_temp->pose.y, 0.1, CARMEN_GREEN);
				}
				else
				{
					publish_larger_laser_point(n_temp->pose.x, n_temp->pose.y, 0.1, CARMEN_ROYALBLUE3);
				}
				n_temp = n_temp->parent;
			}
#endif
			// publish_larger_laser_point(n->pose.x, n->pose.y, 0.3, CARMEN_HOTPINK);
			grid_state_map[temp_index].state = Closed;

#if 0
//			if (cont_rs_nodes > 100) // Apenas para não visualizar os estados iniciais
				publish_intermediate_plan(n);
#endif
			//                        int cond1 = n->number_of_change_in_direction;
			//                        int cond2 = n->it_since_change_in_direction;
			if (((n->number_of_change_in_direction == 0) || ((n->it_since_change_in_direction * EXPAND_NODES_V) > astar_config.min_dist_motion_change)) && is_goal(&n, goal_node))
			{
#ifdef PRINT_TIME
				printf("Elapsed time = %lf\n", curr_time);
#endif

#if PRINT_NODES
				printf("Path encontrado!\n");
				printf("Current_node = %f %f %f %f %f %f %f %f %f\n", n->pose.x, n->pose.y, n->pose.theta, n->pose.trailer_theta[0], n->pose.trailer_theta[1], n->pose.trailer_theta[2], n->pose.trailer_theta[3], n->pose.trailer_theta[4], n->pose.v);
				printf("Initial node: %f %f %f \n", initial_pose->x, initial_pose->y, initial_pose->theta);
				printf("Goal node: %f %f %f \n", goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta);
				printf("condition: %d %d %lf\n", cond1, cond2, astar_config.min_dist_motion_change);
				printf("Expanded_nodes: %d\n", expanded_nodes_cont);
#endif
				get_astar_path(n, path_result);
				// for (size_t i = 1; i < path_result.size(); i++)
				//                             {
				//                                 printf("path: %lf %lf %lf %lf\n", path_result[i].x, path_result[i].y, path_result[i].theta, path_result[i].v);
				//                             }

#ifdef DRAW_EXPANSION_TREE
				for (size_t i = 1; i < path_result.size(); i++)
				{
					draw_point_in_opencv_image(path_result[i], path_result[i - 1], obstacle_distance_grid_map->config, cv::Scalar(0, 0, 255), 2);
				}

				draw_point_on_map_img(initial_pose->x, initial_pose->y, obstacle_distance_grid_map_print_map->config, cv::Scalar(153, 255, 255), map_image);
				carmen_vector_3D_t estimated_initial_pose = simulate_future_pose(initial_pose->x, initial_pose->y, initial_pose->theta, robot_config.distance_between_front_and_rear_axles / 2.0);
				draw_point_on_map_img(estimated_initial_pose.x, estimated_initial_pose.y, obstacle_distance_grid_map_print_map->config, cv::Scalar(255, 153, 255), map_image);

				draw_point_on_map_img(goal_node->pose.x, goal_node->pose.y, obstacle_distance_grid_map_print_map->config, cv::Scalar(0, 0, 255), map_image);
				carmen_vector_3D_t estimated_goal_node = simulate_future_pose(goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta, robot_config.distance_between_front_and_rear_axles / 2.0);
				draw_point_on_map_img(estimated_goal_node.x, estimated_goal_node.y, obstacle_distance_grid_map_print_map->config, cv::Scalar(255, 153, 255), map_image);

				// expansion_tree_file_name[strlen(expansion_tree_file_name) - 5] = str_offroad_planner_request + cont + '0';
				string offroad_expansion_filename = str_offroad_planner_request;
				string offroad_expansion_filename_full = offroad_expansion_filename + to_string(cont) + "__" + "cont_rs_nodes__" + to_string(cont_rs_nodes) + "__" + "elapsed_time" + to_string(curr_time) + ".png";
				printf("Expansion_tree_file_name = %s\n", offroad_expansion_filename_full.c_str());
				printf("cont_rs_nodes: %d\n", cont_rs_nodes);
				imwrite(offroad_expansion_filename_full.c_str(), map_image);

				char cmd_show[1024];
				sprintf(cmd_show, "xdg-open %s", offroad_expansion_filename_full.c_str());
				system(cmd_show);
				cont++;
				printf("Initial_pose: %lf %lf %lf %lf; goal_pose: %lf %lf %lf %lf\n", initial_pose->x, initial_pose->y, initial_pose->theta, initial_pose->trailer_theta[0], goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta, goal_node->pose.trailer_theta[0]);
#endif

#ifdef DRAW_PATHS_AND_POSES
				for (size_t i = 1; i < path_result.size(); i++)
				{

					draw_point_in_opencv_image(path_result[i], path_result[i - 1], obstacle_distance_grid_map->config, cv::Scalar(0, 0, 255), 2);
					draw_truck_and_trailers_paths_and_poses(path_result[i], path_result[i - 1], i, obstacle_distance_grid_map->config, cv::Scalar(0, 0, 255), 2);
				}

				draw_point_on_map_img(initial_pose->x, initial_pose->y, obstacle_distance_grid_map->config, cv::Scalar(153, 255, 255), map_image);

				draw_point_on_map_img(goal_node->pose.x, goal_node->pose.y, obstacle_distance_grid_map->config, cv::Scalar(0, 0, 255), map_image);

				draw_point_on_map_img(initial_pose->x, initial_pose->y, obstacle_distance_grid_map->config, cv::Scalar(153, 255, 255), map_image);

				draw_point_on_map_img(goal_node->pose.x, goal_node->pose.y, obstacle_distance_grid_map->config, cv::Scalar(0, 0, 255), map_image);

				// expansion_tree_file_name[strlen(expansion_tree_file_name) - 5] = str_offroad_planner_request + cont + '0';
				string offroad_paths_and_poses_filename = str_offroad_planner_request_2;
				string offroad_paths_and_poses_filename_full = offroad_paths_and_poses_filename + to_string(cont) + "__" + "cont_rs_nodes__" + to_string(cont_rs_nodes) + "__" + "elapsed_time" + to_string(curr_time) + ".png";
				printf("Paths_And_Poses = %s\n", offroad_paths_and_poses_filename_full.c_str());
				printf("cont_rs_nodes: %d\n", cont_rs_nodes);
				imwrite(offroad_paths_and_poses_filename_full.c_str(), map_image_2);

				char cmd_2_show[1024];
				sprintf(cmd_2_show, "xdg-open %s", offroad_paths_and_poses_filename_full.c_str());
				system(cmd_2_show);
				cont++;

				////
				offroad_paths_and_poses_filename = str_offroad_planner_request_3;
				offroad_paths_and_poses_filename_full = offroad_paths_and_poses_filename + to_string(cont) + "__" + "cont_rs_nodes__" + to_string(cont_rs_nodes) + "__" + "elapsed_time" + to_string(curr_time) + ".png";
				printf("Paths_And_Poses = %s\n", offroad_paths_and_poses_filename_full.c_str());
				printf("cont_rs_nodes: %d\n", cont_rs_nodes);
				imwrite(offroad_paths_and_poses_filename_full.c_str(), map_image_3);

				char cmd_3_show[1024];
				sprintf(cmd_3_show, "xdg-open %s", offroad_paths_and_poses_filename_full.c_str());
				system(cmd_3_show);
				cont++;

				printf("Initial_pose: %lf %lf %lf %lf; goal_pose: %lf %lf %lf %lf\n", initial_pose->x, initial_pose->y, initial_pose->theta, initial_pose->trailer_theta[0], goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta, goal_node->pose.trailer_theta[0]);
#endif

				clear_astar_search(FH, grid_state_map, goal_node);

				state_node *temp;
				while (n != NULL)
				{
					temp = n;
					n = n->parent;
					free(temp);
				}

				feedback = PLAN_OK;

				return (path_result);
			}
			else
			{
				expanded_nodes_cont++;
#if SEND_VIRTUAL_LASER
				publish_simulated_objects();
				// usleep(900);
#endif
				int branch_alive = 0;
				std::vector<state_node *> N = expand_node(n, goal_node, obstacle_distance_grid_map);
				for (unsigned int i = 0; i < N.size(); i++)
				{
					get_current_pos_in_grid_state_map(N[i], current_index, obstacle_distance_grid_map);
					temp_index = get_grid_state_map_index(current_index);

					if (grid_state_map[temp_index].state != Closed)
					{
						N[i]->parent = n;
						// N[i]->g = n->g + movement_cost(n, N[i]) + penalties(N[i], goal_node, obstacle_distance_grid_map);
						N[i]->g = n->g + penalties(N[i], goal_node, obstacle_distance_grid_map);
						//						N[i]->h = h(N[i], goal_node, obstacle_distance_grid_map, goal_distance_map, nonholonomic_heuristic_cost_map);
						//						N[i]->f = N[i]->g + N[i]->h;
						N[i]->branches = 0;
						// printf("n: %lf %lf, temp_index: %d: index: %d %d %d %d %d\n", N[i]->pose.phi, N[i]->g, temp_index, current_index.x, current_index.y, current_index.theta, current_index.direction, current_index.trailer_theta[0]);

						if (grid_state_map[temp_index].state == Not_visited || (grid_state_map[temp_index].state == Open &&
																				grid_state_map[temp_index].g > N[i]->g))
						{
							// Botei essas duas linhas após o if para não calcular inutilmente (quando não entra nesse if)
							N[i]->h = h(N[i], goal_node, obstacle_distance_grid_map, goal_distance_map, nonholonomic_heuristic_cost_map);
							N[i]->f = N[i]->g + N[i]->h;
							grid_state_map[temp_index].state = Open;
							grid_state_map[temp_index].g = N[i]->g;
							N[i]->parent->branches++;

							FH.push(N[i]);

							branch_alive = 1;
						}
						else
						{
							free(N[i]);
						}
					}
					else
					{
						free(N[i]);
					}
				}

#ifdef DRAW_EXPANSION_TREE
				int r = 0, g = 0, b = 0;
				int dist_to_goal = int(DIST2D(n->pose, goal_node->pose));
				int dist_to_initial = int(DIST2D(n->pose, *initial_pose));
				const int dist_initial_pose_to_goal = int(DIST2D(*initial_pose, goal_node->pose));

				g = (int)((float)dist_to_goal / dist_initial_pose_to_goal * 255.0f);
				b = (int)((float)dist_to_initial / dist_initial_pose_to_goal * 255.0f);

				g = carmen_clamp(0, g, 255);
				b = carmen_clamp(0, b, 255);
				draw_state_in_opencv_image(n, obstacle_distance_grid_map->config, cv::Scalar(b, g, r), map_image);
#endif

				if (branch_alive == 0)
				{
					remove_dead_branch(&n);
				}
			}
		}
		else
		{
			remove_dead_branch(&n);
		}

		cont_rs_nodes++;
		curr_time = carmen_get_time() - start_time;
	}

#ifdef PRINT_TIME
	printf("Elapsed time = %lf\n", curr_time);
#endif

	printf("Path não encontrado! %d %lf >= %lf\n", FH.empty(), curr_time, astar_timeout);
	printf("Initial_pose: %lf %lf %lf %lf; goal_pose: %lf %lf %lf %lf\n", initial_pose->x, initial_pose->y, initial_pose->theta, initial_pose->trailer_theta[0], goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta, goal_node->pose.trailer_theta[0]);

#ifdef DRAW_EXPANSION_TREE

	draw_point_on_map_img(initial_pose->x, initial_pose->y, obstacle_distance_grid_map->config, cv::Scalar(153, 255, 255), map_image);
	carmen_vector_3D_t estimated_initial_pose = simulate_future_pose(initial_pose->x, initial_pose->y, initial_pose->theta, robot_config.distance_between_front_and_rear_axles / 2.0);
	draw_point_on_map_img(estimated_initial_pose.x, estimated_initial_pose.y, obstacle_distance_grid_map->config, cv::Scalar(255, 153, 255), map_image);

	draw_point_on_map_img(goal_node->pose.x, goal_node->pose.y, obstacle_distance_grid_map->config, cv::Scalar(0, 0, 255), map_image);
	carmen_vector_3D_t estimated_goal_node = simulate_future_pose(goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta, robot_config.distance_between_front_and_rear_axles / 2.0);
	draw_point_on_map_img(estimated_goal_node.x, estimated_goal_node.y, obstacle_distance_grid_map->config, cv::Scalar(255, 153, 255), map_image);

	string offroad_expansion_filename_full = "GOAL_NOT_FOUND" + to_string(cont) + "__" + "cont_rs_nodes__" + to_string(cont_rs_nodes) + ".png";
	printf("Expansion_tree_file_name = %s\n", offroad_expansion_filename_full.c_str());
	printf("cont_rs_nodes: %d\n\n", cont_rs_nodes);

	imwrite(offroad_expansion_filename_full.c_str(), map_image);

	char cmd_show[1024];
	sprintf(cmd_show, "xdg-open %s &", offroad_expansion_filename_full.c_str());
	system(cmd_show);

	cont++;
#endif

	clear_astar_search(FH, grid_state_map, goal_node);

	feedback = GOAL_NOT_FOUND;

	return {};
}
