#include <algorithm>
#include <carmen/carmen.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/voice_interface_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_index.h>
#include <carmen/rddf_util.h>

#include "frenet_path_planner_interface.h"
#include "frenet_path_planner_messages.h"

#include <gsl/gsl_spline.h>

//#define PRINT_DEBUG

typedef struct
{
	gsl_interp_accel *acc;
	gsl_spline *spline;
	carmen_ackerman_traj_point_t *rddf_poses;
	int rddf_num_poses;
	bool fully_consumed;
} path_t;

extern double rddf_min_distance_between_waypoints;
extern int carmen_rddf_num_poses_ahead_max;
extern bool use_road_map;

bool robot_pose_queued = false;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
carmen_simulator_ackerman_truepos_message *current_truepos_msg = NULL;
carmen_map_p current_road_map = NULL;

extern char *carmen_rddf_filename;

int traffic_sign_is_on = false;
int traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
double traffic_sign_curvature = 0.0;
int state_traffic_sign_is_on = false;
int state_traffic_sign_code = RDDF_ANNOTATION_CODE_NONE;
double state_traffic_sign_curvature = 0.0;
carmen_point_t state_traffic_sign_pose = {0.0, 0.0, 0.0};

int carmen_rddf_end_point_is_set = 0;
carmen_point_t carmen_rddf_end_point;

int carmen_rddf_nearest_waypoint_is_set = 0;
carmen_point_t carmen_rddf_nearest_waypoint_to_end_point;

carmen_ackerman_traj_point_t *carmen_rddf_poses_ahead = NULL;
carmen_ackerman_traj_point_t *carmen_rddf_poses_back = NULL;
int carmen_rddf_num_poses_ahead = 0;
int carmen_rddf_num_poses_back = 0;
int *annotations_codes;
int *annotations;

int carmen_rddf_pose_initialized = 0;
int already_reached_nearest_waypoint_to_end_point = 0;

int frenet_path_planner_num_paths;
double frenet_path_planner_paths_displacement;
double frenet_path_planner_max_derivative_distance;
double frenet_path_planner_max_plan_size;
double frenet_path_planner_max_plan_size_increase_factor;

int selected_path_id;
double localize_ackerman_initialize_message_timestamp = 0.0;


vector<annotation_and_index> annotations_to_publish;
carmen_rddf_annotation_message annotation_queue_message;

int use_truepos = 0;

extern int traffic_lights_camera;
carmen_traffic_light_message *traffic_lights = NULL;

deque<carmen_rddf_dynamic_annotation_message> dynamic_annotation_messages;

carmen_moving_objects_point_clouds_message *moving_objects = NULL;

bool simulated_pedestrian_on = false;

//	- Os planos devem partir da posicao aproximada atual do carro, PA, que eh a posicao no plano atualmente sendo seguido
//	  (um dos 31) mais proxima possivel da globalpos.
//	  Alem disso, devem possuir continuidade temporal; isto eh, devem se conectar suavemente (incluir parte?)
//	  ao plano do time step anterior (ver imagem em https://www.researchgate.net/figure/Optimal-trajectory-thick-gray-with-end-point-on-the-continuous-terminal-manifold_fig5_254098780).
//	- Os planos podem ser computados a partir de PA por meio de um spline na dimensao d de FF, que tenha a
//	  dimensao s de FF como base (o spline eh em (s, d)).
//	  Para garantir a continuidade temporal, a derivada de d com relacao a s, dd/ds, do primeiro ponto, p0(t),
//	  do plano correntemente sendo seguido no tempo t, P(t), deve ser a mesma para todos os 31 planos e
//	  coincidir com a dd/ds do ponto px(t-1), correspondente a p0(t) do plano corrente no time step anterior, P(t-1),
//	  entre ciclos de planejamento para cada um dos 31 planos.
//	  Isto eh, a dd/ds do primeiro ponto, p0(t) = (s0(t), d0(t)), de um plano do time step atual, P(t),
//	  deve ser igual aa dd/ds da projecao de p0(t) na versao anterior de P(t), ou P(t-1).
//	  Ou seja, a dd/ds de p0(t) deve ser igual aa dd/ds do ponto px(t-1), onde a posicao de px(t-1) no mundo
//	  eh a mais proxima possivel de PA.
//	- Note que, alem da dd/ds, os valores de s e d de p0(t) de todos os 31 planos eh igual, mas eles divergem a
//	  partir de p0(t) para alcancar as 31 rotas paralelas no horizonte de tempo, T, e distancia na dimensao s de FF, D(s),
//	  considerados pelo planejador.
//	- O plano que deve ser correntemente seguido no tempo t, P(t), pode representar uma continuidade incompleta do afastamente de R
//	  na direcao de uma das rotas paralelas a R. Para garantir continuidade, o plano P(t) deve ter a mesma forma
//	  do plano anterior, P(t-1), a menos que o plano seja mudado em t. Assim, o plano anterior, P(t-1), deve ser apenas
//	  extendido para se gerar P(t). Os planos paralelos a P(t) devem ser, no entanto, recomputados completamente a partir de PA
//	  com as restricoes mencionadas acima.
//	- No momento inicial, os 31 planos sao gerados a partir da globalpos, isto eh, PA = globalpos. Depois do instante inicial,
//	  o plano sendo seguido em P(t-1) eh extendido a partir de PA (p0 de P(t) eh igual a PA) para se gerar P(t),
//	  e os demais sao computados a partir de PA.
//	- Os planos podem alcancar suas rotas paralelas a R antes do fim de T (em T/2, por exemplo) caso se deseje manobras mais agressivas.
//	- O horizonte de tempo T considerado pode ser aproximandamente igual a 5 segundos, equanto que o horizonte de distancia
//	  D (na dimensao s) vai de um minimo para velocidades baixas ou zero (13m? Um multiplo do tamanho do carro?),
//	  ate a distancia alcancavel dentro de T na velocidade e aceleracao atuais (no momento t).
//	- Um conjunto de valores de T e D podem ser considerados para a geracao de multiplos de 31 planos.
//	  Estes planos extras permitiriam mais opcoes ao behavior selector.
//	- Os valores maximos de dd/ds vao variar de acordo com os valores de T e D considerados e alguns planos podem ser inviaveis e
//	  devem ser descartados por ultrapassar os limites de velocidade de variacao de phi, de aceleracao do carro,
//	  de forca centrifuga, ou de jerk apos a conversao dos pontos (s, d) do plano para (x, y), onde estes limites podem ser
//	  facilmente verificados.
//	- Um plano especial pode ser tambem demandado pelo behavior selector e computado pelo planejador para permitir manobras de precisao,
//	  como as de estacionamento e passagem por espacos estreitos (cancela, por exemplo).
//	  Este plano especial seria especificado como um afastamento especifico na dimensao d do FF da R, e uma distancia D.


static double
compute_s_range(carmen_ackerman_traj_point_t *poses_ahead, int num_poses)
{
	double s_range = 0.0;
	for (int i = 0; i < (num_poses - 1); i++)
		s_range += DIST2D(poses_ahead[i], poses_ahead[i + 1]);

	return (s_range);
}


carmen_ackerman_traj_point_t
get_path_pose(path_t path, carmen_ackerman_traj_point_t *poses_ahead, int pose_i)
{
	carmen_ackerman_traj_point_t displaced_pose = poses_ahead[pose_i]; // copia os campos nao alterados abaixo

	double s = compute_s_range(poses_ahead, pose_i);
	if (s > (path.spline->interp->xmax - 0.01))
		s = path.spline->interp->xmax - 0.01;
	double d = gsl_spline_eval(path.spline, s, path.acc);
	displaced_pose.x = poses_ahead[pose_i].x + d * cos(poses_ahead[pose_i].theta + (M_PI / 2.0));
	displaced_pose.y = poses_ahead[pose_i].y + d * sin(poses_ahead[pose_i].theta + (M_PI / 2.0));

	s += 0.01;
	double delta_theta = atan2(gsl_spline_eval(path.spline, s, path.acc) - d, 0.01);
	displaced_pose.theta = carmen_normalize_theta(displaced_pose.theta + delta_theta);

	return (displaced_pose);
}


int
find_nearest_pose_index(carmen_ackerman_traj_point_t pose, carmen_ackerman_traj_point_t *rddf_poses, path_t *previously_selected_path)
{
	if (previously_selected_path->fully_consumed)
		return (-1);

	int num_rddf_poses = previously_selected_path->rddf_num_poses;
	double min_distance = DIST2D(pose, rddf_poses[0]);
	int nearest_pose_index = 0;
	int reserve_points = 5; // Eh para permitir achar todos os pontos necessários do spline

	for (int i = 1; i < (num_rddf_poses - reserve_points); i++)
	{
		double distance = DIST2D(pose, rddf_poses[i]);
		if (distance < min_distance)
		{
			min_distance = distance;
			nearest_pose_index = i;
		}
	}

	if (nearest_pose_index >= (num_rddf_poses - reserve_points - 1))
		return (-1);

	return (nearest_pose_index);
}


double
consume_s_range(carmen_ackerman_traj_point_t *rddf_poses, double s_range, int max_num_poses)
{
	double s_range_consumed = 0.0;

	int i = 0;
	while ((s_range_consumed < s_range) && (i < (max_num_poses - 1)))
	{
		s_range_consumed += DIST2D(rddf_poses[i], rddf_poses[i + 1]);
		i++;
	}
	
	return (s_range_consumed);
}


void
compute_a_path_within_a_path(path_t &path, path_t *previously_selected_path, double displacement, double s[5], int nearest_pose_index,
		double s_range)
{
	double prev_path_s0 = compute_s_range(previously_selected_path->rddf_poses, nearest_pose_index);
	double prev_path_s1 = prev_path_s0 + rddf_min_distance_between_waypoints;
	double prev_path_s2 = consume_s_range(&(previously_selected_path->rddf_poses[nearest_pose_index]),
			frenet_path_planner_max_derivative_distance * s_range, previously_selected_path->rddf_num_poses - nearest_pose_index) + prev_path_s0;
	if (prev_path_s2 > previously_selected_path->spline->interp->xmax)
		prev_path_s2 = previously_selected_path->spline->interp->xmax;
	if (prev_path_s1 > previously_selected_path->spline->interp->xmax)
	{
		prev_path_s0 = previously_selected_path->spline->interp->xmax - rddf_min_distance_between_waypoints;
		prev_path_s1 = previously_selected_path->spline->interp->xmax;
	}
	double d0 = gsl_spline_eval(previously_selected_path->spline, prev_path_s0, previously_selected_path->acc);
	double d1 = gsl_spline_eval(previously_selected_path->spline, prev_path_s1, previously_selected_path->acc);
	double d2 = (gsl_spline_eval(previously_selected_path->spline, prev_path_s2, previously_selected_path->acc) + displacement) / 2.0;
	double d3 = displacement;
	double d4 = displacement;
	double d[5] = { d0, d1, d2, d3, d4 };
	gsl_spline_init(path.spline, s, d, 5);
}


void
compute_a_path_after_a_path(path_t &path, path_t *previously_selected_path, double displacement, double s[5])
{
	double prev_path_s0 = previously_selected_path->spline->interp->xmax - rddf_min_distance_between_waypoints;
	double prev_path_s1 = previously_selected_path->spline->interp->xmax;
	double d0 = gsl_spline_eval(previously_selected_path->spline, prev_path_s0, previously_selected_path->acc);
	double d1 = gsl_spline_eval(previously_selected_path->spline, prev_path_s1, previously_selected_path->acc);
	double d2 = (gsl_spline_eval(previously_selected_path->spline, prev_path_s1, previously_selected_path->acc) + displacement) / 2.0;
	double d3 = displacement;
	double d4 = displacement;
	double d[5] = { d0, d1, d2, d3, d4 };
	gsl_spline_init(path.spline, s, d, 5);
}


path_t
compute_current_path(int path_id, path_t *previously_selected_path, carmen_ackerman_traj_point_t *poses_ahead, int num_poses,
		double v, bool first_time)
{
	path_t path;
	double displacement = frenet_path_planner_paths_displacement * ((frenet_path_planner_num_paths / 2) - path_id);
	double s_range = compute_s_range(poses_ahead, num_poses);
	double desired_s_range = frenet_path_planner_max_plan_size + v * frenet_path_planner_max_plan_size_increase_factor;
	if (s_range > desired_s_range)
		s_range = desired_s_range;

	double s[5] = {0.0, rddf_min_distance_between_waypoints, frenet_path_planner_max_derivative_distance * s_range, s_range - rddf_min_distance_between_waypoints, s_range};

	if (first_time)
	{
		path.acc = gsl_interp_accel_alloc();
		path.spline = gsl_spline_alloc(gsl_interp_cspline, 5);

		double d[5] = {0.0, 0.0, displacement / 2.0, displacement, displacement};
		gsl_spline_init(path.spline, s, d, 5);
		path.fully_consumed = false;
	}
	else
	{
		path.acc = gsl_interp_accel_alloc();
		path.spline = gsl_spline_alloc(gsl_interp_cspline, 5);

		int nearest_pose_index = find_nearest_pose_index(poses_ahead[0], previously_selected_path->rddf_poses, previously_selected_path);
		if (nearest_pose_index != -1) 	// trocou de plano mas ainda esta dentro do plano anterior
			compute_a_path_within_a_path(path, previously_selected_path, displacement, s, nearest_pose_index, s_range);
		else 							// ja consumiu todo o plano anterior
			compute_a_path_after_a_path(path, previously_selected_path, displacement, s);

		gsl_spline_free(previously_selected_path->spline);
		gsl_interp_accel_free(previously_selected_path->acc);
		free(previously_selected_path->rddf_poses);
	}

	path.fully_consumed = false;

	path.rddf_num_poses = num_poses;
	path.rddf_poses = (carmen_ackerman_traj_point_t *) malloc(path.rddf_num_poses * sizeof(carmen_ackerman_traj_point_t));
	for (int pose_i = 0; pose_i < path.rddf_num_poses; pose_i++)
		path.rddf_poses[pose_i] = poses_ahead[pose_i];

	return (path);
}


path_t
compute_one_path(int path_id, path_t *selected_path, carmen_ackerman_traj_point_t *poses_ahead, int num_poses, double v)
{
	path_t path;
	double disp = frenet_path_planner_paths_displacement;
	double displacement = disp * ((frenet_path_planner_num_paths / 2) - path_id);
	double s_range = compute_s_range(poses_ahead, num_poses);
	double desired_s_range = frenet_path_planner_max_plan_size + v * frenet_path_planner_max_plan_size_increase_factor;
	if (s_range > desired_s_range)
		s_range = desired_s_range;

	double s[5] = {0.0, rddf_min_distance_between_waypoints, frenet_path_planner_max_derivative_distance * s_range, s_range - rddf_min_distance_between_waypoints, s_range};

	path.acc = gsl_interp_accel_alloc();
	path.spline = gsl_spline_alloc(gsl_interp_cspline, 5);
	int nearest_pose_index = find_nearest_pose_index(poses_ahead[0], selected_path->rddf_poses, selected_path);
	if (nearest_pose_index != -1) 	// trocou de plano mas ainda esta dentro do plano anterior
	{
		selected_path->fully_consumed = false;
		compute_a_path_within_a_path(path, selected_path, displacement, s, nearest_pose_index, s_range);
	}
	else 							// ja consumiu todo o plano anterior
	{
		selected_path->fully_consumed = true;
		compute_a_path_after_a_path(path, selected_path, displacement, s);
	}

	path.rddf_num_poses = -1;
	path.rddf_poses = NULL;

	return (path);
}


carmen_ackerman_traj_point_t *
update_poses_back(carmen_ackerman_traj_point_t *current_poses_back, carmen_ackerman_traj_point_t *poses_back,
		carmen_ackerman_traj_point_t *path, int &num_poses_back, int num_poses, int selected_path_id)
{
	static int current_num_poses_back = 0;

	bool current_num_poses_back_increased = false;
	if (current_num_poses_back < num_poses_back)
	{
		current_num_poses_back = num_poses_back;
		current_num_poses_back_increased = true;
	}

	if (current_poses_back == NULL)
	{
		current_poses_back = (carmen_ackerman_traj_point_t *) malloc(current_num_poses_back * sizeof(carmen_ackerman_traj_point_t));
		for (int pose_i = 0; pose_i < current_num_poses_back; pose_i++)
			current_poses_back[pose_i] = poses_back[pose_i];
	}
	else
	{
		if (current_num_poses_back_increased)
		{
			current_poses_back = (carmen_ackerman_traj_point_t *) realloc(current_poses_back, current_num_poses_back * sizeof(carmen_ackerman_traj_point_t));
			for (int pose_i = 0; pose_i < current_num_poses_back; pose_i++)
				current_poses_back[pose_i] = poses_back[pose_i];
		}
		else
		{
			for (int pose_i = current_num_poses_back - 1; pose_i > 0; pose_i--)
				current_poses_back[pose_i] = current_poses_back[pose_i - 1];
		}
		current_poses_back[0] = path[selected_path_id * num_poses + 0];
		num_poses_back = current_num_poses_back; // Se vier menor, reestabelece o valor anterior aqui.
	}

	return (current_poses_back);
}


void
print_poses(carmen_ackerman_traj_point_t *poses_ahead, int num_poses)
{
	FILE *gnuplot_file = fopen("plot.txt", "w");

	for (int i = 0; i < num_poses - 1; i++)
		fprintf(gnuplot_file, "%lf %lf %lf %lf\n",
				poses_ahead[i].x, poses_ahead[i].y,
				poses_ahead[i + 1].x - poses_ahead[i].x,
				poses_ahead[i + 1].y - poses_ahead[i].y);

	fclose(gnuplot_file);
}


bool
reinitialize_current_path(path_t &current_path, int &current_path_id, carmen_ackerman_traj_point_t *&current_poses_back, bool first_time)
{
	if (!first_time)
	{
		gsl_spline_free(current_path.spline);
		gsl_interp_accel_free(current_path.acc);
		free(current_path.rddf_poses);
		free(current_poses_back);
	}
	current_poses_back = NULL;
	current_path_id = selected_path_id = frenet_path_planner_num_paths / 2;
	localize_ackerman_initialize_message_timestamp = 0.0;

	return (true);
}


carmen_frenet_path_planner_set_of_paths
build_frenet_path_plan(carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back,
		int num_poses, int num_poses_back, double v, int *annotations, int * annotations_codes)
{
	static path_t current_path;
	static carmen_ackerman_traj_point_t *current_poses_back = NULL;

//	smooth_rddf_using_conjugate_gradient(poses_ahead, num_poses, poses_back, num_poses_back);

//	print_poses(poses_ahead, num_poses);

    carmen_frenet_path_planner_set_of_paths set_of_paths;
    set_of_paths.set_of_paths_size = frenet_path_planner_num_paths * num_poses;

    static bool first_time = true;
    set_of_paths.set_of_paths = (carmen_ackerman_traj_point_t *) malloc(set_of_paths.set_of_paths_size * sizeof(carmen_ackerman_traj_point_t));
	static int current_path_id = -1;

	if ((current_path_id != selected_path_id) || (localize_ackerman_initialize_message_timestamp != 0.0))
	{
		if (localize_ackerman_initialize_message_timestamp != 0.0)
			first_time = reinitialize_current_path(current_path, current_path_id, current_poses_back, first_time);
		else
			current_path_id = selected_path_id;

		current_path = compute_current_path(current_path_id, &current_path, poses_ahead, num_poses, v, first_time);
	}
	first_time = false;

	for (int path_id = 0; path_id < frenet_path_planner_num_paths; path_id++)
	{
		path_t path;
		path = compute_one_path(path_id, &current_path, poses_ahead, num_poses, v);
		for (int pose_i = 0; pose_i < num_poses; pose_i++)
			set_of_paths.set_of_paths[path_id * num_poses + pose_i] = get_path_pose(path, poses_ahead, pose_i);

		gsl_spline_free(path.spline);
		gsl_interp_accel_free(path.acc);
	}

	current_poses_back = update_poses_back(current_poses_back, poses_back, set_of_paths.set_of_paths, num_poses_back, num_poses, selected_path_id);
//	print_poses(current_poses_back, num_poses_back);

	set_of_paths.selected_path = selected_path_id;
	set_of_paths.poses_back = current_poses_back;
	set_of_paths.rddf_poses_back = poses_back;
	set_of_paths.rddf_poses_ahead = poses_ahead;
	set_of_paths.number_of_poses = num_poses;
	set_of_paths.number_of_poses_back = num_poses_back;
	set_of_paths.annotations = annotations;
	set_of_paths.annotations_codes = annotations_codes;
	set_of_paths.timestamp = carmen_get_time();
	set_of_paths.host = carmen_get_host();

    return (set_of_paths);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_publish_annotation_queue()
{
	IPC_RETURN_TYPE err;

	if (annotations_to_publish.size() == 0)
	{
		annotation_and_index annotation_i;
		memset(&annotation_i, 0, sizeof(annotation_and_index));
		carmen_annotation_t &annotation = annotation_i.annotation;
		annotation.annotation_type = RDDF_ANNOTATION_TYPE_NONE;
		annotation.annotation_code = RDDF_ANNOTATION_CODE_NONE;
		annotations_to_publish.push_back(annotation_i);
	}

	if (annotation_queue_message.annotations == NULL)
	{
		annotation_queue_message.annotations = (carmen_annotation_t *) calloc(annotations_to_publish.size(), sizeof(carmen_annotation_t));

		if (!annotation_queue_message.annotations)
			exit(printf("Allocation error in carmen_rddf_play_publish_annotation_queue()::1\n"));
	}
	else if (annotation_queue_message.num_annotations != (int) annotations_to_publish.size())
	{
		annotation_queue_message.annotations = (carmen_annotation_t *) realloc(annotation_queue_message.annotations, annotations_to_publish.size() * sizeof(carmen_annotation_t));

		if (!annotation_queue_message.annotations)
			exit(printf("Allocation error in carmen_rddf_play_publish_annotation_queue()::2\n"));
	}

	annotation_queue_message.num_annotations = annotations_to_publish.size();

//	printf ("Annotation size %d\n", (int)annotations_to_publish.size());

	for (size_t i = 0; i < annotations_to_publish.size(); i++)
	{
//		printf ("code %d\n", annotations_to_publish[i].annotation.annotation_type);
		memcpy(&(annotation_queue_message.annotations[i]), &(annotations_to_publish[i].annotation), sizeof(carmen_annotation_t));
	}
	annotation_queue_message.host = carmen_get_host();
	annotation_queue_message.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_RDDF_ANNOTATION_MESSAGE_NAME, &annotation_queue_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_RDDF_ANNOTATION_MESSAGE_FMT);
}


static void
carmen_rddf_play_publish_rddf_and_annotations(carmen_point_t robot_pose, double v)
{
	// so publica rddfs quando a pose do robo ja estiver setada
	if ((carmen_rddf_num_poses_ahead > 0) && (carmen_rddf_num_poses_back > 0))
	{
		carmen_rddf_play_clear_annotations();
		carmen_rddf_play_set_annotations(robot_pose);

		carmen_frenet_path_planner_set_of_paths set_of_paths_message = build_frenet_path_plan(
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				carmen_rddf_num_poses_ahead,
				carmen_rddf_num_poses_back,
				v,
				annotations,
				annotations_codes);

		carmen_frenet_path_planner_publish_set_of_paths_message(&set_of_paths_message);

		carmen_rddf_publish_road_profile_message(
			&(set_of_paths_message.set_of_paths[set_of_paths_message.selected_path * set_of_paths_message.number_of_poses]),
			set_of_paths_message.poses_back,
			carmen_rddf_num_poses_ahead,
			set_of_paths_message.number_of_poses_back,
			annotations,
			annotations_codes);

		free(set_of_paths_message.set_of_paths);

		carmen_rddf_play_publish_annotation_queue();

		carmen_rddf_publish_traffic_sign_message(state_traffic_sign_code, state_traffic_sign_curvature);
	}
}


void
carmen_rddf_play_find_and_publish_poses_around_end_point(double x, double y, double yaw, int num_poses_desired, double timestamp)
{
	int num_poses_acquired = 0;
	carmen_ackerman_traj_point_t *poses_around_end_point;

	poses_around_end_point = (carmen_ackerman_traj_point_t *) calloc (num_poses_desired, sizeof(carmen_ackerman_traj_point_t));
	carmen_test_alloc(poses_around_end_point);

	num_poses_acquired = carmen_find_poses_around(x, y, yaw, timestamp, poses_around_end_point, num_poses_desired);
	carmen_rddf_publish_road_profile_around_end_point_message(poses_around_end_point, num_poses_acquired);

	free(poses_around_end_point);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
build_and_send_rddf_and_annotations(carmen_point_t robot_pose, double v, double timestamp)
{
	if (use_road_map)
	{
		robot_pose_queued = (current_road_map == NULL || carmen_rddf_play_pose_out_of_map_coordinates(robot_pose, current_road_map));
		if (robot_pose_queued)
			return;
		carmen_rddf_play_check_reset_traffic_sign_state(robot_pose);
		carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_by_road_map(
				robot_pose,
				current_road_map,
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				&carmen_rddf_num_poses_back,
				carmen_rddf_num_poses_ahead_max);
	}
	else
	{
		carmen_rddf_num_poses_ahead = carmen_rddf_play_find_nearest_poses_ahead(
				robot_pose.x,
				robot_pose.y,
				robot_pose.theta,
				timestamp,
				carmen_rddf_poses_ahead,
				carmen_rddf_poses_back,
				&carmen_rddf_num_poses_back,
				carmen_rddf_num_poses_ahead_max,
				annotations);
	}

	annotations_to_publish.clear();
	carmen_check_for_annotations(robot_pose, carmen_rddf_poses_ahead, carmen_rddf_poses_back,
			carmen_rddf_num_poses_ahead, carmen_rddf_num_poses_back, timestamp);

	carmen_rddf_play_publish_rddf_and_annotations(robot_pose, v);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_rddf_pose_initialized = 1;
	current_globalpos_msg = msg;
	build_and_send_rddf_and_annotations(msg->globalpos, msg->v, msg->timestamp);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	carmen_rddf_pose_initialized = 1;
	current_truepos_msg = msg;
	build_and_send_rddf_and_annotations(msg->truepose, msg->v, msg->timestamp);
}


static void
road_map_handler(carmen_map_server_road_map_message *msg)
{
	static bool first_time = true;

	if (first_time)
	{
		current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
		carmen_grid_mapping_initialize_map(current_road_map, msg->config.x_size, msg->config.resolution, 'r');
		first_time = false;
	}

	if (msg->config.x_origin != current_road_map->config.x_origin || msg->config.y_origin != current_road_map->config.y_origin) // new map
	{
		memcpy(current_road_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
		current_road_map->config = msg->config;

		if (robot_pose_queued)
		{
			if (use_truepos)
				simulator_ackerman_truepos_message_handler(current_truepos_msg);
			else
				localize_globalpos_handler(current_globalpos_msg);
		}
	}
}


//void
//carmen_rddf_play_nearest_waypoint_message_handler(carmen_rddf_nearest_waypoint_message *rddf_nearest_waypoint_message)
//{
//	carmen_rddf_nearest_waypoint_to_end_point = rddf_nearest_waypoint_message->point;
//	carmen_rddf_nearest_waypoint_is_set = 1;
//
//	carmen_rddf_publish_nearest_waypoint_confirmation_message(rddf_nearest_waypoint_message->point);
//	already_reached_nearest_waypoint_to_end_point = 0;
//}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	if (rddf_end_point_message->number_of_poses > 1)
	{
		carmen_rddf_play_find_and_publish_poses_around_end_point(
				rddf_end_point_message->point.x,
				rddf_end_point_message->point.y,
				rddf_end_point_message->point.theta,
				rddf_end_point_message->number_of_poses,
				rddf_end_point_message->timestamp
		);

		carmen_rddf_end_point = rddf_end_point_message->point;
		carmen_rddf_end_point_is_set = 1;
	}
	else
	{
		carmen_rddf_play_find_and_publish_poses_around_end_point(
				rddf_end_point_message->point.x,
				rddf_end_point_message->point.y,
				rddf_end_point_message->point.theta,
				rddf_end_point_message->number_of_poses,
				rddf_end_point_message->timestamp
		);

		carmen_rddf_nearest_waypoint_to_end_point = rddf_end_point_message->point;
		carmen_rddf_nearest_waypoint_is_set = 1;

		already_reached_nearest_waypoint_to_end_point = 0;
	}
}


static void
carmen_frenet_path_planner_selected_path_message_handler(carmen_frenet_path_planner_selected_path *message)
{
	selected_path_id = message->selected_path;
}


static void
carmen_traffic_light_message_handler(carmen_traffic_light_message *message)
{
	traffic_lights = message;
}


static void
carmen_rddf_dynamic_annotation_message_handler(carmen_rddf_dynamic_annotation_message *message)
{
	// Avoid reinserting the same annotation over and over.
	for (size_t i = 0; i < dynamic_annotation_messages.size(); i++)
	{
		carmen_rddf_dynamic_annotation_message &stored = dynamic_annotation_messages[i];
		if (stored.annotation_type == message->annotation_type &&
			stored.annotation_code == message->annotation_code &&
			DIST2D(stored.annotation_point, message->annotation_point) < 0.5)
		{
			stored = *message;
			return;
		}
	}

	dynamic_annotation_messages.push_front(*message);
	if (dynamic_annotation_messages.size() > 30)
		dynamic_annotation_messages.pop_back();
}


void
carmen_moving_objects_point_clouds_message_handler(carmen_moving_objects_point_clouds_message *moving_objects_point_clouds_message)
{
	moving_objects = moving_objects_point_clouds_message;
}


void
carmen_voice_interface_command_message_handler(carmen_voice_interface_command_message *message)
{
	if (message->command_id == SET_COURSE)
	{
		printf("New rddf set by voice command: %s\n", message->command);

		carmen_rddf_index_clear();

		char *carmen_home = getenv("CARMEN_HOME");
		static char rddf_file_name[2048];
		strcpy(rddf_file_name, carmen_home);
		strcat(rddf_file_name, "/");
		strcat(rddf_file_name, message->command);

		carmen_rddf_play_load_index(rddf_file_name);

		char carmen_annotation_filename[2048];
		rddf_file_name[strlen(rddf_file_name) - 4] = '\0';
		sprintf(carmen_annotation_filename, "%s_annotation.txt", rddf_file_name);

		carmen_rddf_play_load_annotation_file(carmen_annotation_filename);
	}
}


static void
carmen_localize_ackerman_initialize_message_handler(carmen_localize_ackerman_initialize_message *initialize_msg)
{
	localize_ackerman_initialize_message_timestamp = initialize_msg->timestamp;
}


static void
frenet_path_planner_shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_frenet_path_planner_subscribe_messages()
{
	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (use_road_map)
		carmen_map_server_subscribe_road_map(NULL, (carmen_handler_t) road_map_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_rddf_subscribe_nearest_waypoint_message(NULL,
//			(carmen_handler_t) carmen_rddf_play_nearest_waypoint_message_handler,
//			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL,
			(carmen_handler_t) carmen_rddf_play_end_point_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

    carmen_traffic_light_subscribe(traffic_lights_camera, NULL, (carmen_handler_t) carmen_traffic_light_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_rddf_subscribe_dynamic_annotation_message(NULL, (carmen_handler_t) carmen_rddf_dynamic_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) carmen_moving_objects_point_clouds_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_voice_interface_subscribe_command_message(NULL, (carmen_handler_t) carmen_voice_interface_command_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_frenet_path_planner_subscribe_selected_path_message(NULL, (carmen_handler_t) carmen_frenet_path_planner_selected_path_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) carmen_localize_ackerman_initialize_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
frenet_path_planner_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "frenet_path_planner", (char *) "num_paths", CARMEN_PARAM_INT, &frenet_path_planner_num_paths, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "paths_displacement", CARMEN_PARAM_DOUBLE, &frenet_path_planner_paths_displacement, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "max_derivative_distance", CARMEN_PARAM_DOUBLE, &frenet_path_planner_max_derivative_distance, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "max_plan_size", CARMEN_PARAM_DOUBLE, &frenet_path_planner_max_plan_size, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "max_plan_size_increase_factor", CARMEN_PARAM_DOUBLE, &frenet_path_planner_max_plan_size_increase_factor, 0, NULL},
		{(char *) "frenet_path_planner", (char *) "max_derivative_distance", CARMEN_PARAM_DOUBLE, &frenet_path_planner_max_derivative_distance, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


void
frenet_path_planner_initialize(void)
{
	carmen_rddf_poses_ahead = (carmen_ackerman_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_ackerman_traj_point_t));
	carmen_rddf_poses_back = (carmen_ackerman_traj_point_t *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(carmen_ackerman_traj_point_t));
	annotations = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	annotations_codes = (int *) calloc (carmen_rddf_num_poses_ahead_max, sizeof(int));
	memset(&annotation_queue_message, 0, sizeof(annotation_queue_message));

	carmen_test_alloc(carmen_rddf_poses_ahead);
	carmen_test_alloc(annotations);

	selected_path_id = frenet_path_planner_num_paths / 2;
}


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, "C");

	char *carmen_annotation_filename = carmen_rddf_play_parse_input_command_line_parameters(argc, argv);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	carmen_rddf_play_get_parameters(argc, argv);
	frenet_path_planner_get_parameters(argc, argv);
	frenet_path_planner_initialize();

	carmen_rddf_define_messages();
	carmen_frenet_path_planner_define_messages();

	signal(SIGINT, frenet_path_planner_shutdown_module);

	if (!use_road_map)
		carmen_rddf_play_load_index(carmen_rddf_filename);

	carmen_rddf_play_load_annotation_file(carmen_annotation_filename);

	carmen_frenet_path_planner_subscribe_messages();

	carmen_ipc_dispatch();

	return (0);
}
