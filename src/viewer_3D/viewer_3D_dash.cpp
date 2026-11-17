/*
 * viewer_3D_dash -- o viewer_3D com um painel do carro por cima.
 *
 * NAO E' UMA COPIA. Este arquivo inclui o proprio viewer_3D.cpp, de modo que a cena 3D e' a
 * mesma, linha por linha, e continua acompanhando qualquer mudanca feita la'. O que muda:
 *
 *   - o main() original e' renomeado (via macro) e substituido pelo daqui;
 *   - as chamadas a processWindow() dentro do viewer_3D.cpp passam por um wrapper, para que o
 *     painel veja o mouse e o teclado antes da cena;
 *   - depois de draw_everything() o painel e' desenhado por cima (o glXSwapBuffers acontece no
 *     showWindow() do quadro seguinte, entao da' tempo).
 *
 * Teclas novas:  F painel on/off   G lista de missoes   H camera
 *
 * Uso:  ./viewer_3D_dash [as mesmas opccoes do viewer_3D]
 */

#include <GL/glew.h>		// tem que vir antes de qualquer gl.h (o Window.h inclui gl.h)

#include <dirent.h>
#include <vector>
#include <string>
#include <algorithm>

#include "Window.h"
#include "dashboard.h"

// prototipos dos ganchos, porque o viewer_3D.cpp os chama antes de existirem
static int dash_process_window(window *w, void (*mouse_func)(int type, int button, int x, int y),
		void (*key_press)(int code), void (*key_release)(int code),
		void (*resize_func)(int width, int height));

// desvia as chamadas de dentro do viewer_3D.cpp sem tocar no arquivo
#define processWindow	dash_process_window
#define main			viewer_3D_unused_main

#include "viewer_3D.cpp"

#undef main
#undef processWindow

#include <carmen/ford_escape_hybrid_messages.h>
#include <carmen/ford_escape_hybrid_interface.h>
#include <carmen/task_manager_interface.h>
#include <carmen/user_app_server_interface.h>
#include <carmen/user_app_server_messages.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/rddf_interface.h>

// declarado a mao: route_planner_interface.h arrasta headers que redefinem tipos que o
// viewer_3D.cpp ja' trouxe (e libglobal_planning_interface colide com librddf_interface)
extern "C" void carmen_route_planner_set_destination(char *destination,
		carmen_point_t destination_point);


static void (*g_original_mouse_func)(int, int, int, int) = NULL;
static void (*g_original_key_press)(int) = NULL;

static char *g_missions_folder = NULL;
static std::vector<std::string> g_mission_files;		// caminho completo
static std::vector<std::string> g_mission_names;		// so' o nome, para a tela
static int g_first_mission = 1;

// lugares para onde a camera pode ser apontada: as anotaccoes do mapa, em coordenadas de mundo
static std::vector<std::string> g_camera_target_names;
static std::vector<carmen_vector_3D_t> g_camera_target_points;

static double g_axle_distance = 2.625;
static double g_max_velocity = 10.0;
static double g_max_steering_angle = 0.5;

// Relaccao de direccao: quantos graus o volante gira por grau de roda. E' o
// ANGLE_TIRE_TO_STEERING = 16.0 do painel do viewer do astro; nao existe parametro para isso
// no carmen, entao fica configuravel pelo ini (viewer_3D_steering_ratio) e pela linha de comando.
static double g_steering_ratio = 16.0;

// O phi que vale e' o do base_ackerman_odometry -- e' de la que o astro tira o angulo do volante.
// O do globalpos so' entra quando nao ha odometria chegando.
static double g_last_odometry_time = 0.0;
static int g_dash_camera_id = 1;

static double g_dash_scale = 0.0;			// 0 = automatica
static double g_dash_opacity = 1.0;
static int g_dash_position = 0;

// teclas novas (keycodes do X, como o resto do viewer_3D)
#define KEY_F				41
#define KEY_G				42
#define KEY_H				43
#define KEY_O				32
#define KEY_BRACKET_LEFT	34
#define KEY_BRACKET_RIGHT	35
#define KEY_COMMA			59
#define KEY_PERIOD			60
#define KEY_Z				52		// 54 (C) ja e' "desenhar o carro" no viewer_3D


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// Codigos de marcha do ByWire XGV (ByWire XGV User Manual v1.5, pag. 67). E' o que o
// ford_escape_hybrid repassa em g_XGV_gear. Qualquer outro valor fica como desconhecido e o
// painel mostra o numero cru, para nao inventar uma marcha que o carro nao disse.
static int
gear_from_xgv_code(int code)
{
	switch (code)
	{
	case 1:		return (DASHBOARD_GEAR_L);		// Low
	case 2:		return (DASHBOARD_GEAR_D);		// Drive
	case 128:	return (DASHBOARD_GEAR_N);		// Neutral
	case 129:	return (DASHBOARD_GEAR_R);		// Reverse
	}

	return (DASHBOARD_GEAR_UNKNOWN);
}


// Sem CAN de verdade -- e' o caso do simulator_ackerman, que publica o ford_escape_status
// inteiro zerado -- a marcha sai do movimento: o sinal da velocidade, ou o do comando quando o
// carro ainda esta parado. Sem isso a marca ficava presa e a inversao nunca aparecia no painel.
static void
update_estimated_gear(void)
{
	static double last_moving_time = 0.0;
	static int last_gear = DASHBOARD_GEAR_N;
	dashboard_data_t *d = dashboard_data();
	double reference;

	if (!d->gear_estimated)
		return;

	if (fabs(d->v) > 0.08)
		reference = d->v;
	else if (fabs(d->v_command) > 0.02)
		reference = d->v_command;
	else
		reference = 0.0;

	if (reference < 0.0)
		last_gear = DASHBOARD_GEAR_R;
	else if (reference > 0.0)
		last_gear = DASHBOARD_GEAR_D;

	if (reference != 0.0)
		last_moving_time = carmen_get_time();
	else if ((carmen_get_time() - last_moving_time) > 1.5)
		last_gear = DASHBOARD_GEAR_N;		// parado ha um tempo: neutro

	d->gear = last_gear;
}


static void
dash_globalpos_handler(carmen_localize_ackerman_globalpos_message *message)
{
	static double last_v = 0.0;
	static double last_timestamp = 0.0;
	dashboard_data_t *d = dashboard_data();

	// Nenhum veiculo terrestre anda a 100 m/s (360 km/h). Um valor acima disso so' pode ser
	// mensagem corrompida ou log lido com o layout errado -- foi o que aconteceu com os logs
	// gravados com semi-reboque, em que o msg->v recebia a coordenada UTM e o painel mostrava
	// 27.928.443,6 km/h (ver carmen_string_to_globalpos_message em src/logger/readlog.cpp).
	// Descartar e' melhor que exibir: o valor tambem envenena a_long e a_lat.
	if (fabs(message->v) < 100.0)
		d->v = message->v;
	else
	{
		static double ultimo_aviso = 0.0;

		if ((carmen_get_time() - ultimo_aviso) > 5.0)
		{
			printf("viewer_3D_dash: globalpos com v absurdo (%.1f m/s) -- ignorado\n", message->v);
			ultimo_aviso = carmen_get_time();
		}
		return;
	}

	if ((carmen_get_time() - g_last_odometry_time) > 1.0)
	{
		d->phi = message->phi;
		d->steering_wheel = carmen_radians_to_degrees(message->phi) * g_steering_ratio;
	}
	d->x = message->globalpos.x;
	d->y = message->globalpos.y;
	d->theta = message->globalpos.theta;
	d->t_globalpos = carmen_get_time();

	if (last_timestamp > 0.0 && (message->timestamp - last_timestamp) > 0.001)
	{
		double a = (message->v - last_v) / (message->timestamp - last_timestamp);

		// filtro de primeira ordem, senao o numero fica ilegivel de tanto pular
		d->a_long = 0.7 * d->a_long + 0.3 * a;
	}
	last_v = message->v;
	last_timestamp = message->timestamp;

	if (g_axle_distance > 0.01)
		d->a_lat = message->v * message->v * tan(message->phi) / g_axle_distance;

	update_estimated_gear();
}


// Mesma fonte que o painel do astro usa (base_ackerman_odometry_handler, viewer_3D.cpp:4762):
// o angulo do volante e' graus(phi) x relaccao de direccao. O g_XGV_steering do CAN e' uma
// PORCENTAGEM do batente, nao um angulo -- por isso ele fica numa linha separada.
static void
dash_base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *message)
{
	dashboard_data_t *d = dashboard_data();

	d->phi = message->phi;
	d->steering_wheel = carmen_radians_to_degrees(message->phi) * g_steering_ratio;
	g_last_odometry_time = carmen_get_time();

	if (g_axle_distance > 0.01)
		d->a_lat = message->v * message->v * tan(message->phi) / g_axle_distance;
}


static void
dash_ford_escape_status_handler(carmen_ford_escape_status_message *message)
{
	dashboard_data_t *d = dashboard_data();

	d->throttle = message->g_XGV_throttle;
	d->brake = message->g_XGV_brakes;
	d->steering_wheel_pct = message->g_XGV_steering;

	d->gear_raw = message->g_XGV_gear;
	{
		int gear = gear_from_xgv_code(message->g_XGV_gear);

		d->gear_estimated = (gear == DASHBOARD_GEAR_UNKNOWN);
		if (!d->gear_estimated)
			d->gear = gear;		// codigo desconhecido nao apaga a marcha; a deducao assume
	}
	d->turn_signal = message->g_XGV_turn_signal;
	d->headlights = message->g_XGV_headlights_status;
	d->horn = message->g_XGV_horn_status;
	d->parking_brake = message->g_XGV_parking_brake;
	d->engine = message->g_XGV_main_propulsion;
	d->t_can = carmen_get_time();

	update_estimated_gear();
}


static void
dash_navigator_status_handler(carmen_navigator_ackerman_status_message *message)
{
	dashboard_data_t *d = dashboard_data();

	d->autonomous = message->autonomous;

	if (message->goal_set)
		d->goal_distance = hypot(message->goal.x - message->robot.x,
				message->goal.y - message->robot.y);
	else
		d->goal_distance = -1.0;
}


static void
dash_behavior_selector_handler(carmen_behavior_selector_state_message *message)
{
	dashboard_data_t *d = dashboard_data();

	strncpy(d->behavior_state, get_low_level_state_name(message->low_level_state),
			sizeof(d->behavior_state) - 1);
	d->behavior_state[sizeof(d->behavior_state) - 1] = '\0';

	strncpy(d->route_planner_state, print_route_planner_state(message->route_planner_state),
			sizeof(d->route_planner_state) - 1);
	d->route_planner_state[sizeof(d->route_planner_state) - 1] = '\0';
	d->t_route = carmen_get_time();
}


static void
dash_mission_state_handler(carmen_task_manager_mission_state_message *message)
{
	dashboard_data_t *d = dashboard_data();

	strncpy(d->mission_state, print_mission_level_state(message->mission_state),
			sizeof(d->mission_state) - 1);
	d->mission_state[sizeof(d->mission_state) - 1] = '\0';

	if (message->mission_id != NULL && message->mission_id[0] != '\0')
	{
		strncpy(d->mission_name, message->mission_id, sizeof(d->mission_name) - 1);
		d->mission_name[sizeof(d->mission_name) - 1] = '\0';
	}
}


static void
dash_motion_command_handler(carmen_base_ackerman_motion_command_message *message)
{
	dashboard_data_t *d = dashboard_data();

	if (message->num_motion_commands > 0)
	{
		d->v_command = message->motion_command[0].v;
		d->phi_command = message->motion_command[0].phi;
	}

	update_estimated_gear();
}


// O map_server so' publica o offline_map na PRIMEIRA pose e depois so' quando o carro muda de
// bloco de mapa. Quem sobe depois disso (o caso normal quando se roda o viewer na mao) nunca
// recebe mapa nenhum. Mas o map_server responde a um pedido -- e ninguem no carmen usava isso.
static void
request_offline_map(void)
{
	static int defined = 0;
	carmen_mapper_map_message *response = NULL;
	IPC_RETURN_TYPE err;

	if (!defined)
	{
		err = IPC_defineMsg(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME,
				IPC_VARIABLE_LENGTH, CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_FMT);
		carmen_test_ipc(err, "Could not define message",
				CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME);
		defined = 1;
	}

	err = IPC_queryResponseData(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME,
			carmen_default_message_create(), (void **) &response, 3000);

	if (err != IPC_OK || response == NULL)
	{
		printf("viewer_3D_dash: o map_server nao respondeu ao pedido de mapa\n");
		return;
	}

	printf("viewer_3D_dash: mapa pedido ao map_server: %dx%d res %.2f origem (%.1f, %.1f)\n",
			response->config.x_size, response->config.y_size, response->config.resolution,
			response->config.x_origin, response->config.y_origin);

	offline_map_update_handler(response);
}


static void
dash_camera_handler(camera_message *message)
{
	dashboard_data_t *d = dashboard_data();

	if (message->number_of_images < 1)
		return;

	camera_image *image = &message->images[0];

	if (image->raw_data == NULL || image->size_in_bytes_of_each_element != 1)
		return;

	dashboard_set_camera_image((unsigned char *) image->raw_data, image->width, image->height,
			image->number_of_channels);
	d->t_camera = carmen_get_time();
}


// A camera so' e' assinada quando alguem abre a imagem (tecla H). Uma camera publicando a
// 30 Hz custa, POR QUADRO, um desempacotamento IPC da imagem inteira mais a conversao para
// textura -- tudo isso dentro do processo do viewer, mesmo com o painel fechado, que e' o
// caso normal. Assinar na hora de abrir custa no maximo o atraso de um quadro da camera.
static void
ensure_camera_subscribed(void)
{
	static int subscribed = 0;

	if (subscribed)
		return;

	camera_drivers_subscribe_message(g_dash_camera_id, NULL,
			(carmen_handler_t) dash_camera_handler, CARMEN_SUBSCRIBE_LATEST);
	subscribed = 1;

	printf("viewer_3D_dash: assinando a camera %d\n", g_dash_camera_id);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Missoes                                                                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
load_mission_list(void)
{
	DIR *directory;
	struct dirent *entry;
	std::vector<std::string> found;

	g_mission_files.clear();
	g_mission_names.clear();

	if (g_missions_folder == NULL || g_missions_folder[0] == '\0')
		return;

	directory = opendir(g_missions_folder);
	if (directory == NULL)
	{
		printf("viewer_3D_dash: nao consegui abrir a pasta de missoes '%s'\n", g_missions_folder);
		return;
	}

	while ((entry = readdir(directory)) != NULL)
	{
		std::string name = entry->d_name;

		if (name[0] == '.')
			continue;
		if (name.size() < 5 || name.compare(name.size() - 4, 4, ".txt") != 0)
			continue;

		found.push_back(name);
	}
	closedir(directory);

	std::sort(found.begin(), found.end());

	for (unsigned int i = 0; i < found.size() && i < DASHBOARD_MAX_MISSIONS; i++)
	{
		g_mission_files.push_back(std::string(g_missions_folder) + "/" + found[i]);
		g_mission_names.push_back(found[i].substr(0, found[i].size() - 4));
	}

	char *names[DASHBOARD_MAX_MISSIONS];
	for (unsigned int i = 0; i < g_mission_names.size(); i++)
		names[i] = (char *) g_mission_names[i].c_str();

	dashboard_set_missions(names, (int) g_mission_names.size());

	printf("viewer_3D_dash: %d missoes em %s\n", (int) g_mission_names.size(), g_missions_folder);
}


static void
execute_mission(const char *name)
{
	std::string path;

	for (unsigned int i = 0; i < g_mission_names.size(); i++)
	{
		if (g_mission_names[i] == name)
			path = g_mission_files[i];
	}

	if (path.empty())
		return;

	FILE *file = fopen(path.c_str(), "r");
	if (file == NULL)
	{
		printf("viewer_3D_dash: nao consegui ler a missao %s\n", path.c_str());
		return;
	}

	std::string text;
	char line[2048];
	while (fgets(line, sizeof(line), file) != NULL)
		text += line;
	fclose(file);

	if (text.empty())
	{
		printf("viewer_3D_dash: missao %s esta' vazia\n", path.c_str());
		return;
	}

	// mesma sequencia do navigator_gui2: da segunda missao em diante, aborta a anterior antes
	if (!g_first_mission)
		carmen_user_app_server_publish_update_mission_message(USER_APP_SERVER_MISSION_ABORT,
				carmen_get_time());

	carmen_user_app_server_publish_execute_mission_message(text.c_str(), carmen_get_time());
	g_first_mission = 0;

	strncpy(dashboard_data()->mission_name, name, DASHBOARD_MISSION_NAME_SIZE - 1);
	dashboard_data()->mission_name[DASHBOARD_MISSION_NAME_SIZE - 1] = '\0';

	printf("viewer_3D_dash: missao '%s' publicada\n", name);
}


static void
abort_mission(void)
{
	carmen_user_app_server_publish_update_mission_message(USER_APP_SERVER_MISSION_ABORT,
			carmen_get_time());
	g_first_mission = 1;
	printf("viewer_3D_dash: missao abortada\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Mirar a camera num lugar do mapa                                                          //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// Os lugares sao as anotaccoes que o viewer_3D ja recebe pela carmen_rddf_annotation_message
// (o vetor 'annotations' do viewer_3D.cpp) -- as mesmas que o navigator_gui2 lista. Nao ha
// arquivo para ler nem mensagem nova para assinar.
static void
load_camera_targets(void)
{
	char *names[DASHBOARD_MAX_MISSIONS];

	g_camera_target_names.clear();
	g_camera_target_points.clear();

	for (unsigned int i = 0; i < annotations.size() && i < DASHBOARD_MAX_MISSIONS; i++)
	{
		char label[DASHBOARD_MISSION_NAME_SIZE];
		const char *description = annotations[i].annotation_description;

		if (description != NULL && description[0] != '\0' && description[0] != ' ')
			snprintf(label, sizeof(label), "%s", description);
		else
			snprintf(label, sizeof(label), "anotação %d (tipo %d)", i, annotations[i].annotation_type);

		g_camera_target_names.push_back(std::string(label));
		g_camera_target_points.push_back(annotations[i].annotation_point);
	}

	for (unsigned int i = 0; i < g_camera_target_names.size(); i++)
		names[i] = (char *) g_camera_target_names[i].c_str();

	dashboard_set_camera_targets(names, (int) g_camera_target_names.size());

	printf("viewer_3D_dash: %d lugares para mirar a câmera\n", (int) g_camera_target_names.size());
}


static void
aim_camera_at(const char *name)
{
	carmen_vector_3D_t offset = get_position_offset();
	carmen_pose_3D_t pose;

	for (unsigned int i = 0; i < g_camera_target_names.size(); i++)
	{
		if (g_camera_target_names[i] != name)
			continue;

		// o mundo e' desenhado em (mundo - offset); a camera mora no mesmo referencial
		pose = car_fused_pose;
		pose.position.x = g_camera_target_points[i].x - offset.x;
		pose.position.y = g_camera_target_points[i].y - offset.y;
		pose.position.z = 0.0;

		follow_car_flag = 0;			// senao o proximo quadro traz a camera de volta ao carro
		set_camera_offset(pose);
		dashboard_set_camera_following(0);

		printf("viewer_3D_dash: câmera mirada em '%s' (%.2f %.2f)\n", name,
				g_camera_target_points[i].x, g_camera_target_points[i].y);

		return;
	}

	printf("viewer_3D_dash: não achei o lugar '%s'\n", name);
}


static void
camera_follow_car(void)
{
	follow_car_flag = 1;
	dashboard_set_camera_following(1);
	printf("viewer_3D_dash: câmera voltou a seguir o carro\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Por o robo / por o destino clicando no mapa                                               //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static carmen_vector_3D_t g_click_start;
static int g_click_active = 0;


// Le' o ponto do mundo sob o cursor. Depende do z-buffer do quadro anterior, entao so'
// funciona onde ha' algo desenhado (o mapa, o chao, a nuvem).
static int
world_point_under_cursor(int x, int y, carmen_vector_3D_t *point)
{
	annotation_point_world.x = 0.0;
	annotation_point_world.y = 0.0;
	annotation_point_world.z = 0.0;

	picking(x, y);

	if (annotation_point_world.x == 0.0 && annotation_point_world.y == 0.0)
		return (0);

	*point = annotation_point_world;

	return (1);
}


static void
publish_robot_pose(double x, double y, double theta)
{
	carmen_point_t pose;
	carmen_point_t std;
	double trailer_theta[MAX_NUM_TRAILERS];

	pose.x = x;
	pose.y = y;
	pose.theta = theta;

	std.x = 0.2;
	std.y = 0.2;
	std.theta = carmen_degrees_to_radians(4.0);

	for (int i = 0; i < MAX_NUM_TRAILERS; i++)
		trailer_theta[i] = theta;

	carmen_localize_ackerman_initialize_gaussian_command(pose, std, trailer_theta,
			semi_trailer_config.num_semi_trailers);

	printf("viewer_3D_dash: robô posto em %.2f %.2f %.3f\n", x, y, theta);
}


// Mesma sequencia do 'set course to' do task_manager, que e' a que funciona:
// task FOLLOW_ROUTE + final goal no rddf + destino para o route_planner.
static void
publish_destination(double x, double y, double theta)
{
	carmen_point_t goal;
	carmen_robot_and_trailers_pose_t goal_with_trailers;

	goal.x = x;
	goal.y = y;
	goal.theta = theta;

	goal_with_trailers.x = x;
	goal_with_trailers.y = y;
	goal_with_trailers.theta = theta;
	goal_with_trailers.num_trailers = semi_trailer_config.num_semi_trailers;
	for (int i = 0; i < MAX_NUM_TRAILERS; i++)
		goal_with_trailers.trailer_theta[i] = theta;

	carmen_behavior_selector_set_task(BEHAVIOR_SELECTOR_FOLLOW_ROUTE, carmen_get_time());
	carmen_rddf_publish_end_point_message(0, goal_with_trailers);
	carmen_route_planner_set_destination((char *) "", goal);

	printf("viewer_3D_dash: destino em %.2f %.2f %.3f -- aperte IR\n", x, y, theta);
}


// Trata o mouse quando o painel esta' esperando um ponto no mapa.
// Devolve 1 se consumiu o evento.
static int
handle_map_pick(int type, int x, int y)
{
	int mode = dashboard_get_mode();
	carmen_vector_3D_t point;

	if (mode == DASHBOARD_MODE_NORMAL)
		return (0);

	if (type == 4)			// ButtonPress: marca o ponto
	{
		if (!world_point_under_cursor(x, y, &point))
		{
			printf("viewer_3D_dash: clique fora do mapa (nada desenhado ali)\n");
			return (1);
		}

		g_click_start = point;
		g_click_active = 1;

		return (1);
	}

	if (type == 5 && g_click_active)	// ButtonRelease: a direccao sai do arrasto
	{
		double theta = car_fused_pose.orientation.yaw;

		g_click_active = 0;

		if (world_point_under_cursor(x, y, &point))
		{
			double dx = point.x - g_click_start.x;
			double dy = point.y - g_click_start.y;

			if (hypot(dx, dy) > 0.5)
				theta = atan2(dy, dx);
		}

		if (mode == DASHBOARD_MODE_PICK_POSE)
			publish_robot_pose(g_click_start.x, g_click_start.y, theta);
		else
			publish_destination(g_click_start.x, g_click_start.y, theta);

		dashboard_set_mode(DASHBOARD_MODE_NORMAL);

		return (1);
	}

	return (1);		// enquanto esta' no modo, o mouse nao gira a camera
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Ganchos de janela                                                                         //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
dash_mouse_func(int type, int button, int x, int y)
{
	char mission[DASHBOARD_MISSION_NAME_SIZE];
	int action = dashboard_mouse(type, button, x, y, window_width, window_height,
			mission, sizeof(mission));

	switch (action)
	{
	case DASHBOARD_ACTION_MISSION:
		execute_mission(mission);
		return;

	case DASHBOARD_ACTION_ABORT:
		abort_mission();
		return;

	case DASHBOARD_ACTION_SET_POSE:
		dashboard_set_mode((dashboard_get_mode() == DASHBOARD_MODE_PICK_POSE) ?
				DASHBOARD_MODE_NORMAL : DASHBOARD_MODE_PICK_POSE);
		g_click_active = 0;
		return;

	case DASHBOARD_ACTION_SET_GOAL:
		dashboard_set_mode((dashboard_get_mode() == DASHBOARD_MODE_PICK_GOAL) ?
				DASHBOARD_MODE_NORMAL : DASHBOARD_MODE_PICK_GOAL);
		g_click_active = 0;
		return;

	case DASHBOARD_ACTION_GO:
		carmen_navigator_ackerman_go();
		printf("viewer_3D_dash: GO\n");
		return;

	case DASHBOARD_ACTION_STOP:
		carmen_navigator_ackerman_stop();
		printf("viewer_3D_dash: STOP\n");
		return;

	case DASHBOARD_ACTION_MISSIONS:
		load_mission_list();
		dashboard_toggle_missions();
		return;

	case DASHBOARD_ACTION_CAMERA_PANEL:
		load_camera_targets();
		dashboard_toggle_camera_panel();
		return;

	case DASHBOARD_ACTION_CAMERA_TARGET:
		aim_camera_at(mission);
		return;

	case DASHBOARD_ACTION_CAMERA_FOLLOW:
		camera_follow_car();
		return;

	case DASHBOARD_ACTION_CONSUMED:
		return;
	}

	if (handle_map_pick(type, x, y))
		return;

	if (g_original_mouse_func != NULL)
		g_original_mouse_func(type, button, x, y);
}


static void
dash_key_press(int code)
{
	switch (code)
	{
	case KEY_F:
		dashboard_toggle_panel();
		return;

	case KEY_G:
		load_mission_list();		// recarrega a pasta a cada abertura
		dashboard_toggle_missions();
		return;

	case KEY_Z:
		load_camera_targets();		// recarrega a lista a cada abertura
		dashboard_toggle_camera_panel();
		return;

	case KEY_H:
		ensure_camera_subscribed();
		dashboard_toggle_camera();
		return;

	case KEY_O:
		dashboard_cycle_position();
		return;

	case KEY_BRACKET_LEFT:
		dashboard_add_scale(-0.1);
		return;

	case KEY_BRACKET_RIGHT:
		dashboard_add_scale(0.1);
		return;

	case KEY_COMMA:
		dashboard_add_opacity(-0.05);
		return;

	case KEY_PERIOD:
		dashboard_add_opacity(0.05);
		return;
	}

	if (g_original_key_press != NULL)
		g_original_key_press(code);
}


static int
dash_process_window(window *w, void (*mouse_func)(int type, int button, int x, int y),
		void (*key_press)(int code), void (*key_release)(int code),
		void (*resize_func)(int width, int height))
{
	g_original_mouse_func = mouse_func;
	g_original_key_press = key_press;

	return (processWindow(w, dash_mouse_func, dash_key_press, key_release, resize_func));
}


// O painel mostra quantos caminhos o frenet publicou, qual foi o escolhido e ha quanto tempo.
// Isso NAO justifica assinar a mensagem de novo: a carmen_frenet_path_planner_set_of_paths
// carrega todos os caminhos e as faixas vizinhas, e uma segunda assinatura faz o IPC
// desempacotar tudo isso mais uma vez por mensagem, dentro deste mesmo processo -- atrasando o
// frenet_path_planner_handler do viewer_3D. E os caminhos so ficam desenhados durante
// persistence_time (0,1 s) depois da mensagem: quando o desenho atrasa, o draw_path() ZERA o
// path_size e o grafo de rota pisca. Os tres numeros saem dos drawers que o proprio
// viewer_3D.cpp ja montou, de graca.
static void
update_frenet_data(void)
{
	dashboard_data_t *d = dashboard_data();

	d->frenet_num_paths = (int) path_plans_frenet_drawer.size();
	d->frenet_selected = -1;

	for (unsigned int i = 0; i < path_plans_frenet_drawer.size(); i++)
	{
		trajectory_drawer *t = path_plans_frenet_drawer[i];

		if (t == NULL)
			continue;

		// o escolhido e o unico azul; o resto do leque e verde (frenet_path_planner_handler)
		if (t->r == 0.0 && t->g == 0.0 && t->b == 1.0)
			d->frenet_selected = (int) i;

		if (t->availability_timestamp > d->t_frenet)
			d->t_frenet = t->availability_timestamp;
	}
}


static void
dash_draw_timer_handler(void)
{
	if (!showWindow(w))
	{
		shutdown_module(0);
		return;
	}

	// O painel desenha com textura e blending; o mundo do viewer_3D espera comecar o quadro
	// como a initGl() deixou -- sem textura ligada e sem nada amarrado. Um so' vazamento aqui
	// pinta o modelo do carro inteiro com a superficie do cairo (ver layer_upload()).
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);

	if (!draw_everything())
	{
		shutdown_module(0);
		return;
	}

	dashboard_data()->max_speed = g_max_velocity;
	dashboard_data()->max_phi = g_max_steering_angle;
	dashboard_data()->steering_ratio = g_steering_ratio;

	update_frenet_data();

	// O menu do proprio viewer_3D mora coladinho na base da janela, exatamente onde o painel
	// fica: sem isto os botoes de Options nascem escondidos embaixo dele.
	{
		static double last_offset = -1.0;
		double offset = dashboard_menu_bottom_offset(window_width, window_height);

		if (fabs(offset - last_offset) > 0.5)
		{
			interface_drawer_set_bottom_offset(offset);
			update_buttons_size(i_drawer, window_width, window_height);
			last_offset = offset;
		}
	}

	dashboard_draw(window_width, window_height);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Inicializaccao                                                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
dash_read_parameters(int argc, char **argv)
{
	carmen_param_t optional[] =
	{
		{(char *) "navigator_panel", (char *) "missions_folder", CARMEN_PARAM_STRING,
				&g_missions_folder, 0, NULL},
	};
	carmen_param_t required[] =
	{
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE,
				&g_axle_distance, 0, NULL},
		{(char *) "robot", (char *) "max_velocity", CARMEN_PARAM_DOUBLE, &g_max_velocity, 0, NULL},
		{(char *) "robot", (char *) "max_steering_angle", CARMEN_PARAM_DOUBLE,
				&g_max_steering_angle, 0, NULL},
	};
	carmen_param_t command_line[] =
	{
		{(char *) "commandline", (char *) "dash_camera", CARMEN_PARAM_INT, &g_dash_camera_id,
				0, NULL},
		{(char *) "commandline", (char *) "dash_scale", CARMEN_PARAM_DOUBLE, &g_dash_scale,
				0, NULL},
		{(char *) "commandline", (char *) "dash_opacity", CARMEN_PARAM_DOUBLE, &g_dash_opacity,
				0, NULL},
		{(char *) "commandline", (char *) "dash_position", CARMEN_PARAM_INT, &g_dash_position,
				0, NULL},
		{(char *) "commandline", (char *) "steering_ratio", CARMEN_PARAM_DOUBLE,
				&g_steering_ratio, 0, NULL},
	};
	carmen_param_t dash_ini[] =
	{
		{(char *) "viewer_3D", (char *) "dash_scale", CARMEN_PARAM_DOUBLE, &g_dash_scale, 0, NULL},
		{(char *) "viewer_3D", (char *) "dash_opacity", CARMEN_PARAM_DOUBLE, &g_dash_opacity,
				0, NULL},
		{(char *) "viewer_3D", (char *) "dash_position", CARMEN_PARAM_INT, &g_dash_position,
				0, NULL},
		{(char *) "viewer_3D", (char *) "steering_ratio", CARMEN_PARAM_DOUBLE,
				&g_steering_ratio, 0, NULL},
	};

	carmen_param_install_params(argc, argv, required,
			sizeof(required) / sizeof(required[0]));

	// a pasta de missoes so' existe nos inis que tem navigator_gui2 configurado
	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional,
			sizeof(optional) / sizeof(optional[0]));
	carmen_param_install_params(argc, argv, dash_ini,
			sizeof(dash_ini) / sizeof(dash_ini[0]));
	carmen_param_install_params(argc, argv, command_line,
			sizeof(command_line) / sizeof(command_line[0]));
	carmen_param_allow_unfound_variables(0);
}


static void
dash_subscribe_messages(void)
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t) dash_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ford_escape_subscribe_status_message(NULL,
			(carmen_handler_t) dash_ford_escape_status_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_status_message(NULL,
			(carmen_handler_t) dash_navigator_status_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL,
			(carmen_handler_t) dash_behavior_selector_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_task_manager_subscribe_mission_state_message(NULL,
			(carmen_handler_t) dash_mission_state_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_motion_command(NULL,
			(carmen_handler_t) dash_motion_command_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL,
			(carmen_handler_t) dash_base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	// A camera NAO e' assinada aqui: ver ensure_camera_subscribed(). E o offline_map tambem
	// nao -- o viewer_3D.cpp ja' o assina no subscribe_ipc_messages() (offline_map_update_handler),
	// e uma segunda assinatura faria o IPC desempacotar o mapa inteiro (varios MB) duas vezes
	// por publicacao, dentro do mesmo processo, so' para imprimir o tamanho.
}


int
main(int argc, char **argv)
{
	argc_g = argc;
	argv_g = argv;

	setvbuf(stdout, NULL, _IOLBF, 0);	// senao as mensagens somem quando o proccontrol mata

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters_and_init_stuff(argc, argv);		// do viewer_3D: cria a janela e os drawers
	dash_read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	subscribe_ipc_messages();						// do viewer_3D
	dash_subscribe_messages();

	printf("viewer_3D_dash: map_mode=%d  (draw_map=%d offline=%d costs=%d remission=%d)\n",
			map_mode, draw_map_flag, draw_offline_map_flag, draw_costs_map_flag,
			draw_remission_map_flag);

	dashboard_init();
	dashboard_set_style(g_dash_scale, g_dash_opacity, g_dash_position);
	load_mission_list();

	if (map_mode == 3)
		request_offline_map();		// nao depende de chegar na hora certa da pose inicial

	glPointSize(point_size);
	lastDisplayTime = carmen_get_time();

	carmen_ipc_addPeriodicTimer(1.0 / 40.0, (TIMER_HANDLER_TYPE) dash_draw_timer_handler, NULL);
	carmen_ipc_dispatch();

	return (0);
}
