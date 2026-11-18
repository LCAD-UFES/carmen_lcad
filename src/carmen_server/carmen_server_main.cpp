/*
 * carmen_server -- ponte entre o IPC do carmen e um app remoto, via ZMQ.
 *
 * Roda no robo, junto com os outros modulos, e nao exige NENHUMA mudanca neles: so' assina
 * mensagens que ja' existem e publica comandos que ja' existem. Tambem faz o papel do
 * proccontrol_gui sem janela (tabela de processos, saida dos modulos, liga/desliga), entao no
 * process ele entra na linha do proc_control no lugar do ./proccontrol_gui.
 *
 *   PUB  porta 5560 (todas as interfaces)  dados    -> multipart [topico, json] ou [topico, json, binario]
 *   REP  porta 5561 (todas as interfaces)  comandos -> pedido json; resposta [json] ou [json, binario]
 *
 * O PUB nunca bloqueia (fila curta + descarte): se a rede do app nao der conta, as mensagens
 * sao descartadas e o robo segue igual. Tudo roda num thread so', dentro do loop do IPC: um
 * timer de 50 Hz atende os comandos e despacha o que estiver pendente.
 *
 * Protocolo completo em PROTOCOLO.md (mesma pasta).
 */

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/route_planner_interface.h>
#include <carmen/proccontrol_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>
#include <carmen/task_manager_interface.h>
#include <carmen/user_app_server_interface.h>
#include <carmen/frenet_path_planner_interface.h>
#include <carmen/offroad_planner_interface.h>
#include <carmen/map_io.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/xsens_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/obstacle_distance_mapper_messages.h>	/* so' a mensagem: a interface puxa o prob_models */

#include <zmq.h>
#include <zlib.h>
#include <dirent.h>

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#define CARMEN_SERVER_PROTOCOL_VERSION 1

static void *zmq_ctx = NULL;
static void *pub_socket = NULL;
static void *rep_socket = NULL;

static int pub_port = 5560;
static int cmd_port = 5561;
static double pose_hz = 10.0;
static double path_hz = 2.0;
static double plan_hz = 5.0;
static double map_min_period = 2.0;
static int max_path_points = 400;
static int output_lines_kept = 300;

static char hostname[256] = "";

/* ------------------------------------------------------------------------------------------ */
/* Estado guardado para o snapshot (quem conecta depois recebe tudo de uma vez)                */
/* ------------------------------------------------------------------------------------------ */

static std::string last_proc_json = "null";
static std::string last_goal_json = "null";
static std::string last_behavior_json = "null";
static std::string last_pose_json = "null";
static std::string last_map_json = "null";
static std::vector<unsigned char> last_map_blob;
static double last_map_sent = -1.0;
static bool map_dirty = false;

static std::string last_car_json = "null";
static std::string last_nav_json = "null";
static std::string last_mission_json = "null";
static std::string last_annotations_json = "null";
static std::string robot_json = "null";
static bool have_goal = false;
static carmen_point_t last_goal = {0.0, 0.0, 0.0};
static bool mission_running = false;

static std::map<std::string, std::deque<std::string> > output_history;
static std::map<std::string, std::string> output_pending;

static double last_pose_sent = 0.0, last_path_sent = 0.0, last_plan_sent = 0.0, last_route_sent = 0.0;
static double last_behavior_sent = 0.0, last_proc_sent = 0.0, last_hello_sent = 0.0, last_output_flush = 0.0;
static double last_odom_sent = 0.0, last_cmd_sent = 0.0, last_car_sent = 0.0, last_nav_sent = 0.0;
static double last_frenet_sent = 0.0, last_lanes_sent = 0.0, last_annotations_sent = 0.0, last_offroad_sent = 0.0;

static std::string missions_folder;		// -missions_folder: pasta com as missoes (.txt) do task_manager
static std::string annotation_path;		// -annotation_path: o mesmo arquivo de anotacoes do navigator_gui2
static std::string map_path;			// -map_path: pasta dos blocos m*.map (para o mapa completo)
static std::string annotation_file_json = "[]";
static std::string full_map_json = "null";
static std::vector<unsigned char> full_map_blob;
static double map_request_at = -1.0;	// quando pedir o mapa ao map_server (<0: nada agendado)
static double start_time = 0.0;
static double last_param_try = 0.0;
static bool robot_params_loaded = false;

/* Watchdog do app: sem nenhum pedido do app por app_watchdog segundos, manda STOP no robo.
   So' arma depois do primeiro pedido (sem app conectado nao faz nada); 0 desliga. */
static double app_watchdog = 5.0;
static double last_app_request = -1.0;
static bool watchdog_fired = false;
static int nav_autonomous = 0;
static double last_watchdog_stop = 0.0;
static double last_globalpos_x = 0, last_globalpos_y = 0, last_globalpos_theta = 0;

/* ------------------------------------------------------------------------------------------ */
/* JSON minimo (escrita) e leitura de chaves de um objeto plano                                */
/* ------------------------------------------------------------------------------------------ */

static std::string
json_escape(const char *s)
{
	std::string out = "\"";
	if (s)
	{
		for (const unsigned char *p = (const unsigned char *) s; *p; p++)
		{
			switch (*p)
			{
			case '"':  out += "\\\""; break;
			case '\\': out += "\\\\"; break;
			case '\n': out += "\\n"; break;
			case '\r': break;
			case '\t': out += "\\t"; break;
			default:
				if (*p < 0x20)
				{
					char buf[8];
					snprintf(buf, sizeof(buf), "\\u%04x", *p);
					out += buf;
				}
				else
					out += (char) *p;
			}
		}
	}
	out += "\"";
	return out;
}

static std::string
num(double v, int decimals = 4)
{
	if (!std::isfinite(v))
		return "null";
	char buf[64];
	snprintf(buf, sizeof(buf), "%.*f", decimals, v);
	return buf;
}

/* Le "chave": valor de um json plano escrito pelo app. Suficiente para o protocolo daqui. */
static const char *
json_find(const std::string &json, const char *key)
{
	std::string k = std::string("\"") + key + "\"";
	size_t pos = json.find(k);
	if (pos == std::string::npos)
		return NULL;
	pos = json.find(':', pos + k.size());
	if (pos == std::string::npos)
		return NULL;
	pos++;
	while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t'))
		pos++;
	return json.c_str() + pos;
}

static bool
json_get_number(const std::string &json, const char *key, double *value)
{
	const char *p = json_find(json, key);
	if (!p)
		return false;
	char *end;
	double v = strtod(p, &end);
	if (end == p)
		return false;
	*value = v;
	return true;
}

static bool
json_get_string(const std::string &json, const char *key, std::string *value)
{
	const char *p = json_find(json, key);
	if (!p || *p != '"')
		return false;
	p++;
	std::string out;
	while (*p && *p != '"')
	{
		if (*p == '\\' && p[1])
			p++;
		out += *p++;
	}
	*value = out;
	return true;
}

/* ------------------------------------------------------------------------------------------ */
/* Envio ZMQ (nunca bloqueia)                                                                  */
/* ------------------------------------------------------------------------------------------ */

/* Ultimo json publicado em cada topico de camada: vai inteiro no snapshot, para quem conecta (ou
   reconecta) ver rota, caminho, goals, frenets etc. na hora -- alguns modulos so' publicam quando
   algo muda, e sem isso o app ficava sem essas camadas ate' a proxima mensagem. */
static std::map<std::string, std::string> last_by_topic;
static const char *cached_topics[] = {"path", "plan", "route", "lanes", "frenets", "offroad", "annotations",
		"objects", "gps", "fused", "imu", "avoider", "odom", "cmd", NULL};

static bool particles_enabled = false;	// -particles on: assina as particulas do localize (mais pesado)

static void
publish(const char *topic, const std::string &json, const void *blob = NULL, size_t blob_size = 0)
{
	if (!blob)
		for (int i = 0; cached_topics[i]; i++)
			if (!strcmp(topic, cached_topics[i]))
			{
				last_by_topic[topic] = json;
				break;
			}
	if (!pub_socket)
		return;
	if (zmq_send(pub_socket, topic, strlen(topic), ZMQ_SNDMORE | ZMQ_DONTWAIT) < 0)
		return;
	if (blob)
	{
		zmq_send(pub_socket, json.data(), json.size(), ZMQ_SNDMORE | ZMQ_DONTWAIT);
		zmq_send(pub_socket, blob, blob_size, ZMQ_DONTWAIT);
	}
	else
		zmq_send(pub_socket, json.data(), json.size(), ZMQ_DONTWAIT);
}

static std::string
traj_points_json(carmen_robot_and_trailers_traj_point_t *poses, int n)
{
	std::string s = "[";
	if (poses && n > 0)
	{
		int step = (n > max_path_points) ? (int) ceil((double) n / max_path_points) : 1;
		bool first = true;
		for (int i = 0; i < n; i += step)
		{
			if (!first)
				s += ",";
			first = false;
			s += "[" + num(poses[i].x, 3) + "," + num(poses[i].y, 3) + "," + num(poses[i].theta, 4) + "," + num(poses[i].v, 3) + "]";
		}
		if ((n - 1) % step != 0)	// garante o ultimo ponto
			s += ",[" + num(poses[n - 1].x, 3) + "," + num(poses[n - 1].y, 3) + "," + num(poses[n - 1].theta, 4) + "," + num(poses[n - 1].v, 3) + "]";
	}
	s += "]";
	return s;
}

/* ------------------------------------------------------------------------------------------ */
/* Handlers IPC                                                                                */
/* ------------------------------------------------------------------------------------------ */

static void
globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	last_pose_json = "{\"t\":" + num(msg->timestamp, 3) +
			",\"x\":" + num(msg->globalpos.x, 3) + ",\"y\":" + num(msg->globalpos.y, 3) +
			",\"theta\":" + num(msg->globalpos.theta, 4) +
			",\"v\":" + num(msg->v, 3) + ",\"phi\":" + num(msg->phi, 4) +
			",\"converged\":" + std::to_string(msg->converged) + "}";

	last_globalpos_x = msg->globalpos.x;
	last_globalpos_y = msg->globalpos.y;
	last_globalpos_theta = msg->globalpos.theta;

	double now = carmen_get_time();
	if (now - last_pose_sent >= 1.0 / pose_hz)
	{
		publish("pose", last_pose_json);
		last_pose_sent = now;
	}
}

static void
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	int n = msg->config.x_size * msg->config.y_size;
	if (n <= 0 || !msg->complete_map)
		return;

	/* Probabilidade de ocupacao 0..1 -> 0..254; desconhecido (<0) -> 255. */
	std::vector<unsigned char> raw(n);
	for (int i = 0; i < n; i++)
	{
		double v = msg->complete_map[i];
		raw[i] = (v < 0.0) ? 255 : (unsigned char) (carmen_clamp(0.0, v, 1.0) * 254.0);
	}

	uLongf zsize = compressBound(n);
	last_map_blob.resize(zsize);
	if (compress2(last_map_blob.data(), &zsize, raw.data(), n, 1) != Z_OK)
		return;
	last_map_blob.resize(zsize);

	last_map_json = "{\"t\":" + num(msg->timestamp, 3) +
			",\"x_origin\":" + num(msg->config.x_origin, 3) + ",\"y_origin\":" + num(msg->config.y_origin, 3) +
			",\"resolution\":" + num(msg->config.resolution, 4) +
			",\"x_size\":" + std::to_string(msg->config.x_size) + ",\"y_size\":" + std::to_string(msg->config.y_size) +
			",\"encoding\":\"zlib-u8\",\"unknown\":255}";
	map_dirty = true;
}

static void
path_goals_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
	double now = carmen_get_time();
	if (now - last_path_sent < 1.0 / path_hz)
		return;
	last_path_sent = now;

	std::string json = "{\"t\":" + num(msg->timestamp, 3) +
			",\"path\":" + traj_points_json(msg->poses, msg->number_of_poses) +
			",\"goals\":" + traj_points_json(msg->goal_list, msg->goal_list_size) + "}";
	publish("path", json);
}

static void
plan_handler(carmen_navigator_ackerman_plan_message *msg)
{
	double now = carmen_get_time();
	if (now - last_plan_sent < 1.0 / plan_hz)
		return;
	last_plan_sent = now;

	publish("plan", "{\"t\":" + num(msg->timestamp, 3) + ",\"path\":" + traj_points_json(msg->path, msg->path_length) + "}");
}

/* Varias faixas (lanes) guardadas uma apos a outra num vetor so', com indice e tamanho de cada uma. */
static std::string
lanes_json(carmen_robot_and_trailers_traj_point_t *lanes, int *indexes, int *sizes, int num_lanes, int points_per_lane)
{
	std::string s = "[";
	int saved_max = max_path_points;
	max_path_points = points_per_lane;
	for (int l = 0; l < num_lanes; l++)
	{
		if (l > 0)
			s += ",";
		s += traj_points_json(lanes + indexes[l], sizes[l]);
	}
	max_path_points = saved_max;
	return s + "]";
}

static void
road_network_handler(carmen_route_planner_road_network_message *msg)
{
	double now = carmen_get_time();
	if (now - last_route_sent >= 1.0)
	{
		last_route_sent = now;
		publish("route", "{\"t\":" + num(msg->timestamp, 3) + ",\"path\":" + traj_points_json(msg->poses, msg->number_of_poses) + "}");
	}

	/* faixas vizinhas do grafo (o que o navigator_gui2 desenha como grafo em volta do carro) */
	if (now - last_lanes_sent >= 1.0 && msg->number_of_nearby_lanes > 0 && msg->nearby_lanes)
	{
		last_lanes_sent = now;
		publish("lanes", "{\"t\":" + num(msg->timestamp, 3) + ",\"lanes\":" +
				lanes_json(msg->nearby_lanes, msg->nearby_lanes_indexes, msg->nearby_lanes_sizes, msg->number_of_nearby_lanes, 120) + "}");
	}
}

static void
end_point_handler(carmen_rddf_end_point_message *msg)
{
	last_goal_json = "{\"t\":" + num(msg->timestamp, 3) +
			",\"x\":" + num(msg->point.x, 3) + ",\"y\":" + num(msg->point.y, 3) + ",\"theta\":" + num(msg->point.theta, 4) + "}";
	last_goal.x = msg->point.x;
	last_goal.y = msg->point.y;
	last_goal.theta = msg->point.theta;
	have_goal = true;
	publish("goal", last_goal_json);
}

/* --- painel (mesmas fontes do viewer_3D_dash) --------------------------------------------- */

static void
odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	double now = carmen_get_time();
	if (now - last_odom_sent < 1.0 / pose_hz)
		return;
	last_odom_sent = now;
	publish("odom", "{\"t\":" + num(msg->timestamp, 3) + ",\"v\":" + num(msg->v, 3) + ",\"phi\":" + num(msg->phi, 4) + "}");
}

static void
motion_command_handler(carmen_base_ackerman_motion_command_message *msg)
{
	double now = carmen_get_time();
	if (msg->num_motion_commands <= 0 || now - last_cmd_sent < 0.2)
		return;
	last_cmd_sent = now;
	publish("cmd", "{\"t\":" + num(msg->timestamp, 3) + ",\"v\":" + num(msg->motion_command[0].v, 3) +
			",\"phi\":" + num(msg->motion_command[0].phi, 4) + "}");
}

static const char *
gear_from_xgv_code(int code)
{
	switch (code)
	{
	case 1:   return "L";
	case 2:   return "D";
	case 128: return "N";
	case 129: return "R";
	}
	return "";
}

static void
ford_escape_status_handler(carmen_ford_escape_status_message *msg)
{
	last_car_json = "{\"t\":" + num(msg->timestamp, 3) +
			",\"throttle\":" + num(msg->g_XGV_throttle, 1) + ",\"brake\":" + num(msg->g_XGV_brakes, 1) +
			",\"steering_pct\":" + num(msg->g_XGV_steering, 1) +
			",\"gear\":" + json_escape(gear_from_xgv_code(msg->g_XGV_gear)) + ",\"gear_raw\":" + std::to_string(msg->g_XGV_gear) +
			",\"engine\":" + std::to_string(msg->g_XGV_main_propulsion) + ",\"parking_brake\":" + std::to_string(msg->g_XGV_parking_brake) +
			",\"turn_signal\":" + std::to_string(msg->g_XGV_turn_signal) + ",\"headlights\":" + std::to_string(msg->g_XGV_headlights_status) +
			",\"horn\":" + std::to_string(msg->g_XGV_horn_status) + "}";
	double now = carmen_get_time();
	if (now - last_car_sent >= 0.2)
	{
		publish("car", last_car_json);
		last_car_sent = now;
	}
}

static void
navigator_status_handler(carmen_navigator_ackerman_status_message *msg)
{
	double dist = msg->goal_set ? hypot(msg->goal.x - msg->robot.x, msg->goal.y - msg->robot.y) : -1.0;
	nav_autonomous = msg->autonomous;
	last_nav_json = "{\"t\":" + num(msg->timestamp, 3) + ",\"autonomous\":" + std::to_string(msg->autonomous) +
			",\"goal_set\":" + std::to_string(msg->goal_set) + ",\"goal_distance\":" + num(dist, 2) + "}";
	double now = carmen_get_time();
	if (now - last_nav_sent >= 0.5)
	{
		publish("nav", last_nav_json);
		last_nav_sent = now;
	}
}

static void
mission_state_handler(carmen_task_manager_mission_state_message *msg)
{
	last_mission_json = "{\"t\":" + num(msg->timestamp, 3) +
			",\"state\":" + json_escape(print_mission_level_state(msg->mission_state)) +
			",\"id\":" + json_escape(msg->mission_id ? msg->mission_id : "") +
			",\"info\":" + json_escape(msg->info ? msg->info : "") + "}";
	publish("mission", last_mission_json);
}

/* --- camadas do navigator_gui2 ------------------------------------------------------------- */

/* circulos de colisao do robo (robot_collision_file), lidos em try_load_robot_params() */
struct CollisionCircle { double x, y, r; };
static std::vector<CollisionCircle> collision_circles;

/* Mapa de distancia ao obstaculo mais proximo (do obstacle_distance_mapper, compacto), em celulas,
   so' para marcar onde cada frenet bate -- o behavior_selector calcula isso mas nao publica. */
static std::vector<unsigned char> distance_cells;	// 255 = longe
static carmen_map_config_t distance_config;
static double last_distance_built = 0.0, last_distance_received = 0.0;

static void
distance_map_handler(carmen_obstacle_distance_mapper_compact_map_message *msg)
{
	double now = carmen_get_time();
	last_distance_received = now;
	if (now - last_distance_built < 0.2 || msg->config.x_size <= 0 || msg->config.y_size <= 0)
		return;
	last_distance_built = now;
	distance_config = msg->config;
	distance_cells.assign((size_t) msg->config.x_size * msg->config.y_size, 255);
	for (int i = 0; i < msg->size; i++)
	{
		int x = msg->coord_x[i], y = msg->coord_y[i];
		if (x < 0 || y < 0 || x >= msg->config.x_size || y >= msg->config.y_size)
			continue;
		double d = hypot((double) msg->x_offset[i], (double) msg->y_offset[i]);
		distance_cells[(size_t) x * msg->config.y_size + y] = (unsigned char) std::min(254.0, d);
	}
}

/* indice do primeiro ponto do caminho em que algum circulo de colisao do robo encosta num obstaculo; -1 = livre */
static int
first_collision(const carmen_robot_and_trailers_traj_point_t *poses, int n)
{
	if (distance_cells.empty() || collision_circles.empty() || carmen_get_time() - last_distance_received > 2.0)
		return -1;
	double res = distance_config.resolution;
	for (int i = 0; i < n; i++)
	{
		double c = cos(poses[i].theta), s = sin(poses[i].theta);
		for (const CollisionCircle &cc : collision_circles)
		{
			double wx = poses[i].x + c * cc.x - s * cc.y, wy = poses[i].y + s * cc.x + c * cc.y;
			int cx = (int) ((wx - distance_config.x_origin) / res), cy = (int) ((wy - distance_config.y_origin) / res);
			if (cx < 0 || cy < 0 || cx >= distance_config.x_size || cy >= distance_config.y_size)
				continue;
			unsigned char d = distance_cells[(size_t) cx * distance_config.y_size + cy];
			if (d != 255 && d * res < cc.r)
				return i;
		}
	}
	return -1;
}

static void
frenet_handler(carmen_frenet_path_planner_set_of_paths *msg)
{
	double now = carmen_get_time();
	if (now - last_frenet_sent < 1.0 / plan_hz || msg->number_of_poses <= 0)
		return;
	last_frenet_sent = now;

	int num_paths = msg->set_of_paths_size / msg->number_of_poses;
	int saved_max = max_path_points;
	max_path_points = 40;	// leque de caminhos: poucos pontos por caminho basta para desenhar
	std::string json = "{\"t\":" + num(msg->timestamp, 3) + ",\"selected\":" + std::to_string(msg->selected_path) + ",\"paths\":[";
	for (int i = 0; i < num_paths; i++)
	{
		if (i > 0)
			json += ",";
		json += traj_points_json(msg->set_of_paths + i * msg->number_of_poses, msg->number_of_poses);
	}
	max_path_points = saved_max;
	/* onde cada caminho bate (fracao do comprimento: o app tem menos pontos que a mensagem) */
	json += "],\"collision\":[";
	for (int i = 0; i < num_paths; i++)
	{
		int k = first_collision(msg->set_of_paths + i * msg->number_of_poses, msg->number_of_poses);
		json += std::string(i ? "," : "") + (k < 0 ? std::string("-1") : num((double) k / std::max(1, msg->number_of_poses - 1), 3));
	}
	json += "],\"rddf\":" + traj_points_json(msg->rddf_poses_ahead, msg->number_of_poses) + "}";
	publish("frenets", json);
}

static void
annotation_handler(carmen_rddf_annotation_message *msg)
{
	std::string json = "[";
	for (int i = 0; i < msg->num_annotations; i++)
	{
		carmen_annotation_t *a = &msg->annotations[i];
		if (i > 0)
			json += ",";
		json += "{\"x\":" + num(a->annotation_point.x, 2) + ",\"y\":" + num(a->annotation_point.y, 2) +
				",\"theta\":" + num(a->annotation_orientation, 3) + ",\"type\":" + std::to_string(a->annotation_type) +
				",\"code\":" + std::to_string(a->annotation_code) + ",\"text\":" + json_escape(a->annotation_description) + "}";
	}
	json += "]";

	double now = carmen_get_time();
	if (json != last_annotations_json || now - last_annotations_sent > 2.0)
	{
		publish("annotations", "{\"t\":" + num(msg->timestamp, 3) + ",\"items\":" + json + "}");
		last_annotations_sent = now;
	}
	last_annotations_json = json;
}

static void
offroad_plan_handler(carmen_offroad_planner_plan_message *msg)
{
	double now = carmen_get_time();
	if (now - last_offroad_sent < 1.0)
		return;
	last_offroad_sent = now;
	publish("offroad", "{\"t\":" + num(msg->timestamp, 3) + ",\"path\":" + traj_points_json(msg->poses, msg->number_of_poses) + "}");
}

/* --- funcoes leves do viewer_3D ------------------------------------------------------------- */

static double last_objects_sent = 0.0, last_gps_sent = 0.0, last_fused_sent = 0.0, last_imu_sent = 0.0;
static double last_avoider_sent = 0.0, last_particles_sent = 0.0;

static void
moving_objects_handler(carmen_moving_objects_point_clouds_message *msg)
{
	double now = carmen_get_time();
	if (now - last_objects_sent < 0.2)
		return;
	last_objects_sent = now;
	/* so' a caixa de cada objeto: os pontos ficam no robo */
	std::string json = "{\"t\":" + num(msg->timestamp, 3) + ",\"items\":[";
	for (int i = 0; i < msg->num_point_clouds; i++)
	{
		t_point_cloud_struct *o = &msg->point_clouds[i];
		json += std::string(i ? "," : "") + "{\"x\":" + num(o->object_pose.x, 2) + ",\"y\":" + num(o->object_pose.y, 2) +
				",\"theta\":" + num(o->orientation, 3) + ",\"l\":" + num(o->length, 2) + ",\"w\":" + num(o->width, 2) +
				",\"h\":" + num(o->height, 2) + ",\"v\":" + num(o->linear_velocity, 2) +
				",\"id\":" + std::to_string(o->num_associated) + ",\"model\":" + std::to_string(o->geometric_model) + "}";
	}
	publish("objects", json + "]}");
}

static void
gps_xyz_handler(carmen_gps_xyz_message *msg)
{
	double now = carmen_get_time();
	if (now - last_gps_sent < 0.5)
		return;
	last_gps_sent = now;
	publish("gps", "{\"t\":" + num(msg->timestamp, 3) + ",\"x\":" + num(msg->x, 3) + ",\"y\":" + num(msg->y, 3) +
			",\"z\":" + num(msg->z, 2) + ",\"quality\":" + std::to_string(msg->gps_quality) +
			",\"sats\":" + std::to_string(msg->num_satellites) + ",\"hdop\":" + num(msg->hdop, 2) + "}");
}

static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	double now = carmen_get_time();
	if (now - last_fused_sent < 0.2)
		return;
	last_fused_sent = now;
	publish("fused", "{\"t\":" + num(msg->timestamp, 3) + ",\"x\":" + num(msg->pose.position.x, 3) +
			",\"y\":" + num(msg->pose.position.y, 3) + ",\"theta\":" + num(msg->pose.orientation.yaw, 4) +
			",\"roll\":" + num(msg->pose.orientation.roll, 4) + ",\"pitch\":" + num(msg->pose.orientation.pitch, 4) + "}");
}

static void
xsens_quat_handler(carmen_xsens_global_quat_message *msg)
{
	double now = carmen_get_time();
	if (now - last_imu_sent < 0.2)
		return;
	last_imu_sent = now;
	/* m_data = (w, x, y, z) */
	double w = msg->quat_data.m_data[0], x = msg->quat_data.m_data[1], y = msg->quat_data.m_data[2], z = msg->quat_data.m_data[3];
	double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
	double sp = carmen_clamp(-1.0, 2.0 * (w * y - z * x), 1.0);
	double pitch = asin(sp);
	double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
	publish("imu", "{\"t\":" + num(msg->timestamp, 3) + ",\"roll\":" + num(roll, 4) + ",\"pitch\":" + num(pitch, 4) +
			",\"yaw\":" + num(yaw, 4) + ",\"acc_x\":" + num(msg->m_acc.x, 3) + ",\"acc_y\":" + num(msg->m_acc.y, 3) +
			",\"gyr_z\":" + num(msg->m_gyr.z, 4) + "}");
}

static void
obstacle_avoider_path_handler(carmen_navigator_ackerman_plan_message *msg)
{
	double now = carmen_get_time();
	if (now - last_avoider_sent < 1.0 / plan_hz)
		return;
	last_avoider_sent = now;
	publish("avoider", "{\"t\":" + num(msg->timestamp, 3) + ",\"path\":" + traj_points_json(msg->path, msg->path_length) + "}");
}

static void
particles_handler(carmen_localize_ackerman_particle_message *msg)
{
	double now = carmen_get_time();
	if (now - last_particles_sent < 0.5)
		return;
	last_particles_sent = now;
	int step = std::max(1, msg->num_particles / 300);	// no maximo ~300 particulas
	std::string json = "{\"t\":" + num(msg->timestamp, 3) + ",\"items\":[";
	bool first = true;
	for (int i = 0; i < msg->num_particles; i += step)
	{
		json += std::string(first ? "" : ",") + "[" + num(msg->particles[i].x, 2) + "," + num(msg->particles[i].y, 2) + "," +
				num(msg->particles[i].theta, 3) + "]";
		first = false;
	}
	publish("particles", json + "]}");
}

/* --- parametros do robo (para o app desenhar o tamanho certo e as escalas do painel) -------- */

/* Modelo de colisao (robot_collision_file, relativo ao bin/): n, uma linha a mais, e n linhas "x y raio nivel".
   x e' a partir do eixo traseiro, como o obstacle_avoider usa. */
static std::string
collision_circles_json(const char *file)
{
	if (!file || !file[0])
		return "[]";
	FILE *f = fopen(file, "r");
	if (!f)
		return "[]";
	int n = 0, extra = 0;
	std::string json = "[";
	collision_circles.clear();
	if (fscanf(f, "%d %d", &n, &extra) == 2)
	{
		for (int i = 0; i < n && i < 200; i++)
		{
			double x, y, r;
			int level;
			if (fscanf(f, "%lf %lf %lf %d", &x, &y, &r, &level) != 4)
				break;
			json += std::string(i ? "," : "") + "[" + num(x, 3) + "," + num(y, 3) + "," + num(r, 3) + "]";
			collision_circles.push_back({x, y, r});
		}
	}
	fclose(f);
	return json + "]";
}

static void
try_load_robot_params()
{
	double length = 0.70, width = 0.40, axle = 0.35, rear = 0.15, max_v = 1.0, max_phi = 0.6;
	char *collision_file = NULL;
	carmen_param_allow_unfound_variables(1);
	carmen_param_t list[] = {
		{(char *) "robot", (char *) "collision_file", CARMEN_PARAM_STRING, &collision_file, 0, NULL},
		{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &length, 0, NULL},
		{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &width, 0, NULL},
		{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &axle, 0, NULL},
		{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &rear, 0, NULL},
		{(char *) "robot", (char *) "max_velocity", CARMEN_PARAM_DOUBLE, &max_v, 0, NULL},
		{(char *) "robot", (char *) "max_steering_angle", CARMEN_PARAM_DOUBLE, &max_phi, 0, NULL},
	};
	char *argv0[] = {(char *) "carmen_server", NULL};
	carmen_param_install_params(1, argv0, list, sizeof(list) / sizeof(list[0]));
	robot_json = "{\"length\":" + num(length, 3) + ",\"width\":" + num(width, 3) + ",\"axle\":" + num(axle, 3) +
			",\"rear_overhang\":" + num(rear, 3) + ",\"max_v\":" + num(max_v, 3) + ",\"max_phi\":" + num(max_phi, 4) +
			",\"collision\":" + collision_circles_json(collision_file) + "}";
	robot_params_loaded = true;
	publish("robot", robot_json);
}

/* --- modelo 3D do carro (o mesmo .obj do viewer_3D: parametros carmodel_* do carmen.ini) ---- */

static std::string car_model_json;
static std::vector<unsigned char> car_model_blob;

static std::string
load_car_model()
{
	char *file = NULL;
	double size_x = 0.65, size_y = 0.65, size_z = 0.65, yaw = 0.0;
	carmen_param_allow_unfound_variables(1);
	carmen_param_t list[] = {
		{(char *) "carmodel", (char *) "file_name", CARMEN_PARAM_STRING, &file, 0, NULL},
		{(char *) "carmodel", (char *) "size_x", CARMEN_PARAM_DOUBLE, &size_x, 0, NULL},
		{(char *) "carmodel", (char *) "size_y", CARMEN_PARAM_DOUBLE, &size_y, 0, NULL},
		{(char *) "carmodel", (char *) "size_z", CARMEN_PARAM_DOUBLE, &size_z, 0, NULL},
		{(char *) "carmodel", (char *) "yaw", CARMEN_PARAM_DOUBLE, &yaw, 0, NULL},
	};
	char *argv0[] = {(char *) "carmen_server", NULL};
	carmen_param_install_params(1, argv0, list, sizeof(list) / sizeof(list[0]));
	if (!file || !file[0])
		return "carmodel_file_name nao esta' no carmen.ini";

	FILE *f = fopen(file, "rb");	/* relativo ao bin/, como no viewer_3D */
	if (!f)
		return std::string("nao consegui abrir ") + file;
	std::vector<unsigned char> raw;
	unsigned char buf[65536];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
		raw.insert(raw.end(), buf, buf + n);
	fclose(f);

	uLongf zsize = compressBound(raw.size());
	car_model_blob.resize(zsize);
	if (compress2(car_model_blob.data(), &zsize, raw.data(), raw.size(), 6) != Z_OK)
		return "falha ao comprimir o modelo";
	car_model_blob.resize(zsize);
	car_model_json = "{\"file\":" + json_escape(file) + ",\"size\":[" + num(size_x, 3) + "," + num(size_y, 3) + "," + num(size_z, 3) +
			"],\"yaw\":" + num(yaw, 4) + ",\"bytes\":" + std::to_string(raw.size()) + ",\"encoding\":\"zlib-obj\"}";
	fprintf(stderr, "carmen_server: modelo do carro %s (%lu -> %lu bytes)\n", file, (unsigned long) raw.size(), (unsigned long) zsize);
	return "";
}

/* --- missoes do task_manager (arquivos .txt de uma pasta, como no viewer_3D_dash) ----------- */

static std::vector<std::string>
list_missions()
{
	std::vector<std::string> names;
	if (missions_folder.empty())
		return names;
	DIR *dir = opendir(missions_folder.c_str());
	if (!dir)
		return names;
	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL)
	{
		std::string name = entry->d_name;
		if (name[0] != '.' && name.size() > 4 && name.compare(name.size() - 4, 4, ".txt") == 0)
			names.push_back(name.substr(0, name.size() - 4));
	}
	closedir(dir);
	std::sort(names.begin(), names.end());
	return names;
}

static const char *
task_name(int task)
{
	static const char *names[] = {"FOLLOW_ROUTE", "PARK", "PARK_SEMI_TRAILER", "PARK_TRUCK_SEMI_TRAILER",
			"MOVE_TO_ENGAGE_POSE", "GET_ENGAGE_POSE", "HUMAN_INTERVENTION"};
	if (task >= 0 && task < (int) (sizeof(names) / sizeof(names[0])))
		return names[task];
	return "UNKNOWN";
}

static void
behavior_state_handler(carmen_behavior_selector_state_message *msg)
{
	static std::string last_key;
	std::string key = ",\"task\":" + json_escape(task_name(msg->task)) +
			",\"state\":" + json_escape(get_low_level_state_name(msg->low_level_state)) +
			",\"route_planner_state\":" + std::to_string((int) msg->route_planner_state) +
			",\"task_code\":" + std::to_string((int) msg->task) + ",\"algorithm\":" + std::to_string((int) msg->algorithm) + "}";
	last_behavior_json = "{\"t\":" + num(msg->timestamp, 3) + key;

	/* manda quando o estado muda, e no maximo a cada 1 s se nao mudar */
	double now = carmen_get_time();
	if (key != last_key || now - last_behavior_sent > 1.0)
	{
		publish("behavior", last_behavior_json);
		last_behavior_sent = now;
		last_key = key;
	}
}

static void
pidtable_handler(carmen_proccontrol_pidtable_message *msg)
{
	std::string json = "{\"t\":" + num(msg->timestamp, 3) + ",\"host\":" + json_escape(msg->host) + ",\"processes\":[";
	for (int i = 0; i < msg->num_processes; i++)
	{
		carmen_proccontrol_process_t *p = &msg->process[i];
		if (i > 0)
			json += ",";
		json += "{\"group\":" + json_escape(p->group_name) + ",\"module\":" + json_escape(p->module_name) +
				",\"active\":" + std::to_string(p->active) + ",\"requested\":" + std::to_string(p->requested_state) +
				",\"pid\":" + std::to_string(p->pid) + "}";
	}
	json += "]}";
	last_proc_json = json;

	double now = carmen_get_time();
	if (now - last_proc_sent >= 0.5)
	{
		publish("proc", last_proc_json);
		last_proc_sent = now;
	}
}

static void
output_handler(carmen_proccontrol_output_message *msg)
{
	if (!msg->module_name || !msg->output)
		return;
	std::string module = msg->module_name;
	output_pending[module] += msg->output;

	/* historico por linha, para o get_output */
	std::deque<std::string> &hist = output_history[module];
	const char *start = msg->output;
	for (const char *p = msg->output; ; p++)
	{
		if (*p == '\n' || *p == '\0')
		{
			if (p > start)
				hist.push_back(std::string(start, p - start));
			if (*p == '\0')
				break;
			start = p + 1;
		}
	}
	while ((int) hist.size() > output_lines_kept)
		hist.pop_front();
}

/* --- mapa: o map_server so' publica na 1a pose e na troca de bloco; ele responde a um pedido -- */

static bool
request_offline_map()
{
	static bool defined = false;
	if (!defined)
	{
		IPC_defineMsg(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_FMT);
		defined = true;
	}
	carmen_mapper_map_message *response = NULL;
	IPC_RETURN_TYPE err = IPC_queryResponseData(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME,
			carmen_default_message_create(), (void **) &response, 3000);
	if (err != IPC_OK || response == NULL)
	{
		fprintf(stderr, "carmen_server: o map_server nao respondeu ao pedido de mapa\n");
		return false;
	}
	offline_map_handler(response);
	free(response->complete_map);
	free(response->config.map_name);
	free(response->host);
	free(response);
	return true;
}

/* --- anotacoes do arquivo (mesmo formato que o navigator_gui2 le) -------------------------- */

static void
load_annotation_file()
{
	if (annotation_path.empty())
		return;
	FILE *f = fopen(annotation_path.c_str(), "r");
	if (!f)
	{
		fprintf(stderr, "carmen_server: nao consegui abrir o arquivo de anotacoes %s\n", annotation_path.c_str());
		return;
	}
	char line[1024], description[1024];
	int type, code, n = 0;
	double theta, x, y, z;
	std::string json = "[";
	while (fgets(line, sizeof(line), f))
	{
		if (sscanf(line, "%1023s %d %d %lf %lf %lf %lf", description, &type, &code, &theta, &x, &y, &z) != 7 || description[0] == '#')
			continue;
		json += std::string(n++ ? "," : "") + "{\"x\":" + num(x, 2) + ",\"y\":" + num(y, 2) + ",\"theta\":" + num(theta, 3) +
				",\"type\":" + std::to_string(type) + ",\"code\":" + std::to_string(code) + ",\"text\":" + json_escape(description) + "}";
	}
	fclose(f);
	annotation_file_json = json + "]";
	fprintf(stderr, "carmen_server: %d anotacoes em %s\n", n, annotation_path.c_str());
}

/* --- mapa completo: mosaico de todos os blocos m<x>_<y>.map da pasta do mapa ---------------- */

static std::string
build_full_map(int max_side)
{
	if (map_path.empty())
		return "pasta do mapa nao informada (-map_path)";
	DIR *dir = opendir(map_path.c_str());
	if (!dir)
		return "nao consegui abrir " + map_path;

	struct Block { int x, y; carmen_map_t map; };
	std::vector<Block> blocks;
	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL)
	{
		int bx, by;
		char tail[8];
		if (sscanf(entry->d_name, "m%d_%d%7s", &bx, &by, tail) != 3 || strcmp(tail, ".map") != 0)
			continue;
		Block b;
		b.x = bx;
		b.y = by;
		memset(&b.map, 0, sizeof(b.map));
		std::string file = map_path + "/" + entry->d_name;
		if (carmen_map_read_gridmap_chunk_verbose((char *) file.c_str(), &b.map, 0) == -1 || !b.map.complete_map)
			continue;
		blocks.push_back(b);
	}
	closedir(dir);
	if (blocks.empty())
		return "nenhum bloco m*.map em " + map_path;

	double res = blocks[0].map.config.resolution;
	double x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
	for (size_t i = 0; i < blocks.size(); i++)
	{
		x0 = std::min(x0, (double) blocks[i].x);
		y0 = std::min(y0, (double) blocks[i].y);
		x1 = std::max(x1, blocks[i].x + blocks[i].map.config.x_size * res);
		y1 = std::max(y1, blocks[i].y + blocks[i].map.config.y_size * res);
	}
	int full_x = (int) ceil((x1 - x0) / res), full_y = (int) ceil((y1 - y0) / res);
	int factor = std::max(1, (int) ceil((double) std::max(full_x, full_y) / max_side));
	int out_x = (full_x + factor - 1) / factor, out_y = (full_y + factor - 1) / factor;

	/* reduz pegando a celula mais ocupada (nao some obstaculo fino); 255 = desconhecido */
	std::vector<unsigned char> grid((size_t) out_x * out_y, 255);
	for (size_t i = 0; i < blocks.size(); i++)
	{
		carmen_map_t &m = blocks[i].map;
		int ox = (int) lround((blocks[i].x - x0) / res), oy = (int) lround((blocks[i].y - y0) / res);
		for (int cx = 0; cx < m.config.x_size; cx++)
			for (int cy = 0; cy < m.config.y_size; cy++)
			{
				double v = m.complete_map[cx * m.config.y_size + cy];
				if (v < 0.0)
					continue;
				int gx = (ox + cx) / factor, gy = (oy + cy) / factor;
				if (gx < 0 || gx >= out_x || gy < 0 || gy >= out_y)
					continue;
				unsigned char u = (unsigned char) (carmen_clamp(0.0, v, 1.0) * 254.0);
				unsigned char &dst = grid[(size_t) gx * out_y + gy];
				if (dst == 255 || u > dst)
					dst = u;
			}
		free(m.complete_map);
		free(m.map);
		free(m.config.map_name);
	}

	uLongf zsize = compressBound(grid.size());
	full_map_blob.resize(zsize);
	if (compress2(full_map_blob.data(), &zsize, grid.data(), grid.size(), 6) != Z_OK)
		return "falha ao comprimir";
	full_map_blob.resize(zsize);
	full_map_json = "{\"x_origin\":" + num(x0, 3) + ",\"y_origin\":" + num(y0, 3) + ",\"resolution\":" + num(res * factor, 4) +
			",\"x_size\":" + std::to_string(out_x) + ",\"y_size\":" + std::to_string(out_y) +
			",\"blocks\":" + std::to_string(blocks.size()) + ",\"encoding\":\"zlib-u8\",\"unknown\":255}";
	fprintf(stderr, "carmen_server: mapa completo %dx%d (res %.2f, %d blocos, %lu bytes)\n",
			out_x, out_y, res * factor, (int) blocks.size(), (unsigned long) full_map_blob.size());
	return "";
}

/* ------------------------------------------------------------------------------------------ */
/* Comandos (REP)                                                                              */
/* ------------------------------------------------------------------------------------------ */

#include "carmen_server_extra_layers.h"

static std::string
layers_json()
{
	std::string json = "{";
	bool first = true;
	for (std::map<std::string, std::string>::iterator it = last_by_topic.begin(); it != last_by_topic.end(); ++it)
	{
		json += std::string(first ? "" : ",") + json_escape(it->first.c_str()) + ":" + it->second;
		first = false;
	}
	return json + "}";
}

static std::string
handle_command(const std::string &req, std::vector<unsigned char> **blob_out)
{
	std::string cmd;
	*blob_out = NULL;
	if (!json_get_string(req, "cmd", &cmd))
		return "{\"ok\":false,\"error\":\"pedido sem cmd\"}";

	if (cmd == "ping")
		return "{\"ok\":true,\"version\":" + std::to_string(CARMEN_SERVER_PROTOCOL_VERSION) + ",\"host\":" + json_escape(hostname) + "}";

	if (cmd == "snapshot")
	{
		if (!last_map_blob.empty())
			*blob_out = &last_map_blob;
		return "{\"ok\":true,\"version\":" + std::to_string(CARMEN_SERVER_PROTOCOL_VERSION) +
				",\"host\":" + json_escape(hostname) +
				",\"pose\":" + last_pose_json + ",\"proc\":" + last_proc_json + ",\"goal\":" + last_goal_json +
				",\"behavior\":" + last_behavior_json + ",\"car\":" + last_car_json + ",\"nav\":" + last_nav_json +
				",\"mission\":" + last_mission_json + ",\"robot\":" + robot_json +
				",\"annotation_file\":" + annotation_file_json + ",\"layers\":" + layers_json() +
				",\"map\":" + (last_map_blob.empty() ? std::string("null") : last_map_json) + "}";
	}

	if (cmd == "get_map")
	{
		double refresh = 0;
		json_get_number(req, "refresh", &refresh);
		if (refresh != 0)
			request_offline_map();
		if (last_map_blob.empty())
			return "{\"ok\":false,\"error\":\"ainda nao ha mapa\"}";
		*blob_out = &last_map_blob;
		map_dirty = false;
		return "{\"ok\":true,\"map\":" + last_map_json + "}";
	}

	if (cmd == "full_map")
	{
		double max_side = 3000;
		json_get_number(req, "max_side", &max_side);
		if (full_map_blob.empty())
		{
			std::string err = build_full_map((int) carmen_clamp(500, max_side, 8000));
			if (!err.empty())
				return "{\"ok\":false,\"error\":" + json_escape(err.c_str()) + "}";
		}
		*blob_out = &full_map_blob;
		return "{\"ok\":true,\"map\":" + full_map_json + "}";
	}

	if (cmd == "behavior")
	{
		/* os combos do navigator_gui2: tarefa (Follow Route, Park...) e planejador (Frenet, A*, RRT, RDDF, Gradient) */
		double task = -1, algorithm = -1;
		json_get_number(req, "task", &task);
		json_get_number(req, "algorithm", &algorithm);
		if (task < BEHAVIOR_SELECTOR_FOLLOW_ROUTE || task > BEHAVIOR_SELECTOR_HUMAN_INTERVENTION)
			return "{\"ok\":false,\"error\":\"tarefa invalida\"}";
		if (algorithm >= CARMEN_BEHAVIOR_SELECTOR_FRENET && algorithm < CARMEN_BEHAVIOR_SELECTOR_INVALID_PLANNER)
			carmen_behavior_selector_set_algorithm((carmen_behavior_selector_algorithm_t) (int) algorithm,
					(carmen_behavior_selector_task_t) (int) task, carmen_get_time());
		else
			carmen_behavior_selector_set_task((carmen_behavior_selector_task_t) (int) task, carmen_get_time());
		return "{\"ok\":true}";
	}

	if (cmd == "map_layer")
		return command_map_layer(req);

	if (cmd == "lidar")
		return command_lidar(req);

	if (cmd == "camera")
		return command_camera(req);

	if (cmd == "car_model")
	{
		if (car_model_blob.empty())
		{
			std::string err = load_car_model();
			if (!err.empty())
				return "{\"ok\":false,\"error\":" + json_escape(err.c_str()) + "}";
		}
		*blob_out = &car_model_blob;
		return "{\"ok\":true,\"model\":" + car_model_json + "}";
	}

	if (cmd == "annotations")
		return "{\"ok\":true,\"items\":" + annotation_file_json + "}";

	if (cmd == "go")
	{
		carmen_navigator_ackerman_go();
		return "{\"ok\":true}";
	}

	if (cmd == "stop")
	{
		carmen_navigator_ackerman_stop();
		return "{\"ok\":true}";
	}

	if (cmd == "compute_route")
	{
		/* botao Compute Route do navigator_gui2: rota ate' o destino que esta' marcado */
		if (!have_goal)
			return "{\"ok\":false,\"error\":\"nenhum destino marcado\"}";
		char destination[] = "";
		carmen_route_planner_set_destination(destination, last_goal);
		return "{\"ok\":true}";
	}

	if (cmd == "missions")
	{
		std::vector<std::string> names = list_missions();
		std::string json = "{\"ok\":true,\"folder\":" + json_escape(missions_folder.c_str()) + ",\"missions\":[";
		for (size_t i = 0; i < names.size(); i++)
			json += (i ? "," : "") + json_escape(names[i].c_str());
		return json + "]}";
	}

	if (cmd == "execute_mission")
	{
		std::string name;
		if (!json_get_string(req, "name", &name) || name.find('/') != std::string::npos)
			return "{\"ok\":false,\"error\":\"nome de missao invalido\"}";
		std::string path = missions_folder + "/" + name + ".txt";
		FILE *f = fopen(path.c_str(), "r");
		if (!f)
			return "{\"ok\":false,\"error\":" + json_escape(("nao consegui ler " + path).c_str()) + "}";
		std::string text;
		char line[2048];
		while (fgets(line, sizeof(line), f))
			text += line;
		fclose(f);
		if (text.empty())
			return "{\"ok\":false,\"error\":\"missao vazia\"}";
		/* mesma sequencia do navigator_gui2/viewer_3D_dash: aborta a anterior antes da nova */
		if (mission_running)
			carmen_user_app_server_publish_update_mission_message(USER_APP_SERVER_MISSION_ABORT, carmen_get_time());
		carmen_user_app_server_publish_execute_mission_message(text.c_str(), carmen_get_time());
		mission_running = true;
		return "{\"ok\":true}";
	}

	if (cmd == "abort_mission")
	{
		carmen_user_app_server_publish_update_mission_message(USER_APP_SERVER_MISSION_ABORT, carmen_get_time());
		mission_running = false;
		return "{\"ok\":true}";
	}

	double x, y, theta = 0.0;
	if (cmd == "initial_pose" || cmd == "final_goal" || cmd == "route_to")
	{
		if (!json_get_number(req, "x", &x) || !json_get_number(req, "y", &y))
			return "{\"ok\":false,\"error\":\"faltam x/y\"}";
		json_get_number(req, "theta", &theta);

		if (cmd == "initial_pose")
		{
			/* igual ao util_publish_initial_pose */
			carmen_point_t pose = {x, y, theta};
			carmen_point_t std_dev = {0.001, 0.001, carmen_degrees_to_radians(0.01)};
			double trailer_theta[MAX_NUM_TRAILERS];
			for (int i = 0; i < MAX_NUM_TRAILERS; i++)
				trailer_theta[i] = theta;
			carmen_localize_ackerman_initialize_gaussian_command(pose, std_dev, trailer_theta, 1);
			map_request_at = carmen_get_time() + 1.0;	// o map_server pode nao republicar: pede o mapa do lugar novo
		}
		else if (cmd == "final_goal")
		{
			/* igual ao util_publish_final_goal / navigator_gui2 */
			carmen_robot_and_trailers_pose_t pose;
			memset(&pose, 0, sizeof(pose));
			pose.x = x;
			pose.y = y;
			pose.theta = theta;
			pose.num_trailers = 0;
			for (int i = 0; i < MAX_NUM_TRAILERS; i++)
				pose.trailer_theta[i] = theta;
			carmen_rddf_publish_end_point_message(50, pose);
		}
		else
		{
			/* igual ao botao Compute Route do navigator_gui2 */
			carmen_point_t point = {x, y, theta};
			char destination[] = "";
			carmen_route_planner_set_destination(destination, point);
		}
		return "{\"ok\":true}";
	}

	if (cmd == "module" || cmd == "group")
	{
		std::string name;
		double state;
		if (!json_get_string(req, "name", &name) || !json_get_number(req, "state", &state))
			return "{\"ok\":false,\"error\":\"faltam name/state\"}";
		if (cmd == "module")
			carmen_proccontrol_set_module_state((char *) name.c_str(), (int) state);
		else
			carmen_proccontrol_set_group_state((char *) name.c_str(), (int) state);
		return "{\"ok\":true}";
	}

	if (cmd == "get_output")
	{
		std::string name;
		if (!json_get_string(req, "name", &name))
			return "{\"ok\":false,\"error\":\"falta name\"}";
		std::string json = "{\"ok\":true,\"module\":" + json_escape(name.c_str()) + ",\"lines\":[";
		std::deque<std::string> &hist = output_history[name];
		for (size_t i = 0; i < hist.size(); i++)
			json += (i ? "," : "") + json_escape(hist[i].c_str());
		return json + "]}";
	}

	return "{\"ok\":false,\"error\":" + json_escape(("comando desconhecido: " + cmd).c_str()) + "}";
}

static void
poll_commands()
{
	for (int i = 0; i < 20; i++)	// atende ate' 20 pedidos por ciclo
	{
		zmq_msg_t req;
		zmq_msg_init(&req);
		if (zmq_msg_recv(&req, rep_socket, ZMQ_DONTWAIT) < 0)
		{
			zmq_msg_close(&req);
			return;
		}
		std::string text((const char *) zmq_msg_data(&req), zmq_msg_size(&req));
		last_app_request = carmen_get_time();
		if (watchdog_fired)
		{
			watchdog_fired = false;
			fprintf(stderr, "carmen_server: app voltou; watchdog rearmado\n");
			publish("watchdog", "{\"state\":\"ok\"}");
		}
		/* descarta frames extras de um pedido multipart */
		int more = 0;
		size_t more_size = sizeof(more);
		zmq_getsockopt(rep_socket, ZMQ_RCVMORE, &more, &more_size);
		zmq_msg_close(&req);
		while (more)
		{
			zmq_msg_t extra;
			zmq_msg_init(&extra);
			zmq_msg_recv(&extra, rep_socket, 0);
			zmq_msg_close(&extra);
			zmq_getsockopt(rep_socket, ZMQ_RCVMORE, &more, &more_size);
		}

		std::vector<unsigned char> *blob = NULL;
		std::string reply = handle_command(text, &blob);
		if (blob)
		{
			zmq_send(rep_socket, reply.data(), reply.size(), ZMQ_SNDMORE);
			zmq_send(rep_socket, blob->data(), blob->size(), 0);
		}
		else
			zmq_send(rep_socket, reply.data(), reply.size(), 0);
	}
}

/* ------------------------------------------------------------------------------------------ */
/* Timer de 50 Hz: comandos + despacho do que esta' pendente                                    */
/* ------------------------------------------------------------------------------------------ */

static void
periodic_timer(void *clientData __attribute__((unused)), unsigned long currentTime __attribute__((unused)),
		unsigned long scheduledTime __attribute__((unused)))
{
	poll_commands();

	double now = carmen_get_time();

	/* pedido de mapa agendado (pose nova) ou nenhum mapa ainda 3 s depois de subir */
	if ((map_request_at > 0 && now >= map_request_at) ||
		(last_map_blob.empty() && map_request_at < 0 && now - start_time > 3.0 && now - last_param_try > 4.0))
	{
		map_request_at = -1.0;
		request_offline_map();
	}

	/* o param_daemon sobe junto com este modulo: tenta ler os parametros do robo depois de 2 s */
	if (!robot_params_loaded && now - start_time > 2.0 && now - last_param_try > 5.0)
	{
		last_param_try = now;
		try_load_robot_params();
	}

	extra_layers_watchdog(now);

	/* app sumiu (Wi-Fi caiu, app travou/fechou): STOP, e repete a cada 2 s enquanto o navigator ainda
	   disser que esta' autonomo */
	if (app_watchdog > 0.0 && last_app_request > 0.0 && now - last_app_request > app_watchdog)
	{
		if (!watchdog_fired || (nav_autonomous && now - last_watchdog_stop > 2.0))
		{
			carmen_navigator_ackerman_stop();
			last_watchdog_stop = now;
			if (!watchdog_fired)
				fprintf(stderr, "carmen_server: WATCHDOG: %.1f s sem o app -> STOP no robo\n", now - last_app_request);
			watchdog_fired = true;
			publish("watchdog", "{\"state\":\"stopped\",\"silence\":" + num(now - last_app_request, 1) + "}");
		}
	}

	if (map_dirty && now - last_map_sent >= map_min_period)
	{
		publish("map", last_map_json, last_map_blob.data(), last_map_blob.size());
		last_map_sent = now;
		map_dirty = false;
	}

	if (now - last_output_flush >= 0.2 && !output_pending.empty())
	{
		for (std::map<std::string, std::string>::iterator it = output_pending.begin(); it != output_pending.end(); ++it)
			publish("out", "{\"module\":" + json_escape(it->first.c_str()) + ",\"text\":" + json_escape(it->second.c_str()) + "}");
		output_pending.clear();
		last_output_flush = now;
	}

	if (now - last_hello_sent >= 1.0)
	{
		publish("hello", "{\"t\":" + num(now, 3) + ",\"host\":" + json_escape(hostname) +
				",\"version\":" + std::to_string(CARMEN_SERVER_PROTOCOL_VERSION) + "}");
		last_hello_sent = now;
	}
}

/* ------------------------------------------------------------------------------------------ */

static void
shutdown_module(int sig)
{
	if (sig == SIGINT || sig == SIGTERM)
	{
		if (pub_socket) zmq_close(pub_socket);
		if (rep_socket) zmq_close(rep_socket);
		if (zmq_ctx) zmq_ctx_term(zmq_ctx);
		carmen_ipc_disconnect();
		fprintf(stderr, "carmen_server: desconectado.\n");
		exit(0);
	}
}

static void
read_args(int argc, char **argv)
{
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "-pub_port") && i + 1 < argc)          pub_port = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-cmd_port") && i + 1 < argc)     cmd_port = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-pose_hz") && i + 1 < argc)      pose_hz = atof(argv[++i]);
		else if (!strcmp(argv[i], "-path_hz") && i + 1 < argc)      path_hz = atof(argv[++i]);
		else if (!strcmp(argv[i], "-plan_hz") && i + 1 < argc)      plan_hz = atof(argv[++i]);
		else if (!strcmp(argv[i], "-map_period") && i + 1 < argc)   map_min_period = atof(argv[++i]);
		else if (!strcmp(argv[i], "-max_path_points") && i + 1 < argc) max_path_points = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-missions_folder") && i + 1 < argc) missions_folder = argv[++i];
		else if (!strcmp(argv[i], "-annotation_path") && i + 1 < argc) annotation_path = argv[++i];
		else if (!strcmp(argv[i], "-map_path") && i + 1 < argc)        map_path = argv[++i];
		else if (!strcmp(argv[i], "-particles") && i + 1 < argc)       particles_enabled = !strcmp(argv[++i], "on");
		else if (!strcmp(argv[i], "-app_watchdog") && i + 1 < argc)    app_watchdog = atof(argv[++i]);
		else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
		{
			printf("uso: %s [-pub_port 5560] [-cmd_port 5561] [-pose_hz 10] [-path_hz 2] [-plan_hz 5]\n"
				   "          [-map_period 2] [-max_path_points 400] [-missions_folder PASTA]\n"
				   "          [-annotation_path ARQ] [-map_path PASTA] [-particles on|off]\n"
				   "          [-app_watchdog 5 (s sem o app -> STOP; 0 desliga)]\n", argv[0]);
			exit(0);
		}
	}
}

static void *
make_socket(int type, int port)
{
	void *s = zmq_socket(zmq_ctx, type);
	int linger = 0;
	zmq_setsockopt(s, ZMQ_LINGER, &linger, sizeof(linger));
	if (type == ZMQ_PUB)
	{
		int hwm = 20;	// fila curta: o que nao couber e' descartado
		zmq_setsockopt(s, ZMQ_SNDHWM, &hwm, sizeof(hwm));
	}
	char endpoint[64];
	snprintf(endpoint, sizeof(endpoint), "tcp://*:%d", port);
	if (zmq_bind(s, endpoint) != 0)
	{
		fprintf(stderr, "carmen_server: nao consegui abrir %s: %s\n", endpoint, zmq_strerror(zmq_errno()));
		exit(1);
	}
	fprintf(stderr, "carmen_server: escutando em %s\n", endpoint);
	return s;
}

int
main(int argc, char **argv)
{
	read_args(argc, argv);

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	gethostname(hostname, sizeof(hostname) - 1);

	zmq_ctx = zmq_ctx_new();
	pub_socket = make_socket(ZMQ_PUB, pub_port);
	rep_socket = make_socket(ZMQ_REP, cmd_port);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(NULL, (carmen_handler_t) path_goals_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_navigator_ackerman_subscribe_plan_message(NULL, (carmen_handler_t) plan_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_route_planner_subscribe_road_network_message(NULL, (carmen_handler_t) road_network_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) end_point_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_state_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_proccontrol_subscribe_pidtable_message(NULL, (carmen_handler_t) pidtable_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_proccontrol_subscribe_output_message(NULL, (carmen_handler_t) output_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) odometry_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) motion_command_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ford_escape_subscribe_status_message(NULL, (carmen_handler_t) ford_escape_status_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_navigator_ackerman_subscribe_status_message(NULL, (carmen_handler_t) navigator_status_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_task_manager_subscribe_mission_state_message(NULL, (carmen_handler_t) mission_state_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_frenet_path_planner_subscribe_set_of_paths_message(NULL, (carmen_handler_t) frenet_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_subscribe_message((char *) CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME, (char *) CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_FMT,
			NULL, sizeof(carmen_obstacle_distance_mapper_compact_map_message), (carmen_handler_t) distance_map_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) annotation_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_offroad_planner_subscribe_plan_message(NULL, (carmen_handler_t) offroad_plan_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_moving_objects_point_clouds_subscribe_message(NULL, (carmen_handler_t) moving_objects_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) gps_xyz_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) xsens_quat_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_obstacle_avoider_subscribe_path_message(NULL, (carmen_handler_t) obstacle_avoider_path_handler, CARMEN_SUBSCRIBE_LATEST);
	if (particles_enabled)
		carmen_localize_ackerman_subscribe_particle_correction_message(NULL, (carmen_handler_t) particles_handler, CARMEN_SUBSCRIBE_LATEST);

	load_annotation_file();
	start_time = carmen_get_time();
	carmen_ipc_addPeriodicTimer(0.02, periodic_timer, NULL);

	carmen_ipc_dispatch();
	return 0;
}
