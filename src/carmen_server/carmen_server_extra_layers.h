/*
 * Camadas sob demanda do carmen_server: outros mapas (como o menu Maps do navigator_gui2 / viewer_3D)
 * e a nuvem de um LiDAR. Nada daqui e' assinado no IPC ate' o app pedir; desligar cancela a assinatura.
 *
 * Incluido no meio do carmen_server_main.cpp (usa publish(), num(), json_*). Os dois sao pesados no IPC
 * (o mapa do mapper do ARGOS tem 8,4 MB por mensagem), por isso:
 *  - mapas: reduzidos para no maximo max_side celulas de lado (max das celulas para nao sumir obstaculo),
 *    u8 + zlib, no maximo 1 por map_period segundos;
 *  - lidar: so' 1 a cada N disparos e 1 a cada M raios, ate' max_points pontos, no referencial do carro,
 *    int16 em cm + intensidade u8, zlib, no maximo lidar_hz por segundo.
 */
#ifndef CARMEN_SERVER_EXTRA_LAYERS_H
#define CARMEN_SERVER_EXTRA_LAYERS_H

#include <carmen/mapper_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/camera_drivers_interface.h>
#include <jpeglib.h>

/* ------------------------------------------------------------------------------------------ */
/* Mapas extras                                                                                */
/* ------------------------------------------------------------------------------------------ */

static std::string layer_source = "none";	// mapper, mapper_level1, cost, likelihood, global_likelihood, remission, moving_objects, road
static double layer_period = 1.0;
static int layer_max_side = 600;
static double layer_last_sent = 0.0;
static std::string layer_json = "null";
static std::vector<unsigned char> layer_blob;
static double layer_refreshed_at = 0.0;	// o app repete o pedido a cada ~10 s; sem isso desliga (app fechado)

/* Reduz e codifica um mapa de doubles. normalize: estica [min,max] dos valores validos para 0..254
   (likelihood e remissao nao sao probabilidades). unknown: valores < 0 (ou == -1 nos normalizados). */
static void
send_layer_map(const char *source, const double *data, const carmen_map_config_t &config, double timestamp, bool normalize, bool force = false)
{
	double now = carmen_get_time();
	if (!force && now - layer_last_sent < layer_period)
		return;
	int xs = config.x_size, ys = config.y_size;
	if (xs <= 0 || ys <= 0 || !data)
		return;
	int step = std::max(1, (int) ceil(std::max(xs, ys) / (double) layer_max_side));
	int ox = (xs + step - 1) / step, oy = (ys + step - 1) / step;

	double lo = 0.0, hi = 1.0;
	if (normalize)
	{
		lo = 1e300;
		hi = -1e300;
		for (int i = 0; i < xs * ys; i += 7)	// amostra: rapido e suficiente para a escala
		{
			double v = data[i];
			if (v == -1.0 || !std::isfinite(v))
				continue;
			lo = std::min(lo, v);
			hi = std::max(hi, v);
		}
		if (hi <= lo)
			hi = lo + 1.0;
	}

	/* o carmen guarda x maior (coluna) por fora: data[x * y_size + y] */
	std::vector<unsigned char> raw((size_t) ox * oy);
	for (int bx = 0; bx < ox; bx++)
		for (int by = 0; by < oy; by++)
		{
			double best = -2.0;
			bool any = false;
			for (int x = bx * step; x < std::min(xs, (bx + 1) * step); x++)
				for (int y = by * step; y < std::min(ys, (by + 1) * step); y++)
				{
					double v = data[(size_t) x * ys + y];
					bool unknown = normalize ? (v == -1.0 || !std::isfinite(v)) : (v < 0.0);
					if (unknown)
						continue;
					if (normalize)
						v = (v - lo) / (hi - lo);
					if (!any || v > best)
						best = v;
					any = true;
				}
			raw[(size_t) bx * oy + by] = any ? (unsigned char) (carmen_clamp(0.0, best, 1.0) * 254.0) : 255;
		}

	uLongf zsize = compressBound(raw.size());
	layer_blob.resize(zsize);
	if (compress2(layer_blob.data(), &zsize, raw.data(), raw.size(), 1) != Z_OK)
		return;
	layer_blob.resize(zsize);
	layer_json = "{\"t\":" + num(timestamp, 3) + ",\"source\":\"" + source + "\"" +
			",\"x_origin\":" + num(config.x_origin, 3) + ",\"y_origin\":" + num(config.y_origin, 3) +
			",\"resolution\":" + num(config.resolution * step, 4) +
			",\"x_size\":" + std::to_string(ox) + ",\"y_size\":" + std::to_string(oy) +
			",\"encoding\":\"zlib-u8\",\"unknown\":255}";
	publish("map2", layer_json, layer_blob.data(), layer_blob.size());
	layer_last_sent = now;
}

static void
layer_mapper_handler(carmen_mapper_map_message *msg)
{
	send_layer_map("mapper", msg->complete_map, msg->config, msg->timestamp, false);
}

static void
layer_mapper_level1_handler(carmen_mapper_map_message *msg)
{
	send_layer_map("mapper_level1", msg->complete_map, msg->config, msg->timestamp, false);
}

static void
layer_road_handler(carmen_map_server_road_map_message *msg)
{
	send_layer_map("road", msg->complete_map, msg->config, msg->timestamp, true);
}

static void
layer_moving_objects_handler(carmen_moving_objects_map_message *msg)
{
	send_layer_map("moving_objects", msg->complete_map, msg->config, msg->timestamp, false);
}

static void
layer_localize_map_handler(carmen_map_server_localize_map_message *msg)
{
	if (layer_source == "likelihood")
		send_layer_map("likelihood", msg->complete_prob, msg->config, msg->timestamp, true);
	else if (layer_source == "global_likelihood")
		send_layer_map("global_likelihood", msg->complete_gprob, msg->config, msg->timestamp, true);
	else if (layer_source == "remission")
		send_layer_map("remission", msg->complete_mean_remission_map, msg->config, msg->timestamp, true);
}

static void
layer_cost_handler(carmen_map_server_compact_cost_map_message *msg)
{
	/* mapa compacto: so' as celulas com custo; o resto e' livre (0) */
	static std::vector<double> full;
	int xs = msg->config.x_size, ys = msg->config.y_size;
	if (xs <= 0 || ys <= 0)
		return;
	if (carmen_get_time() - layer_last_sent < layer_period)
		return;
	full.assign((size_t) xs * ys, 0.0);
	for (int i = 0; i < msg->size; i++)
	{
		int x = msg->coord_x[i], y = msg->coord_y[i];
		if (x >= 0 && x < xs && y >= 0 && y < ys)
			full[(size_t) x * ys + y] = msg->value[i];
	}
	send_layer_map("cost", full.data(), msg->config, msg->timestamp, false);
}

static void
unsubscribe_layer(const std::string &source)
{
	if (source == "mapper")
		carmen_unsubscribe_message((char *) CARMEN_MAPPER_MAP_MESSAGE_NAME, (carmen_handler_t) layer_mapper_handler);
	else if (source == "mapper_level1")
		carmen_unsubscribe_message((char *) CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME, (carmen_handler_t) layer_mapper_level1_handler);
	else if (source == "road")
		carmen_unsubscribe_message((char *) CARMEN_MAP_SERVER_ROAD_MAP_NAME, (carmen_handler_t) layer_road_handler);
	else if (source == "moving_objects")
		carmen_unsubscribe_message((char *) CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME, (carmen_handler_t) layer_moving_objects_handler);
	else if (source == "likelihood" || source == "global_likelihood" || source == "remission")
		carmen_unsubscribe_message((char *) CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME, (carmen_handler_t) layer_localize_map_handler);
	else if (source == "cost")
		carmen_unsubscribe_message((char *) CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME, (carmen_handler_t) layer_cost_handler);
}

static bool
subscribe_layer(const std::string &source)
{
	if (source == "mapper")
		carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) layer_mapper_handler, CARMEN_SUBSCRIBE_LATEST);
	else if (source == "mapper_level1")
		carmen_mapper_subscribe_map_level1_message(NULL, (carmen_handler_t) layer_mapper_level1_handler, CARMEN_SUBSCRIBE_LATEST);
	else if (source == "road")
		carmen_map_server_subscribe_road_map(NULL, (carmen_handler_t) layer_road_handler, CARMEN_SUBSCRIBE_LATEST);
	else if (source == "moving_objects")
		carmen_moving_objects_map_subscribe_message(NULL, (carmen_handler_t) layer_moving_objects_handler, CARMEN_SUBSCRIBE_LATEST);
	else if (source == "likelihood" || source == "global_likelihood" || source == "remission")
		carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) layer_localize_map_handler, CARMEN_SUBSCRIBE_LATEST);
	else if (source == "cost")
		carmen_map_server_subscribe_compact_cost_map(NULL, (carmen_handler_t) layer_cost_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		return source == "none";
	return true;
}

/* comando map_layer {source, period, max_side} */
static std::string
command_map_layer(const std::string &req)
{
	std::string source = "none";
	json_get_string(req, "source", &source);
	double period = layer_period, max_side = layer_max_side;
	json_get_number(req, "period", &period);
	json_get_number(req, "max_side", &max_side);
	layer_period = carmen_clamp(0.2, period, 30.0);
	layer_max_side = (int) carmen_clamp(100, max_side, 2000);
	layer_refreshed_at = carmen_get_time();

	if (source != layer_source)
	{
		bool same_localize = (source == "likelihood" || source == "global_likelihood" || source == "remission") &&
				(layer_source == "likelihood" || layer_source == "global_likelihood" || layer_source == "remission");
		if (!same_localize)
			unsubscribe_layer(layer_source);
		std::string previous = layer_source;
		layer_source = source;
		if (!same_localize && !subscribe_layer(source))
		{
			layer_source = "none";
			return "{\"ok\":false,\"error\":\"mapa desconhecido: " + source + "\"}";
		}
		layer_json = "null";
		layer_blob.clear();
		layer_last_sent = 0.0;	// o proximo chega na hora
	}
	return "{\"ok\":true,\"source\":\"" + layer_source + "\"}";
}

/* ------------------------------------------------------------------------------------------ */
/* Nuvem do LiDAR                                                                              */
/* ------------------------------------------------------------------------------------------ */

static int lidar_id = -1;			// -1: desligado
static carmen_lidar_config *lidar_config = NULL;
static double lidar_hz = 2.0;
static int lidar_max_points = 15000;
static double lidar_last_sent = 0.0;
static std::vector<unsigned char> lidar_blob;
static double lidar_refreshed_at = 0.0;
static double board_x = 0, board_y = 0, board_z = 0, board_roll = 0, board_pitch = 0, board_yaw = 0;

/* matriz de rotacao roll-pitch-yaw (como o carmen_rotation_matrix) */
static void
rpy_matrix(double roll, double pitch, double yaw, double m[3][3])
{
	double cr = cos(roll), sr = sin(roll), cp = cos(pitch), sp = sin(pitch), cy = cos(yaw), sy = sin(yaw);
	m[0][0] = cy * cp; m[0][1] = cy * sp * sr - sy * cr; m[0][2] = cy * sp * cr + sy * sr;
	m[1][0] = sy * cp; m[1][1] = sy * sp * sr + cy * cr; m[1][2] = sy * sp * cr - cy * sr;
	m[2][0] = -sp;     m[2][1] = cp * sr;                m[2][2] = cp * cr;
}

static void
lidar_handler(carmen_velodyne_variable_scan_message *msg)
{
	double now = carmen_get_time();
	if (!lidar_config || now - lidar_last_sent < 1.0 / lidar_hz || msg->number_of_shots <= 0)
		return;
	lidar_last_sent = now;

	int shot_size = msg->partial_scan[0].shot_size;
	long total = (long) msg->number_of_shots * shot_size;
	/* decimacao: metade nos disparos, o resto nos raios (mantem as linhas do lidar) */
	int ray_step = 1, shot_step = 1;
	while (total / ((long) ray_step * shot_step) > lidar_max_points)
	{
		if (shot_step <= 2 * ray_step || ray_step * 2 > shot_size / 4)
			shot_step++;
		else
			ray_step *= 2;
	}

	double ml[3][3], mb[3][3];
	rpy_matrix(lidar_config->pose.orientation.roll, lidar_config->pose.orientation.pitch, lidar_config->pose.orientation.yaw, ml);
	rpy_matrix(board_roll, board_pitch, board_yaw, mb);
	double max_range = lidar_config->max_range > 0 ? lidar_config->max_range : 70.0;
	double div = lidar_config->range_division_factor > 0 ? lidar_config->range_division_factor : 500.0;

	std::vector<unsigned char> raw;
	raw.reserve((size_t) lidar_max_points * 7 + 64);
	int n = 0;
	for (int s = 0; s < msg->number_of_shots; s += shot_step)
	{
		const carmen_velodyne_shot &shot = msg->partial_scan[s];
		double h = carmen_degrees_to_radians(-shot.angle);
		double ch = cos(h), sh = sin(h);
		for (int r = 0; r < shot.shot_size && r < lidar_config->shot_size; r += ray_step)
		{
			double range = shot.distance[r] / div;
			if (range < 0.5 || range > max_range)
				continue;
			int ray = lidar_config->ray_order ? lidar_config->ray_order[r] : r;
			double v = carmen_degrees_to_radians(lidar_config->vertical_angles[ray]);
			double p[3] = {range * cos(v) * ch, range * cos(v) * sh, range * sin(v)};
			double q[3], c[3];
			for (int k = 0; k < 3; k++)
				q[k] = ml[k][0] * p[0] + ml[k][1] * p[1] + ml[k][2] * p[2];
			q[0] += lidar_config->pose.position.x;
			q[1] += lidar_config->pose.position.y;
			q[2] += lidar_config->pose.position.z;
			for (int k = 0; k < 3; k++)
				c[k] = mb[k][0] * q[0] + mb[k][1] * q[1] + mb[k][2] * q[2];
			c[0] += board_x;
			c[1] += board_y;
			c[2] += board_z;
			short xyz[3] = {(short) carmen_clamp(-32000.0, c[0] * 100.0, 32000.0), (short) carmen_clamp(-32000.0, c[1] * 100.0, 32000.0),
					(short) carmen_clamp(-32000.0, c[2] * 100.0, 32000.0)};
			raw.insert(raw.end(), (unsigned char *) xyz, (unsigned char *) xyz + 6);
			raw.push_back(shot.intensity ? (unsigned char) std::min(255, (int) shot.intensity[r]) : 0);
			n++;
		}
	}
	uLongf zsize = compressBound(raw.size());
	lidar_blob.resize(zsize);
	if (compress2(lidar_blob.data(), &zsize, raw.data(), raw.size(), 1) != Z_OK)
		return;
	lidar_blob.resize(zsize);
	publish("lidar", "{\"t\":" + num(msg->timestamp, 3) + ",\"id\":" + std::to_string(lidar_id) +
			",\"n\":" + std::to_string(n) + ",\"x\":" + num(last_globalpos_x, 3) + ",\"y\":" + num(last_globalpos_y, 3) +
			",\"theta\":" + num(last_globalpos_theta, 4) + ",\"encoding\":\"zlib-i16cm-u8\"}", lidar_blob.data(), lidar_blob.size());
}

/* comando lidar {id (-1 desliga), hz, max_points} */
static std::string
command_lidar(const std::string &req)
{
	double id = -1, hz = lidar_hz, max_points = lidar_max_points;
	json_get_number(req, "id", &id);
	json_get_number(req, "hz", &hz);
	json_get_number(req, "max_points", &max_points);
	lidar_hz = carmen_clamp(0.2, hz, 10.0);
	lidar_max_points = (int) carmen_clamp(1000, max_points, 100000);
	lidar_refreshed_at = carmen_get_time();

	int new_id = (int) id;
	if (new_id == lidar_id)
		return "{\"ok\":true,\"id\":" + std::to_string(lidar_id) + "}";
	if (lidar_id >= 0)
		carmen_velodyne_unsubscribe_variable_scan_message((carmen_handler_t) lidar_handler, lidar_id);
	lidar_id = -1;
	if (new_id < 0)
		return "{\"ok\":true,\"id\":-1}";

	/* load_lidar_config sai do programa se faltar parametro: confere antes */
	char module[32];
	int shot_size = 0;
	snprintf(module, sizeof(module), "lidar%d", new_id);
	carmen_param_allow_unfound_variables(1);
	carmen_param_set_module(module);
	int status = carmen_param_get_int("shot_size", &shot_size, NULL);
	carmen_param_set_module(NULL);
	if (status < 0 || shot_size <= 0)
		return "{\"ok\":false,\"error\":\"lidar" + std::to_string(new_id) + " nao esta' no carmen.ini\"}";
	char *argv_fake[] = {(char *) "carmen_server", NULL};
	carmen_lidar_config *cfg = NULL;
	load_lidar_config(1, argv_fake, new_id, &cfg);
	if (!cfg || !cfg->vertical_angles)
		return "{\"ok\":false,\"error\":\"nao consegui ler a configuracao do lidar\"}";
	lidar_config = cfg;
	carmen_param_t board[] = {
		{(char *) "sensor_board_1", (char *) "x", CARMEN_PARAM_DOUBLE, &board_x, 0, NULL},
		{(char *) "sensor_board_1", (char *) "y", CARMEN_PARAM_DOUBLE, &board_y, 0, NULL},
		{(char *) "sensor_board_1", (char *) "z", CARMEN_PARAM_DOUBLE, &board_z, 0, NULL},
		{(char *) "sensor_board_1", (char *) "roll", CARMEN_PARAM_DOUBLE, &board_roll, 0, NULL},
		{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &board_pitch, 0, NULL},
		{(char *) "sensor_board_1", (char *) "yaw", CARMEN_PARAM_DOUBLE, &board_yaw, 0, NULL},
	};
	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(1, argv_fake, board, sizeof(board) / sizeof(board[0]));

	lidar_id = new_id;
	lidar_last_sent = 0.0;
	carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) lidar_handler, CARMEN_SUBSCRIBE_LATEST, lidar_id);
	return "{\"ok\":true,\"id\":" + std::to_string(lidar_id) + "}";
}

/* ------------------------------------------------------------------------------------------ */
/* Cameras (camera_drivers: camera<N>, BGR)                                                     */
/* ------------------------------------------------------------------------------------------ */

#define SERVER_MAX_CAMERA_ID 9

struct CameraSub
{
	bool on = false;
	double hz = 5.0, last_sent = 0.0, refreshed_at = 0.0;
	int width = 480, quality = 60;
};
static CameraSub cameras[SERVER_MAX_CAMERA_ID + 1];

/* reduz por um fator inteiro (media do bloco), troca BGR -> RGB e comprime em JPEG */
static bool
encode_camera_jpeg(const camera_image *img, int target_width, int quality, std::vector<unsigned char> *out, int *ow, int *oh)
{
	int ch = img->number_of_channels;
	if (!img->raw_data || img->size_in_bytes_of_each_element != 1 || (ch != 1 && ch != 3) || img->width <= 0 || img->height <= 0)
		return false;
	int f = std::max(1, img->width / std::max(64, target_width));
	int w = img->width / f, hgt = img->height / f;
	std::vector<unsigned char> rgb((size_t) w * hgt * 3);
	const unsigned char *src = (const unsigned char *) img->raw_data;
	for (int y = 0; y < hgt; y++)
		for (int x = 0; x < w; x++)
		{
			int acc[3] = {0, 0, 0};
			for (int dy = 0; dy < f; dy++)
			{
				const unsigned char *p = src + ((size_t) (y * f + dy) * img->width + (size_t) x * f) * ch;
				for (int dx = 0; dx < f; dx++, p += ch)
				{
					if (ch == 3) { acc[0] += p[2]; acc[1] += p[1]; acc[2] += p[0]; }
					else { acc[0] += p[0]; acc[1] += p[0]; acc[2] += p[0]; }
				}
			}
			unsigned char *d = &rgb[((size_t) y * w + x) * 3];
			for (int k = 0; k < 3; k++)
				d[k] = (unsigned char) (acc[k] / (f * f));
		}

	jpeg_compress_struct cinfo;
	jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	unsigned char *mem = NULL;
	unsigned long mem_size = 0;
	jpeg_mem_dest(&cinfo, &mem, &mem_size);
	cinfo.image_width = w;
	cinfo.image_height = hgt;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, quality, TRUE);
	jpeg_start_compress(&cinfo, TRUE);
	while (cinfo.next_scanline < cinfo.image_height)
	{
		JSAMPROW row = &rgb[(size_t) cinfo.next_scanline * w * 3];
		jpeg_write_scanlines(&cinfo, &row, 1);
	}
	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	out->assign(mem, mem + mem_size);
	free(mem);
	*ow = w;
	*oh = hgt;
	return true;
}

static void
camera_frame(int id, camera_message *msg)
{
	CameraSub &c = cameras[id];
	double now = carmen_get_time();
	if (!c.on || now - c.last_sent < 1.0 / c.hz || msg->number_of_images < 1)
		return;
	c.last_sent = now;
	std::vector<unsigned char> jpeg;
	int w = 0, h = 0;
	if (!encode_camera_jpeg(&msg->images[0], c.width, c.quality, &jpeg, &w, &h))
		return;
	publish("camera", "{\"t\":" + num(msg->timestamp, 3) + ",\"id\":" + std::to_string(id) + ",\"w\":" + std::to_string(w) +
			",\"h\":" + std::to_string(h) + ",\"encoding\":\"jpeg\"}", jpeg.data(), jpeg.size());
}

template <int ID> static void camera_handler(camera_message *msg) { camera_frame(ID, msg); }
static carmen_handler_t camera_handlers[SERVER_MAX_CAMERA_ID + 1] = {
	(carmen_handler_t) camera_handler<0>, (carmen_handler_t) camera_handler<1>, (carmen_handler_t) camera_handler<2>,
	(carmen_handler_t) camera_handler<3>, (carmen_handler_t) camera_handler<4>, (carmen_handler_t) camera_handler<5>,
	(carmen_handler_t) camera_handler<6>, (carmen_handler_t) camera_handler<7>, (carmen_handler_t) camera_handler<8>,
	(carmen_handler_t) camera_handler<9>};

/* comando camera {id, on (0/1), hz, width, quality} */
static std::string
command_camera(const std::string &req)
{
	double id = -1, on = 1, hz, width, quality;
	json_get_number(req, "id", &id);
	json_get_number(req, "on", &on);
	if (id < 0 || id > SERVER_MAX_CAMERA_ID)
		return "{\"ok\":false,\"error\":\"camera id fora de 0..9\"}";
	CameraSub &c = cameras[(int) id];
	hz = c.hz; width = c.width; quality = c.quality;
	json_get_number(req, "hz", &hz);
	json_get_number(req, "width", &width);
	json_get_number(req, "quality", &quality);
	c.hz = carmen_clamp(0.2, hz, 15.0);
	c.width = (int) carmen_clamp(64, width, 1280);
	c.quality = (int) carmen_clamp(20, quality, 95);
	c.refreshed_at = carmen_get_time();
	if ((on != 0) != c.on)
	{
		if (on != 0)
			camera_drivers_subscribe_message((int) id, NULL, camera_handlers[(int) id], CARMEN_SUBSCRIBE_LATEST);
		else
			camera_drivers_unsubscribe_message((int) id, camera_handlers[(int) id]);
		c.on = (on != 0);
		c.last_sent = 0.0;
		fprintf(stderr, "carmen_server: camera%d %s\n", (int) id, c.on ? "ligada" : "desligada");
	}
	return "{\"ok\":true,\"id\":" + std::to_string((int) id) + ",\"on\":" + (c.on ? "true" : "false") + "}";
}

/* Chamado pelo timer: se o app sumiu (nao repetiu o pedido em 30 s), para de assinar o que e' pesado. */
static void
extra_layers_watchdog(double now)
{
	if (layer_source != "none" && now - layer_refreshed_at > 30.0)
	{
		fprintf(stderr, "carmen_server: app sem renovar o mapa %s ha' 30 s, desligando\n", layer_source.c_str());
		command_map_layer("{\"source\":\"none\"}");
	}
	if (lidar_id >= 0 && now - lidar_refreshed_at > 30.0)
	{
		fprintf(stderr, "carmen_server: app sem renovar o lidar ha' 30 s, desligando\n");
		command_lidar("{\"id\":-1}");
	}
	for (int i = 0; i <= SERVER_MAX_CAMERA_ID; i++)
		if (cameras[i].on && now - cameras[i].refreshed_at > 30.0)
		{
			fprintf(stderr, "carmen_server: app sem renovar a camera%d ha' 30 s, desligando\n", i);
			command_camera("{\"id\":" + std::to_string(i) + ",\"on\":0}");
		}
}

#endif
