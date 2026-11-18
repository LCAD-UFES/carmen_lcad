/*
 * odometry_6d_go2 -- publica a odometria 6D (velocidades + aceleracao angular) do Unitree Go2.
 *
 * Fonte: o nav_msgs/Odometry_ que o proprio Go2 publica por DDS (padrao rt/utlidar/robot_odom).
 * Desta mensagem so' o twist (velocidade linear e angular no corpo) e' usado; a pose e' ignorada
 * de proposito (ver README.md). A aceleracao angular e' a derivada da velocidade angular, com um
 * passa-baixa (-alpha_filter).
 *
 * Opcional (-fused_odometry on): integra as velocidades e publica carmen_fused_odometry_message
 * ancorada em (-initial_x, -initial_y, -initial_theta). Com o localize_ackerman -mapping_mode on
 * isso vira o globalpos -- e' o mapeamento "so' com odometria". Um carmen_localize_ackerman_initialize
 * (Pose inicial no app/navigator) reancora a pose.
 *
 * O IPC e' publicado so' pela thread do carmen (timer): a callback do DDS apenas guarda a amostra.
 * carmen_get_host() e' resolvido no main(), antes de existir thread do DDS (nao e' thread-safe).
 *
 * Uso:
 *   ./odometry_6d_go2 -network_interface eth0
 *   ./odometry_6d_go2 -network_interface eth0 -print on
 *   ./odometry_6d_go2 -network_interface eth0 -fused_odometry on -initial_x 10000 -initial_y 10000
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <signal.h>

#if __has_include(<unitree/idl/ros2/Odometry_.hpp>)
#include <unitree/idl/ros2/Odometry_.hpp>		// ~/unitree_sdk2
#else
#include <unitree/ros2_idl/Odometry_.hpp>		// copia de /usr/local
#endif
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <carmen/carmen.h>
#include <carmen/odometry_6d_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>

using namespace unitree::robot;


///////////////////////////////////////////////////////////////////////////////////////////////
// Parametros

static char  *g_network_interface = NULL;
static int    g_domain_id = 0;
static char  *g_odom_topic = (char *) "rt/utlidar/robot_odom";
static double g_alpha_filter = 0.2;		// passa-baixa da aceleracao angular (0..1; 1 = sem filtro)
static int    g_print = 0;
static int    g_fused_odometry = 0;
static double g_initial_x = 0.0, g_initial_y = 0.0, g_initial_theta = 0.0;
static double g_axle = 0.35;			// robot_distance_between_front_and_rear_axles (phi equivalente)

static char *g_host = NULL;


///////////////////////////////////////////////////////////////////////////////////////////////
// Estado compartilhado (thread do DDS -> thread do IPC)

struct Sample
{
	double v[3], w[3];
	double timestamp;
};

static std::mutex g_mutex;
static Sample g_last;
static bool g_have_sample = false;


// Thread do DDS: so' guarda.
static void
odometry_callback(const void *dds_message)
{
	const nav_msgs::msg::dds_::Odometry_ *odom = (const nav_msgs::msg::dds_::Odometry_ *) dds_message;
	const auto &lin = odom->twist().twist().linear();
	const auto &ang = odom->twist().twist().angular();

	Sample s;
	s.v[0] = lin.x(); s.v[1] = lin.y(); s.v[2] = lin.z();
	s.w[0] = ang.x(); s.w[1] = ang.y(); s.w[2] = ang.z();
	s.timestamp = carmen_get_time();	// gettimeofday: thread-safe

	std::lock_guard<std::mutex> lock(g_mutex);
	g_last = s;
	g_have_sample = true;
}


///////////////////////////////////////////////////////////////////////////////////////////////
// fused_odometry (opcional): integra as velocidades a partir de uma ancora

static double g_x, g_y, g_theta;

static void
initialize_handler(carmen_localize_ackerman_initialize_message *msg)
{
	if (!msg->mean || msg->num_modes < 1)
		return;
	g_x = msg->mean[0].x;
	g_y = msg->mean[0].y;
	g_theta = msg->mean[0].theta;
	printf("odometry_6d_go2: fused_odometry reancorada em %.2f %.2f %.3f\n", g_x, g_y, g_theta);
}


static void
publish_fused_odometry(const carmen_odometry_6d_message &m, double dt)
{
	/* integra no referencial do mapa: deslocamento do corpo girado pela guinada atual */
	double c = cos(g_theta), s = sin(g_theta);
	g_x += (c * m.vx - s * m.vy) * dt;
	g_y += (s * m.vx + c * m.vy) * dt;
	g_theta = carmen_normalize_theta(g_theta + m.wz * dt + 0.5 * m.alpha_z * dt * dt);

	carmen_fused_odometry_message f;
	memset(&f, 0, sizeof(f));
	f.pose.position.x = g_x;
	f.pose.position.y = g_y;
	f.pose.orientation.yaw = g_theta;
	f.velocity.x = m.vx;
	f.velocity.y = m.vy;
	f.velocity.z = m.vz;
	f.angular_velocity.roll = m.wx;
	f.angular_velocity.pitch = m.wy;
	f.angular_velocity.yaw = m.wz;
	f.phi = (fabs(m.vx) > 0.05) ? carmen_clamp(-1.0, atan(m.wz * g_axle / m.vx), 1.0) : 0.0;
	f.num_trailers = 0;
	f.timestamp = m.timestamp;
	f.host = g_host;
	carmen_fused_odometry_publish_message(&f);
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Thread do IPC: publica cada amostra nova uma vez

static void
publish_odometry_6d(void *clientData __attribute__ ((unused)), unsigned long currentTime __attribute__ ((unused)),
		unsigned long scheduledTime __attribute__ ((unused)))
{
	static double last_timestamp = -1.0, last_warning = 0.0;
	static double last_w[3] = {0, 0, 0}, alpha[3] = {0, 0, 0};
	static bool announced = false;

	Sample s;
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		if (!g_have_sample)
		{
			if (carmen_get_time() - last_warning > 5.0)
			{
				printf("odometry_6d_go2: ainda sem odometria do robo em '%s' (interface '%s').\n", g_odom_topic, g_network_interface);
				last_warning = carmen_get_time();
			}
			return;
		}
		s = g_last;
	}
	if (s.timestamp == last_timestamp)
		return;

	double dt = (last_timestamp > 0.0) ? s.timestamp - last_timestamp : 0.0;
	if (dt > 1e-4 && dt < 0.5)
		for (int k = 0; k < 3; k++)
			alpha[k] = g_alpha_filter * (s.w[k] - last_w[k]) / dt + (1.0 - g_alpha_filter) * alpha[k];
	else
		for (int k = 0; k < 3; k++)
			alpha[k] = 0.0;	// primeira amostra ou buraco grande: sem derivada confiavel
	for (int k = 0; k < 3; k++)
		last_w[k] = s.w[k];
	last_timestamp = s.timestamp;

	carmen_odometry_6d_message m;
	m.vx = s.v[0]; m.vy = s.v[1]; m.vz = s.v[2];
	m.wx = s.w[0]; m.wy = s.w[1]; m.wz = s.w[2];
	m.alpha_x = alpha[0]; m.alpha_y = alpha[1]; m.alpha_z = alpha[2];
	m.timestamp = s.timestamp;
	m.host = g_host;
	carmen_odometry_6d_publish_message(&m);

	if (!announced)
	{
		printf("odometry_6d_go2: recebendo '%s', publicando %s.\n", g_odom_topic, CARMEN_ODOMETRY_6D_NAME);
		announced = true;
	}
	if (g_fused_odometry && dt > 0.0 && dt < 0.5)
		publish_fused_odometry(m, dt);
	if (g_print)
		printf("ODOM6D v %.3f %.3f %.3f  w %.3f %.3f %.3f  alpha %.3f %.3f %.3f\n",
				m.vx, m.vy, m.vz, m.wx, m.wy, m.wz, m.alpha_x, m.alpha_y, m.alpha_z);
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Inicializacao

static void
read_parameters(int argc, char **argv)
{
	carmen_param_allow_unfound_variables(1);
	carmen_param_t param_list[] =
	{
		{(char *) "commandline", (char *) "network_interface", CARMEN_PARAM_STRING, &g_network_interface, 0, NULL},
		{(char *) "commandline", (char *) "domain_id",         CARMEN_PARAM_INT,    &g_domain_id,         0, NULL},
		{(char *) "commandline", (char *) "odom_topic",        CARMEN_PARAM_STRING, &g_odom_topic,        0, NULL},
		{(char *) "commandline", (char *) "alpha_filter",      CARMEN_PARAM_DOUBLE, &g_alpha_filter,      0, NULL},
		{(char *) "commandline", (char *) "print",             CARMEN_PARAM_ONOFF,  &g_print,             0, NULL},
		{(char *) "commandline", (char *) "fused_odometry",    CARMEN_PARAM_ONOFF,  &g_fused_odometry,    0, NULL},
		{(char *) "commandline", (char *) "initial_x",         CARMEN_PARAM_DOUBLE, &g_initial_x,         0, NULL},
		{(char *) "commandline", (char *) "initial_y",         CARMEN_PARAM_DOUBLE, &g_initial_y,         0, NULL},
		{(char *) "commandline", (char *) "initial_theta",     CARMEN_PARAM_DOUBLE, &g_initial_theta,     0, NULL},
		{(char *) "robot",       (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &g_axle, 0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
	carmen_param_allow_unfound_variables(0);

	if (g_network_interface == NULL)
		g_network_interface = (char *) "";
	g_alpha_filter = carmen_clamp(0.01, g_alpha_filter, 1.0);
	g_x = g_initial_x;
	g_y = g_initial_y;
	g_theta = g_initial_theta;
}


static void
shutdown_module(int sig)
{
	if (sig == SIGINT || sig == SIGTERM)
	{
		carmen_ipc_disconnect();
		printf("\nodometry_6d_go2: desconectado do IPC. Sig: %d.\n", sig);
		exit(0);
	}
}


int
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	g_host = carmen_get_host();
	read_parameters(argc, argv);
	carmen_odometry_6d_define_message();

	printf("odometry_6d_go2: interface '%s', topico '%s'%s.\n", g_network_interface, g_odom_topic,
			g_fused_odometry ? ", publicando fused_odometry" : "");

	try
	{
		ChannelFactory::Instance()->Init(g_domain_id, g_network_interface);
	}
	catch (const std::exception &e)
	{
		fprintf(stderr, "odometry_6d_go2: NAO consegui subir o DDS na interface '%s' (robo desligado ou cabo fora?): %s\n",
				g_network_interface, e.what());
		exit(1);
	}

	ChannelSubscriberPtr<nav_msgs::msg::dds_::Odometry_> subscriber;
	subscriber.reset(new ChannelSubscriber<nav_msgs::msg::dds_::Odometry_>(g_odom_topic));
	subscriber->InitChannel(odometry_callback, 1);

	if (g_fused_odometry)
		carmen_localize_ackerman_subscribe_initialize_message(NULL, (carmen_handler_t) initialize_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_addPeriodicTimer(1.0 / 200.0, (TIMER_HANDLER_TYPE) publish_odometry_6d, NULL);
	carmen_ipc_dispatch();
	return 0;
}
