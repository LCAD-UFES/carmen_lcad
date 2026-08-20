/*
 * graphslam_publish_globalpos
 *
 * Publica carmen_localize_ackerman_globalpos_message DIRETAMENTE a partir de um
 * arquivo de poses otimizadas (poses_opt.dat), dispensando o localize_ackerman.
 *
 * Por que existe (o graphslam_publish antigo continua intacto, ver
 * graphslam_publish_main.cpp):
 *
 *   O graphslam_publish original publica apenas carmen_fused_odometry_message --
 *   a funcao assembly_and_publish_globalpos_message que ele carrega e' codigo
 *   morto, nunca chamada. Como o mapper so' assina globalpos, era obrigatorio
 *   por um "./localize_ackerman -mapping_mode on" no meio do caminho so' para
 *   converter fused_odometry -> globalpos. Isso cria tres casamentos de
 *   timestamp encadeados, nenhum deles com tolerancia:
 *
 *     1. graphslam_publish: pega a pose anterior mais proxima, de QUALQUER idade,
 *        e so' publica quando a odometria acusa v > 0.1 (com o carro parado ele
 *        fica mudo);
 *     2. localize_ackerman: pega a fused_odometry mais proxima, de QUALQUER idade,
 *        e extrapola com ds = v*dt sem teto no dt (ha' um TODO no proprio fonte
 *        perguntando se nao deveria deixar de publicar quando dt e' grande --
 *        localize_ackerman_main.cpp);
 *     3. mapper: usa sempre a globalpos MAIS RECENTE, sem comparar timestamp com
 *        o scan.
 *
 *   O resultado e' pose velha aplicada a scan novo. No mapper isso nao apenas
 *   pinta no lugar errado: o atraso entra como offset de deskew
 *   (dt1 = points_timestamp - robot_timestamp - N*dt em mapper.cpp), ou seja, a
 *   nuvem e' ESTICADA proporcionalmente ao atraso da pose. E' a assinatura de
 *   borrao ("mancha") no mapa.
 *
 *   Este modulo faz o que um fork deste codigo ja' fazia: casa a pose com o
 *   timestamp da PROPRIA mensagem de lidar, exigindo casamento exato (1 us por
 *   padrao), e publica globalpos direto. Sem casamento exato ele simplesmente
 *   NAO publica -- e' melhor o mapper ficar sem pose nova do que pintar com uma
 *   pose errada. Nao ha' extrapolacao de especie alguma e nao ha' gate de
 *   velocidade: com o carro parado ele continua publicando, desde que exista
 *   pose para aquele scan.
 *
 * Formatos de arquivo aceitos (colunas separadas por espaco):
 *    4:  x y theta t_vertice
 *    6:  x y theta t_lidar t_odom t_vertice          (todos os t viram t_vertice)
 *    7:  x y theta t_vertice t_lidar t_odom t_gps
 *   10:  x y z roll pitch yaw t_vertice t_lidar t_odom t_gps   (pose 6D do SC-LIO-SAM)
 *
 * O SC-LIO-SAM grava o formato de 10 colunas. Colunas de timestamp valem -1
 * quando nao existem (o ARGOS nao tem odometria Ackermann, entao t_odom = -1 --
 * use -poses_from lidar, que e' o padrao).
 *
 * Uso:
 *   ./graphslam_publish_globalpos <poses_opt.dat> [opcoes]
 *
 *   -poses_from     lidar | velodyne | odometry   (padrao: lidar)
 *   -lidar_id       N        id do variable scan a assinar (padrao: 5)
 *   -tolerance      S        janela de casamento exato, em segundos (padrao: 1e-6)
 *   -fake_timestamp on|off   off (padrao) = sem casamento exato, nao publica.
 *                            on = aceita a pose mais proxima seja qual for a
 *                            distancia. So' use para depurar; e' exatamente o
 *                            comportamento frouxo que borra o mapa.
 *   -verbose        on|off   imprime estatisticas a cada 5 s (padrao: on)
 *   -save_globalpos_file <arquivo>
 *                            grava cada globalpos publicada, no MESMO formato do
 *                            localize_ackerman -save_globalpos_file:
 *                            "x  y  theta  v  phi  timestamp" separados por TAB.
 */

#include <vector>
#include <string>
#include <algorithm>
#include <carmen/carmen.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/velodyne_interface.h>

using namespace std;


typedef struct
{
	double x, y, theta;
	double vertex_timestamp;
	double lidar_timestamp;
	double odometry_timestamp;
	double gps_timestamp;
} pose_entry_t;


#define ODOMETRY_QUEUE_SIZE 200

static vector<pose_entry_t> poses_array;
static carmen_base_ackerman_odometry_message odometry_queue[ODOMETRY_QUEUE_SIZE];
static int odometry_queue_index = 0;

static char *poses_from = NULL;
static int lidar_id = 5;
static double tolerance = 1e-6;
static int fake_timestamp = 0;
static int verbose = 1;
static char *save_globalpos_file = NULL;
static FILE *globalpos_file = NULL;

static carmen_localize_ackerman_globalpos_message globalpos;

static long int stat_published = 0;
static long int stat_skipped_no_pose = 0;
static long int stat_skipped_repeated = 0;
static double stat_worst_match = 0.0;
static double stat_last_report = 0.0;


static void
shutdown_module(int signo)
{
	if ((signo == SIGINT) || (signo == SIGTERM))
	{
		if (globalpos_file)
		{
			fclose(globalpos_file);
			globalpos_file = NULL;
			printf("\ngraphslam_publish_globalpos: globalpos gravada em '%s'.\n", save_globalpos_file);
		}

		carmen_ipc_disconnect();
		printf("\ngraphslam_publish_globalpos: publicadas %ld, sem pose exata %ld, repetidas %ld.\n",
				stat_published, stat_skipped_no_pose, stat_skipped_repeated);
		printf("graphslam_publish_globalpos: desconectado.\n");
		exit(0);
	}
}


static void
report_stats(double timestamp)
{
	if (!verbose)
		return;

	if (stat_last_report == 0.0)
		stat_last_report = timestamp;

	if ((timestamp - stat_last_report) < 5.0)
		return;

	long int total = stat_published + stat_skipped_no_pose;
	printf("graphslam_publish_globalpos: %ld publicadas, %ld sem pose exata (%.1f%%), pior casamento %.9f s\n",
			stat_published, stat_skipped_no_pose,
			(total > 0) ? (100.0 * (double) stat_skipped_no_pose / (double) total) : 0.0,
			stat_worst_match);
	stat_last_report = timestamp;
}


/*
 * Le o arquivo de poses. Reconhece 4, 6, 7 ou 10 colunas por linha; linhas em
 * branco, comentarios (#) e linhas com outro numero de colunas sao ignoradas.
 */
static void
load_corrected_poses(const char *filename)
{
	FILE *f = fopen(filename, "r");
	if (f == NULL)
		exit(printf("graphslam_publish_globalpos: nao consegui abrir '%s'\n", filename));

	char line[4096];
	long int ignored = 0;

	while (fgets(line, sizeof(line), f) != NULL)
	{
		char *p = line;
		while (*p == ' ' || *p == '\t')
			p++;
		if (*p == '#' || *p == '\n' || *p == '\0')
			continue;

		double c[10];
		int n = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9]);

		pose_entry_t pose;
		memset(&pose, 0, sizeof(pose));

		if (n == 4)
		{
			pose.x = c[0]; pose.y = c[1]; pose.theta = c[2];
			pose.vertex_timestamp = pose.lidar_timestamp = pose.odometry_timestamp = pose.gps_timestamp = c[3];
		}
		else if (n == 6)
		{
			// Aqui o timestamp que vale e' o do vertice; os outros dois sao
			// redundantes nesse formato.
			pose.x = c[0]; pose.y = c[1]; pose.theta = c[2];
			pose.vertex_timestamp = pose.lidar_timestamp = pose.odometry_timestamp = pose.gps_timestamp = c[5];
		}
		else if (n == 7)
		{
			pose.x = c[0]; pose.y = c[1]; pose.theta = c[2];
			pose.vertex_timestamp   = c[3];
			pose.lidar_timestamp    = c[4];
			pose.odometry_timestamp = c[5];
			pose.gps_timestamp      = c[6];
		}
		else if (n == 10)
		{
			// Pose 6D: x y z roll pitch yaw ...  -- theta e' o yaw (coluna 6).
			pose.x = c[0]; pose.y = c[1]; pose.theta = c[5];
			pose.vertex_timestamp   = c[6];
			pose.lidar_timestamp    = c[7];
			pose.odometry_timestamp = c[8];
			pose.gps_timestamp      = c[9];
		}
		else
		{
			ignored++;
			continue;
		}

		poses_array.push_back(pose);
	}

	fclose(f);

	if (poses_array.empty())
		exit(printf("graphslam_publish_globalpos: '%s' nao tem nenhuma linha com 4, 6, 7 ou 10 colunas.\n", filename));

	if (ignored > 0)
		printf("graphslam_publish_globalpos: %ld linha(s) ignorada(s) por numero de colunas inesperado.\n", ignored);

	printf("graphslam_publish_globalpos: %zu poses carregadas de '%s'.\n", poses_array.size(), filename);
	printf("graphslam_publish_globalpos: janela de tempo %.6f .. %.6f (%.1f s).\n",
			poses_array.front().vertex_timestamp, poses_array.back().vertex_timestamp,
			poses_array.back().vertex_timestamp - poses_array.front().vertex_timestamp);
}


/*
 * Devolve o indice da pose cujo timestamp (o da fonte escolhida em -poses_from)
 * casa com o da mensagem dentro de 'tolerance'. Sem casamento devolve -1, e o
 * chamador nao publica nada -- essa e' a diferenca central em relacao ao
 * graphslam_publish antigo, que aceitava a pose anterior mais proxima de
 * qualquer idade.
 */
static int
find_synchronized_pose(double timestamp)
{
	double min_diff = DBL_MAX;
	int min_diff_index = -1;

	for (size_t i = 0; i < poses_array.size(); i++)
	{
		double key;

		if (strcmp(poses_from, "odometry") == 0)
			key = poses_array[i].odometry_timestamp;
		else
			key = poses_array[i].lidar_timestamp;

		if (key <= 0.0)		// coluna ausente no arquivo (gravada como -1)
			continue;

		double diff = fabs(timestamp - key);

		if (diff < min_diff)
		{
			min_diff = diff;
			min_diff_index = i;

			if (diff <= tolerance)
			{
				if (diff > stat_worst_match)
					stat_worst_match = diff;
				return (i);
			}
		}
	}

	if (fake_timestamp)
	{
		if (min_diff > stat_worst_match)
			stat_worst_match = min_diff;
		return (min_diff_index);
	}

	return (-1);
}


/*
 * v e phi saem da odometria casada EXATAMENTE com o instante gravado na coluna
 * odometry_timestamp da pose. Sem casamento exato ficam zerados de proposito:
 * v e phi so' alimentam o deskew do mapper, e um valor chutado ali estica a
 * nuvem. Zero significa "nao compense movimento", que e' o comportamento seguro.
 * O ARGOS nao tem odometria Ackermann (odometry_timestamp = -1), entao esse e'
 * o caminho normal la'.
 */
static void
get_odometry_at(double odometry_timestamp, double *v, double *phi)
{
	*v = 0.0;
	*phi = 0.0;

	if (odometry_timestamp <= 0.0)
		return;

	for (int i = 0; i < ODOMETRY_QUEUE_SIZE; i++)
	{
		if ((odometry_queue[i].timestamp > 0.0) &&
			(fabs(odometry_timestamp - odometry_queue[i].timestamp) <= tolerance))
		{
			*v = odometry_queue[i].v;
			*phi = odometry_queue[i].phi;
			return;
		}
	}
}


static void
publish_globalpos(const pose_entry_t &pose, double timestamp)
{
	static double last_published_timestamp = -1.0;

	if ((last_published_timestamp > 0.0) && (fabs(last_published_timestamp - timestamp) <= tolerance))
	{
		stat_skipped_repeated++;
		return;
	}

	double v, phi;
	get_odometry_at(pose.odometry_timestamp, &v, &phi);

	memset(&globalpos, 0, sizeof(globalpos));

	globalpos.globalpos.x = pose.x;
	globalpos.globalpos.y = pose.y;
	globalpos.globalpos.theta = pose.theta;

	globalpos.pose.position.x = pose.x;
	globalpos.pose.position.y = pose.y;
	globalpos.pose.position.z = 0.0;
	globalpos.pose.orientation.yaw = pose.theta;

	globalpos.velocity.x = v;
	globalpos.v = v;
	globalpos.phi = phi;

	globalpos.converged = 1;
	globalpos.num_trailers = 0;
	globalpos.semi_trailer_engaged = 0;
	globalpos.semi_trailer_type = 0;

	globalpos.timestamp = timestamp;
	globalpos.host = carmen_get_host();

	carmen_localize_ackerman_publish_globalpos_message(&globalpos);

	if (save_globalpos_file)
	{
		if (globalpos_file == NULL)
		{
			globalpos_file = fopen(save_globalpos_file, "w");
			if (globalpos_file == NULL)
				fprintf(stderr, "graphslam_publish_globalpos: nao consegui abrir '%s' para gravar.\n", save_globalpos_file);
		}
		// Mesmo formato do localize_ackerman -save_globalpos_file.
		if (globalpos_file)
			fprintf(globalpos_file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					globalpos.pose.position.x, globalpos.pose.position.y,
					globalpos.pose.orientation.yaw, v, phi, timestamp);
	}

	last_published_timestamp = timestamp;
	stat_published++;
}


static void
handle_message_timestamp(double timestamp)
{
	int index = find_synchronized_pose(timestamp);

	if (index < 0)
	{
		stat_skipped_no_pose++;
		report_stats(timestamp);
		return;
	}

	publish_globalpos(poses_array[index], timestamp);
	report_stats(timestamp);
}


static void
variable_scan_message_handler(carmen_velodyne_variable_scan_message *msg)
{
	handle_message_timestamp(msg->timestamp);
}


static void
partial_scan_message_handler(carmen_velodyne_partial_scan_message *msg)
{
	handle_message_timestamp(msg->timestamp);
}


static void
base_ackerman_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	odometry_queue[odometry_queue_index] = *msg;
	odometry_queue_index = (odometry_queue_index + 1) % ODOMETRY_QUEUE_SIZE;

	if (strcmp(poses_from, "odometry") == 0)
		handle_message_timestamp(msg->timestamp);
}


/*
 * carmen_param_check_unhandled_commandline_args() nao serve aqui porque o
 * arquivo de poses e' um argumento posicional. Esta checagem cobre o caso que
 * importa: uma flag escrita errado sendo ignorada em silencio.
 */
static void
check_unknown_flags(int argc, char **argv)
{
	static const char *known[] = {"-poses_from", "-lidar_id", "-tolerance",
			"-fake_timestamp", "-verbose", "-save_globalpos_file", "-robot", NULL};

	for (int i = 2; i < argc; i++)
	{
		if (argv[i][0] != '-')
			continue;

		// numero negativo e' valor, nao flag
		if (isdigit((unsigned char) argv[i][1]) || argv[i][1] == '.')
			continue;

		int found = 0;
		for (int k = 0; known[k] != NULL; k++)
		{
			if (strcmp(argv[i], known[k]) == 0)
			{
				found = 1;
				break;
			}
		}

		if (!found)
			exit(printf("graphslam_publish_globalpos: opcao desconhecida '%s'.\n"
					"Use: %s <poses_opt.dat> [-poses_from lidar|velodyne|odometry] [-lidar_id N]\n"
					"                        [-tolerance S] [-fake_timestamp on|off] [-verbose on|off]\n"
				"                        [-save_globalpos_file <arquivo>]\n",
					argv[i], argv[0]));
	}
}


static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "commandline", (char *) "poses_from",     CARMEN_PARAM_STRING, &poses_from,     0, NULL},
		{(char *) "commandline", (char *) "lidar_id",       CARMEN_PARAM_INT,    &lidar_id,       0, NULL},
		{(char *) "commandline", (char *) "tolerance",      CARMEN_PARAM_DOUBLE, &tolerance,      0, NULL},
		{(char *) "commandline", (char *) "fake_timestamp", CARMEN_PARAM_ONOFF,  &fake_timestamp, 0, NULL},
		{(char *) "commandline", (char *) "verbose",        CARMEN_PARAM_ONOFF,  &verbose,        0, NULL},
		{(char *) "commandline", (char *) "save_globalpos_file", CARMEN_PARAM_STRING, &save_globalpos_file, 0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	if (poses_from == NULL)
		poses_from = strdup("lidar");

	for (char *p = poses_from; *p; ++p)
		*p = tolower(*p);

	if ((strcmp(poses_from, "lidar") != 0) &&
		(strcmp(poses_from, "velodyne") != 0) &&
		(strcmp(poses_from, "odometry") != 0))
		exit(printf("graphslam_publish_globalpos: -poses_from '%s' invalido (use lidar, velodyne ou odometry).\n", poses_from));

	if (tolerance < 0.0)
		exit(printf("graphslam_publish_globalpos: -tolerance nao pode ser negativa.\n"));
}


int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use: %s <poses_opt.dat> [-poses_from lidar|velodyne|odometry] [-lidar_id N]\n"
				"                        [-tolerance S] [-fake_timestamp on|off] [-verbose on|off]\n"
				"                        [-save_globalpos_file <arquivo>]\n", argv[0]));

	// Saida em linha: este modulo normalmente roda sob o proccontrol, que
	// redireciona stdout para um pipe (block buffering). Sem isso as
	// estatisticas ficam presas no buffer e somem quando o processo e' morto.
	setvbuf(stdout, NULL, _IOLBF, 0);

	// O caminho do arquivo precisa ser copiado ANTES de qualquer chamada do
	// carmen: carmen_ipc_initialize() e carmen_param_install_params() consomem
	// e compactam o argv, e o argumento posicional deixa de estar em argv[1].
	char *poses_file = strdup(argv[1]);
	carmen_test_alloc(poses_file);

	check_unknown_flags(argc, argv);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	memset(odometry_queue, 0, sizeof(odometry_queue));
	load_corrected_poses(poses_file);

	carmen_localize_ackerman_define_globalpos_messages();

	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	// A odometria e' assinada sempre: mesmo em -poses_from lidar ela alimenta
	// v/phi da globalpos quando o arquivo traz odometry_timestamp valido.
	carmen_base_ackerman_subscribe_odometry_message(NULL,
			(carmen_handler_t) base_ackerman_message_handler, CARMEN_SUBSCRIBE_LATEST);

	if (strcmp(poses_from, "lidar") == 0)
	{
		carmen_velodyne_subscribe_variable_scan_message(NULL,
				(carmen_handler_t) variable_scan_message_handler, CARMEN_SUBSCRIBE_LATEST, lidar_id);
		printf("graphslam_publish_globalpos: publicando globalpos no ritmo do lidar %d.\n", lidar_id);
	}
	else if (strcmp(poses_from, "velodyne") == 0)
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL,
				(carmen_handler_t) partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
		printf("graphslam_publish_globalpos: publicando globalpos no ritmo do velodyne (partial scan).\n");
	}
	else
	{
		printf("graphslam_publish_globalpos: publicando globalpos no ritmo da odometria base_ackerman.\n");
	}

	printf("graphslam_publish_globalpos: tolerancia de casamento %.9f s%s.\n",
			tolerance, fake_timestamp ? "  [-fake_timestamp on: ACEITANDO a pose mais proxima, so' para depuracao]" : "");
	printf("graphslam_publish_globalpos: NAO e' preciso rodar o localize_ackerman.\n");

	carmen_ipc_dispatch();

	return (0);
}
