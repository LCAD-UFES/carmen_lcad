
/*
 * argos_unitree_camera -- camera frontal do ARGOS (Unitree Go2) pela rede,
 * usando o unitree_sdk2 (DDS). Roda em PC externo, pelo cabo, igual ao
 * argos_unitree_driver e ao xsens_argos.
 *
 * Substitui, no process do ARGOS, a entrada
 *
 *   camera1   sensors   ./camera_drivers unitree1 1
 *
 * que usa o caminho antigo (unitree1_model = udp_rtp_h264, multicast 230.1.1.1:1720
 * no carmen-argos.ini). Esse stream RTP e' do Go1/B1; o Go2 entrega a imagem por
 * DDS, via VideoClient::GetImageSample(), em JPEG.
 *
 * Fluxo:
 *
 *   Go2 --DDS/eth--> VideoClient::GetImageSample() (JPEG)
 *        -> cv::imdecode (BGR) -> camera_message (IPC)
 *        -> camera_viewer / neural_detector / image_path_projector ...
 *
 * A mensagem sai no MESMO formato do camera_drivers (camera_message, BGR,
 * unsigned_char, undistorted = 0), entao quem ja' consumia a camera continua
 * funcionando sem alteracao.
 *
 * Uso:
 *   ./argos_unitree_camera -network_interface eth0 -camera_id 1
 *   ./argos_unitree_camera -network_interface eth0 -camera_id 1 -resize_factor 0.5
 */

#include <atomic>
#include <cstdio>
#include <stdexcept>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <signal.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <carmen/carmen.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/camera_drivers_messages.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Parametros                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static char  *g_network_interface = NULL;   // vazio = CycloneDDS escolhe sozinho
static int    g_camera_id = 1;              // publica em "camera<N>"
static double g_publish_hz = 15.0;          // o Go2 entrega ~10-15 fps
static double g_resize_factor = 1.0;        // 0.5 = metade da largura e da altura
static double g_client_timeout = 1.0;       // s, timeout do RPC do VideoClient
static int    g_verbose = 0;


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Estado compartilhado (thread de captura -> thread do IPC)                                    //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
//
// Mesma separacao dos outros drivers do ARGOS: GetImageSample() e' um RPC com
// timeout sobre DDS, e decodificar JPEG custa milissegundos. Nada disso pode rodar
// dentro do laco do IPC -- travaria a recepcao de mensagens do carmen INTEIRO
// enquanto o robo demora a responder. Entao: thread de captura decodifica e guarda
// o ultimo frame; a thread do IPC so' copia e publica.

static std::mutex g_frame_mutex;
static cv::Mat    g_frame_bgr;          // ultimo frame pronto, em BGR
static double     g_frame_timestamp = 0.0;
static bool       g_frame_is_new = false;

static std::atomic<bool> g_running(true);
static std::atomic<long> g_frames_captured(0);
static std::atomic<long> g_frames_published(0);


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publisher (thread do IPC)                                                                    //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_camera_message(void *clientData __attribute__ ((unused)),
		unsigned long currentTime __attribute__ ((unused)),
		unsigned long scheduledTime __attribute__ ((unused)))
{
	static camera_message message;
	static camera_image image;
	static bool message_ready = false;
	static int last_width = -1, last_height = -1;
	static int heartbeat = 0;
	static bool had_frame_before = false;

	cv::Mat frame;
	double timestamp = 0.0;

	{
		std::lock_guard<std::mutex> lock(g_frame_mutex);
		if (!g_frame_is_new)
		{
			// Nao republica o mesmo frame: encheria a fila do IPC com imagem
			// repetida e faria os consumidores acharem que a camera esta' viva
			// e congelada, em vez de perceberem a ausencia.
			if (!had_frame_before && (heartbeat % (int) g_publish_hz) == 0)
				printf("argos_unitree_camera: ainda sem imagem do robo (interface '%s').\n",
						g_network_interface);
			heartbeat++;
			return;
		}
		frame = g_frame_bgr.clone();
		timestamp = g_frame_timestamp;
		g_frame_is_new = false;
	}

	if (!had_frame_before)
	{
		printf("argos_unitree_camera: recebendo imagem %dx%d, publicando em 'camera%d'.\n",
				frame.cols, frame.rows, g_camera_id);
		had_frame_before = true;
	}

	// (Re)monta a struct so' quando a resolucao muda. image_size TEM que bater com
	// o buffer, senao o IPC quebra na serializacao (ver o comentario em
	// camera_drivers_messages.h:42).
	if (!message_ready || frame.cols != last_width || frame.rows != last_height)
	{
		if (message_ready)
			free(image.raw_data);

		image.width = frame.cols;
		image.height = frame.rows;
		image.number_of_channels = frame.channels();
		image.size_in_bytes_of_each_element = sizeof(unsigned char);
		image.data_type = unsigned_char_data;
		image.image_size = image.width * image.height * image.number_of_channels *
				image.size_in_bytes_of_each_element;
		image.raw_data = malloc(image.image_size);

		if (image.raw_data == NULL)
		{
			fprintf(stderr, "argos_unitree_camera: malloc de %d bytes falhou.\n", image.image_size);
			return;
		}

		message.number_of_images = 1;
		message.images = &image;
		// undistorted = 1 diz ao consumidor "nao remapeie". Com 0, o process_image
		// monta um initUndistortRectifyMap com os parametros do <camera_name> no ini
		// (camera_drivers_process_image.cpp:48-76) -- e os da unitree1 no
		// carmen-argos.ini sao placeholder: k1 = k2 = k3 = p1 = p2 = 0.5, distorcao
		// radial gigante, que estica a imagem inteira. Quando a Go2 for calibrada de
		// verdade, poe os valores no ini e volta este campo para 0.
		message.undistorted = 1;
		message.host = carmen_get_host();

		last_width = frame.cols;
		last_height = frame.rows;
		message_ready = true;
	}

	// cv::Mat pode ter padding no fim da linha; copiar linha a linha evita mandar
	// lixo quando isContinuous() e' falso (acontece depois de crop/resize).
	const size_t row_bytes = (size_t) image.width * image.number_of_channels;
	unsigned char *dst = (unsigned char *) image.raw_data;
	for (int r = 0; r < image.height; r++)
		memcpy(dst + r * row_bytes, frame.ptr<unsigned char>(r), row_bytes);

	message.timestamp = timestamp;

	camera_drivers_publish_message(g_camera_id, &message);
	g_frames_published++;

	if (g_verbose && (heartbeat % ((int) g_publish_hz * 5) == 0))
		printf("argos_unitree_camera: %ld capturados, %ld publicados.\n",
				g_frames_captured.load(), g_frames_published.load());

	heartbeat++;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Thread de captura                                                                            //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
capture_loop(unitree::robot::go2::VideoClient *video_client)
{
	std::vector<uint8_t> jpeg;
	int consecutive_errors = 0;

	while (g_running.load())
	{
		jpeg.clear();
		const int32_t ret = video_client->GetImageSample(jpeg);

		if (ret != 0 || jpeg.empty())
		{
			// Nao aborta: o Go2 devolve erro enquanto o servico de video ainda esta'
			// subindo, e tambem quando o cabo cai. Avisa de vez em quando e insiste.
			if ((consecutive_errors++ % 50) == 0)
				fprintf(stderr, "argos_unitree_camera: GetImageSample devolveu %d "
						"(%zu bytes). Tentando de novo.\n", (int) ret, jpeg.size());
			usleep(200000);
			continue;
		}
		consecutive_errors = 0;

		// imdecode devolve BGR, que e' exatamente o que a camera_message do carmen
		// trafega. O camera_viewer so' faz RGB->BGR quando undistorted == 2
		// (camera_viewer.cpp:138), flag de retrocompatibilidade que o readlog poe
		// em log antigo (readlog.cpp:1832); driver ao vivo publica BGR com
		// undistorted = 0. Converter aqui trocava R com B e a imagem saia azulada.
		cv::Mat bgr = cv::imdecode(jpeg, cv::IMREAD_COLOR);
		if (bgr.empty())
		{
			fprintf(stderr, "argos_unitree_camera: imdecode falhou em %zu bytes de JPEG.\n",
					jpeg.size());
			continue;
		}

		if (g_resize_factor > 0.0 && g_resize_factor != 1.0)
			cv::resize(bgr, bgr, cv::Size(), g_resize_factor, g_resize_factor, cv::INTER_AREA);

		{
			std::lock_guard<std::mutex> lock(g_frame_mutex);
			g_frame_bgr = bgr;
			// Carimba na captura, nao na publicacao: o consumidor precisa do
			// instante da imagem, sem a latencia do timer do IPC embutida.
			g_frame_timestamp = carmen_get_time();
			g_frame_is_new = true;
		}

		g_frames_captured++;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Inicializacoes                                                                               //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
read_parameters(int argc, char **argv)
{
	carmen_param_allow_unfound_variables(1);
	carmen_param_t param_list[] =
	{
		{(char *) "commandline", (char *) "network_interface", CARMEN_PARAM_STRING, &g_network_interface, 0, NULL},
		{(char *) "commandline", (char *) "camera_id",         CARMEN_PARAM_INT,    &g_camera_id,         0, NULL},
		{(char *) "commandline", (char *) "publish_hz",        CARMEN_PARAM_DOUBLE, &g_publish_hz,        0, NULL},
		{(char *) "commandline", (char *) "resize_factor",     CARMEN_PARAM_DOUBLE, &g_resize_factor,     0, NULL},
		{(char *) "commandline", (char *) "client_timeout",    CARMEN_PARAM_DOUBLE, &g_client_timeout,    0, NULL},
		{(char *) "commandline", (char *) "verbose",           CARMEN_PARAM_ONOFF,  &g_verbose,           0, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
	carmen_param_allow_unfound_variables(0);

	if (g_network_interface == NULL)
		g_network_interface = (char *) "";

	if (g_publish_hz <= 0.0)
		g_publish_hz = 15.0;

	if (g_camera_id < 1 || g_camera_id > MAX_CAMERA_ID)
	{
		fprintf(stderr, "argos_unitree_camera: -camera_id %d fora da faixa 1..%d.\n",
				g_camera_id, MAX_CAMERA_ID);
		exit(1);
	}
}


static void
shutdown_module(int signo)
{
	static int done = 0;

	if (!done)
	{
		done = 1;
		g_running.store(false);
		carmen_ipc_disconnect();
		printf("\nargos_unitree_camera: desconectado do IPC. Sig: %d.\n", signo);
	}

	exit(0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////



// O CycloneDDS LANCA excecao quando a interface nao existe ou esta' sem carrier (robo
// desligado, cabo fora). Sem capturar, o std::terminate derruba o processo com core dump
// e a unica pista fica no stderr -- que o proccontrol nao mostra. Vira loop de respawn
// sem explicacao. Aqui a mensagem e' explicita e a saida e' limpa.
static void
init_dds_or_die(const char *module, int domain_id, const char *network_interface)
{
	try
	{
		unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
	}
	catch (const std::exception &e)
	{
		fprintf(stderr, "\n%s: NAO consegui subir o DDS na interface '%s'.\n", module, network_interface);
		fprintf(stderr, "%s: confira com `ip -br link show %s` -- precisa estar UP e com carrier\n",
				module, network_interface);
		fprintf(stderr, "%s: (robo desligado ou cabo fora da' exatamente este erro).\n", module);
		fprintf(stderr, "%s: detalhe do CycloneDDS: %s\n\n", module, e.what());
		exit(1);
	}
}


int
main(int argc, char **argv)
{
	signal(SIGINT, shutdown_module);
	signal(SIGTERM, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	printf("argos_unitree_camera: interface '%s', camera_id %d, %.0f Hz, resize %.2f.\n",
			g_network_interface, g_camera_id, g_publish_hz, g_resize_factor);

	camera_drivers_define_message(g_camera_id);

	// Conecta no DDS do robo pela interface indicada (o cabo). Tem que vir antes de
	// qualquer Client do SDK.
	init_dds_or_die("argos_unitree_camera", 0, g_network_interface);

	unitree::robot::go2::VideoClient video_client;
	video_client.SetTimeout((float) g_client_timeout);
	video_client.Init();

	std::thread capture_thread(capture_loop, &video_client);

	carmen_ipc_addPeriodicTimer(1.0 / g_publish_hz,
			(TIMER_HANDLER_TYPE) publish_camera_message, NULL);

	carmen_ipc_dispatch();

	g_running.store(false);
	capture_thread.join();

	return (0);
}
