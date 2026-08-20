/*********************************************************
 pi_nit_camera_publisher - publica camera_message a partir de video ou fotos

 Serve para exercitar a cadeia IPC inteira no PC, sem camera, sem log e sem
 tocar na configuracao do carro:

   pi_nit_camera_publisher  ->  camera_message
                                     |
                            pi_nit_client_driver  ->  neural_detector_message
                                     |
                            multiple_object_tracker  ->  objetos moveis 3D

 A mensagem sai com undistorted=1, o que faz o process_image() pular a
 correcao de distorcao - assim nao e' preciso ter os parametros de calibracao
 da camera no arquivo .ini para o teste rodar.

 Uso:
   ./pi_nit_camera_publisher <numero_da_mensagem> <arquivo_ou_pasta> [opcoes]

 Exemplos:
   ./pi_nit_camera_publisher 3 $CARMEN_HOME/data/pi_nit/pedestres.avi -fps 15
   ./pi_nit_camera_publisher 3 /tmp/fotos/                 # pasta com imagens
   ./pi_nit_camera_publisher 3 /tmp/pessoas.jpg -fps 5     # uma foto so

 Opcoes:
   -fps <n>        taxa de publicacao          (padrao 15)
   -loop <0|1>     repete ao terminar          (padrao 1)
   -resize <LxA>   redimensiona antes de publicar, ex: 640x480
   -show <0|1>     abre janela com o que esta sendo publicado (padrao 0)
 *********************************************************/

#include <carmen/carmen.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/camera_drivers_messages.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <dirent.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

static int camera_message_number = -1;
static const char *source_path = NULL;
static double target_fps = 15.0;
static int loop_forever = 1;
static int resize_width = 0;
static int resize_height = 0;
static int show_output = 0;

static bool running = true;
static int published = 0;


static void
shutdown_module(int signal_number)
{
	if (signal_number == SIGINT)
	{
		printf("\npi_nit_camera_publisher: %d mensagens publicadas\n", published);
		running = false;
	}
}


static bool
is_directory(const char *path)
{
	struct stat info;

	return ((stat(path, &info) == 0) && S_ISDIR(info.st_mode));
}


static bool
has_image_extension(const string &name)
{
	static const char *extensions[] = { ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", NULL };

	string lower = name;
	transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

	for (int i = 0; extensions[i] != NULL; i++)
	{
		size_t position = lower.rfind(extensions[i]);
		if ((position != string::npos) && (position == lower.size() - strlen(extensions[i])))
			return (true);
	}

	return (false);
}


// Lista as imagens de uma pasta em ordem alfabetica, para a sequencia ser
// reproduzivel entre execucoes.
static vector<string>
list_images(const char *directory)
{
	vector<string> files;
	DIR *handle = opendir(directory);

	if (handle == NULL)
		return (files);

	struct dirent *entry;
	while ((entry = readdir(handle)) != NULL)
	{
		string name = entry->d_name;
		if (has_image_extension(name))
			files.push_back(string(directory) + "/" + name);
	}
	closedir(handle);

	sort(files.begin(), files.end());

	return (files);
}


static void
publish_image(const Mat &image, double timestamp)
{
	camera_image camera_image_data;
	camera_message message;

	camera_image_data.width = image.cols;
	camera_image_data.height = image.rows;
	camera_image_data.number_of_channels = image.channels();
	camera_image_data.size_in_bytes_of_each_element = 1;
	camera_image_data.data_type = unsigned_char_data;
	// image_size TEM que estar certo, senao o IPC quebra na serializacao
	camera_image_data.image_size = image.cols * image.rows * image.channels();
	camera_image_data.raw_data = (void *) image.data;

	message.number_of_images = 1;
	message.images = &camera_image_data;
	// undistorted=1 faz o process_image() pular a correcao de distorcao, que
	// exigiria os parametros de calibracao da camera no .ini
	message.undistorted = 1;
	message.timestamp = timestamp;
	message.host = carmen_get_host();

	camera_drivers_publish_message(camera_message_number, &message);
	published++;
}


static void
read_parameters(int argc, char **argv)
{
	if (argc < 3)
		carmen_die("%s: numero de parametros invalido.\n"
				"Uso: %s <numero_da_mensagem> <arquivo_ou_pasta> "
				"[-fps <n>] [-loop <0|1>] [-resize <LxA>] [-show <0|1>]\n\n"
				"Exemplos:\n"
				"  %s 3 $CARMEN_HOME/data/pi_nit/pedestres.avi -fps 15\n"
				"  %s 3 /tmp/fotos/\n"
				"  %s 3 /tmp/pessoas.jpg -fps 5\n", argv[0], argv[0], argv[0], argv[0], argv[0]);

	camera_message_number = atoi(argv[1]);
	source_path = argv[2];

	for (int i = 3; i < argc; i++)
	{
		if ((strcmp(argv[i], "-fps") == 0) && (i + 1 < argc))
			target_fps = atof(argv[++i]);
		else if ((strcmp(argv[i], "-loop") == 0) && (i + 1 < argc))
			loop_forever = atoi(argv[++i]);
		else if ((strcmp(argv[i], "-show") == 0) && (i + 1 < argc))
			show_output = atoi(argv[++i]);
		else if ((strcmp(argv[i], "-resize") == 0) && (i + 1 < argc))
		{
			if (sscanf(argv[++i], "%dx%d", &resize_width, &resize_height) != 2)
				carmen_die("%s: -resize espera o formato LARGURAxALTURA, ex: 640x480\n", argv[0]);
		}
	}
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	camera_drivers_define_message(camera_message_number);
	signal(SIGINT, shutdown_module);

	VideoCapture capture;
	vector<string> image_files;
	size_t image_index = 0;
	bool is_video = false;

	if (is_directory(source_path))
	{
		image_files = list_images(source_path);
		if (image_files.empty())
			carmen_die("pi_nit_camera_publisher: nenhuma imagem em %s\n", source_path);
		printf("pi_nit_camera_publisher: %d imagem(ns) em %s\n", (int) image_files.size(), source_path);
	}
	else
	{
		// Tenta como video; se o OpenCV nao abrir, trata como imagem unica
		capture.open(source_path);
		if (capture.isOpened())
		{
			is_video = true;
			printf("pi_nit_camera_publisher: video %s (%d frames)\n",
					source_path, (int) capture.get(CAP_PROP_FRAME_COUNT));
		}
		else
		{
			image_files.push_back(source_path);
			printf("pi_nit_camera_publisher: imagem unica %s\n", source_path);
		}
	}

	printf("pi_nit_camera_publisher: publicando camera%d a %.1f fps%s\n",
			camera_message_number, target_fps, loop_forever ? " (em loop)" : "");

	double period = (target_fps > 0.0) ? (1.0 / target_fps) : 0.0;
	Mat image;

	while (running)
	{
		double started_at = carmen_get_time();

		if (is_video)
		{
			if (!capture.read(image) || image.empty())
			{
				if (!loop_forever)
					break;
				capture.set(CAP_PROP_POS_FRAMES, 0);
				if (!capture.read(image) || image.empty())
					break;
			}
		}
		else
		{
			image = imread(image_files[image_index]);
			if (image.empty())
			{
				fprintf(stderr, "pi_nit_camera_publisher: nao consegui ler %s\n",
						image_files[image_index].c_str());
				image_index = (image_index + 1) % image_files.size();
				continue;
			}
			image_index++;
			if (image_index >= image_files.size())
			{
				if (!loop_forever)
					running = false;
				image_index = 0;
			}
		}

		if ((resize_width > 0) && (resize_height > 0))
			resize(image, image, Size(resize_width, resize_height));

		// O IPC serializa a partir do ponteiro: a imagem precisa ser continua
		if (!image.isContinuous())
			image = image.clone();

		publish_image(image, carmen_get_time());

		if (show_output)
		{
			imshow("pi_nit_camera_publisher", image);
			if ((waitKey(1) & 0xFF) == 'q')
				break;
		}

		if ((published % 100) == 0)
			printf("  %d mensagens publicadas (%dx%d)\n", published, image.cols, image.rows);

		double elapsed = carmen_get_time() - started_at;
		if (period > elapsed)
			usleep((useconds_t) ((period - elapsed) * 1000000.0));
	}

	printf("pi_nit_camera_publisher: encerrado, %d mensagens publicadas\n", published);
	carmen_ipc_disconnect();

	return (0);
}
