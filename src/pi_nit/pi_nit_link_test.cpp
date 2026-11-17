/*********************************************************
 pi_nit_link_test - testa o enlace com o Raspberry sem depender do CARMEN

 Nao usa IPC nem camera: manda uma imagem (arquivo, webcam ou sintetica) na
 taxa pedida e imprime as deteccoes que voltam. E' a primeira coisa a rodar
 quando o pi_nit_client_driver nao publica nada - separa problema de rede /
 Raspberry de problema de IPC / camera.

 Uso:
   ./pi_nit_link_test 192.168.1.20                       # imagem sintetica
   ./pi_nit_link_test 192.168.1.20 -image /tmp/foto.jpg
   ./pi_nit_link_test 192.168.1.20 -camera 0 -show
 *********************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include "pi_nit_protocol.h"
#include "pi_nit_zmq_client.hpp"

using namespace cv;
using namespace std;

static bool running = true;


static void
stop(int)
{
	running = false;
}


static double
now_seconds(void)
{
	struct timeval time_value;
	gettimeofday(&time_value, NULL);

	return (time_value.tv_sec + time_value.tv_usec / 1000000.0);
}


static Mat
synthetic_image(void)
{
	Mat image(480, 640, CV_8UC3, Scalar(40, 40, 40));
	rectangle(image, Rect(260, 120, 120, 320), Scalar(200, 200, 200), -1);

	return (image);
}


static void
letterbox(const Mat &source, Mat &destination, double &scale, int &pad_x, int &pad_y)
{
	int size = PI_NIT_INFERENCE_SIZE;

	scale = min((double) size / source.cols, (double) size / source.rows);
	int new_width = min((int) (source.cols * scale + 0.5), size);
	int new_height = min((int) (source.rows * scale + 0.5), size);
	pad_x = (size - new_width) / 2;
	pad_y = (size - new_height) / 2;

	destination.create(size, size, source.type());
	destination.setTo(Scalar(114, 114, 114));

	Mat resized;
	resize(source, resized, Size(new_width, new_height));
	resized.copyTo(destination(Rect(pad_x, pad_y, new_width, new_height)));
}


int
main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("Uso: %s <ip_do_raspberry> [-image <arquivo>] [-camera <n>] [-fps <n>] "
				"[-frames <n>] [-frame_port <n>] [-result_port <n>] [-show]\n", argv[0]);
		return (1);
	}

	const char *host = argv[1];
	const char *image_path = NULL;
	int camera_index = -1;
	int frame_port = PI_NIT_DEFAULT_FRAME_PORT;
	int result_port = PI_NIT_DEFAULT_RESULT_PORT;
	double fps = 15.0;
	int max_frames = 0;
	bool show = false;

	for (int i = 2; i < argc; i++)
	{
		if ((strcmp(argv[i], "-image") == 0) && (i + 1 < argc))
			image_path = argv[++i];
		else if ((strcmp(argv[i], "-camera") == 0) && (i + 1 < argc))
			camera_index = atoi(argv[++i]);
		else if ((strcmp(argv[i], "-fps") == 0) && (i + 1 < argc))
			fps = atof(argv[++i]);
		else if ((strcmp(argv[i], "-frames") == 0) && (i + 1 < argc))
			max_frames = atoi(argv[++i]);
		else if ((strcmp(argv[i], "-frame_port") == 0) && (i + 1 < argc))
			frame_port = atoi(argv[++i]);
		else if ((strcmp(argv[i], "-result_port") == 0) && (i + 1 < argc))
			result_port = atoi(argv[++i]);
		else if (strcmp(argv[i], "-show") == 0)
			show = true;
	}

	PiNitZmqClient client;
	if (!client.connect(host, frame_port, result_port))
		return (1);

	VideoCapture capture;
	if (camera_index >= 0)
	{
		capture.open(camera_index);
		if (!capture.isOpened())
		{
			fprintf(stderr, "erro: nao consegui abrir a camera %d\n", camera_index);
			return (1);
		}
	}

	Mat still_image;
	if (image_path != NULL)
	{
		still_image = imread(image_path);
		if (still_image.empty())
		{
			fprintf(stderr, "erro: nao consegui abrir %s\n", image_path);
			return (1);
		}
	}

	signal(SIGINT, stop);

	// Guarda os dados de letterbox de cada frame em voo
	struct PendingFrame
	{
		double scale;
		int    pad_x;
		int    pad_y;
		double sent_at;
		Mat    image;

		PendingFrame() : scale(1.0), pad_x(0), pad_y(0), sent_at(0.0) {}
	};
	PendingFrame pending[32];

	uint64_t frame_id = 0;
	int sent = 0;
	int received = 0;
	double period = (fps > 0.0) ? (1.0 / fps) : 0.0;
	double started_at = now_seconds();

	while (running && ((max_frames == 0) || (sent < max_frames)))
	{
		double loop_started_at = now_seconds();

		Mat image;
		if (capture.isOpened())
		{
			if (!capture.read(image) || image.empty())
				break;
		}
		else if (!still_image.empty())
			image = still_image;
		else
			image = synthetic_image();

		Mat letterboxed;
		double scale;
		int pad_x, pad_y;
		letterbox(image, letterboxed, scale, pad_x, pad_y);

		vector<uchar> encoded;
		vector<int> encode_parameters;
		encode_parameters.push_back(IMWRITE_JPEG_QUALITY);
		encode_parameters.push_back(80);
		imencode(".jpg", letterboxed, encoded, encode_parameters);

		frame_id++;
		if (client.send_frame(&encoded[0], encoded.size(), PI_NIT_FORMAT_JPEG,
				PI_NIT_INFERENCE_SIZE, PI_NIT_INFERENCE_SIZE, 0, frame_id, now_seconds()))
		{
			int slot = frame_id % 32;
			pending[slot].scale = scale;
			pending[slot].pad_x = pad_x;
			pending[slot].pad_y = pad_y;
			pending[slot].sent_at = now_seconds();
			pending[slot].image = image.clone();
			sent++;
		}

		pi_nit_result_header_t header;
		vector<pi_nit_detection_t> detections;
		while (client.receive_result(header, detections))
		{
			int slot = header.frame_id % 32;
			double round_trip_ms = (now_seconds() - pending[slot].sent_at) * 1000.0;
			received++;

			printf("frame %5lu | rtt %6.1f ms | hailo %5.1f ms | fila %5.1f ms | %d deteccao(oes)\n",
					(unsigned long) header.frame_id, round_trip_ms, header.inference_ms,
					header.queue_ms, header.num_detections);

			Mat canvas;
			if (show)
				canvas = pending[slot].image.clone();

			for (size_t i = 0; i < detections.size(); i++)
			{
				double x1 = (detections[i].x1 - pending[slot].pad_x) / pending[slot].scale;
				double y1 = (detections[i].y1 - pending[slot].pad_y) / pending[slot].scale;
				double x2 = (detections[i].x2 - pending[slot].pad_x) / pending[slot].scale;
				double y2 = (detections[i].y2 - pending[slot].pad_y) / pending[slot].scale;

				printf("        classe %d conf %.2f [%.0f,%.0f -> %.0f,%.0f]\n",
						detections[i].class_id, detections[i].score, x1, y1, x2, y2);

				if (show && !canvas.empty())
					rectangle(canvas, Point((int) x1, (int) y1), Point((int) x2, (int) y2),
							Scalar(0, 255, 0), 2);
			}

			if (show && !canvas.empty())
			{
				imshow("pi_nit_link_test", canvas);
				waitKey(1);
			}
		}

		double elapsed = now_seconds() - loop_started_at;
		if (period > elapsed)
			usleep((useconds_t) ((period - elapsed) * 1000000.0));
	}

	double total = now_seconds() - started_at;
	printf("\nenviados %d | recebidos %d | %.1f fps de retorno\n",
			sent, received, (total > 0.0) ? (received / total) : 0.0);

	if (received == 0)
	{
		fprintf(stderr, "nenhuma resposta: confira se o pi_nit_server esta rodando no %s e se as "
				"portas %d/%d estao liberadas\n", host, frame_port, result_port);
		return (1);
	}

	return (0);
}
