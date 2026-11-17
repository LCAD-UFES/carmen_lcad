/*********************************************************
 Pi NIT Client Driver - Detector de pessoas com Raspberry Pi 5 + Hailo-8L

 Roda no PC do CARMEN. Assina a imagem de ate MAX_CAMERAS cameras
 (camera_drivers), prepara cada frame 640x640 (letterbox), envia para o
 Raspberry via ZMQ, recebe as caixas detectadas e publica a
 neural_detector_message padrao do CARMEN - a mesma que o
 neural_image_tracker publica, de forma que todos os consumidores existentes
 funcionem sem alteracao.

 Uso:
   ./pi_nit_client_driver <camera_model> <msg_number> [<camera_model> <msg_number> ...] [opcoes]

 Exemplo com as 3 cameras do plano:
   ./pi_nit_client_driver intelbras1 3 intelbras2 4 intelbras3 5 \
       -pi_host 192.168.1.20 -fps 15

 Opcoes (commandline):
   -pi_host       <ip>     ip do Raspberry                    (padrao 192.168.1.20)
   -frame_port    <n>      porta de envio de frames           (padrao 5560)
   -result_port   <n>      porta de retorno das deteccoes     (padrao 5561)
   -fps           <n>      taxa maxima por camera             (padrao 15)
   -jpeg_quality  <n>      1..100, ou 0 para enviar BGR cru   (padrao 80)
   -confidence    <f>      confianca minima aceita            (padrao 0.4)
   -image         <n>      indice da imagem na camera_message (padrao 0)
   -undistort     <0|1>    corrige distorcao antes de enviar  (padrao 1)
   -person_only   <0|1>    publica apenas classe pessoa       (padrao 0)
   -track         <0|1>    preenche track_id por IoU          (padrao 1)
   -coco_ids      <0|1>    1 = publica ids COCO (pessoa=0) em vez de -1
   -publish       <0|1>    0 = so detecta e mostra, nao publica  (padrao 1)
   -ignore_bottom <f>      apaga a base da imagem antes de enviar (o capo do
                           carro): < 1 e' fracao da altura, >= 1 e' pixel,
                           0 desliga (padrao). Por camera no carmen.ini:
                           <modelo>_pi_nit_ignore_bottom  0.13
   -roi_top       <f>      corta o topo da imagem ANTES do letterbox: < 1 e'
                           fracao da altura, >= 1 e' pixel, 0 desliga
                           (padrao). Reduz o desperdicio do letterbox em
                           cameras largas (1280x720). Por camera no
                           carmen.ini: <modelo>_pi_nit_roi_top  0.15
   -roi_bottom    <f>      mesma ideia, cortando a base (alem do que o
                           ignore_bottom ja pinta de cinza - este de fato
                           reduz a imagem enviada). carmen.ini:
                           <modelo>_pi_nit_roi_bottom  0.10
   -show          <on|off> janela de visualizacao             (padrao off)
 *********************************************************/

#include <carmen/carmen.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/camera_drivers_messages.h>
#include <carmen/camera_drivers_process_image.hpp>
#include <carmen/neural_detector_interface.h>
#include <carmen/neural_detector_messages.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "pi_nit_interface.h"
#include "pi_nit_protocol.h"
#include "pi_nit_tracker.hpp"
#include "pi_nit_zmq_client.hpp"

using namespace cv;
using namespace std;

#define MAX_CAMERAS     5    // o plano usa 3; sobra folga
#define PENDING_FRAMES  32   // potencia de 2: buffer circular de frames em voo


// Dados de cada frame enviado, necessarios para desfazer o letterbox quando
// o resultado voltar do Raspberry.
typedef struct
{
	uint64_t frame_id;
	double   image_timestamp;
	double   sent_at;
	double   scale;
	int      pad_x;
	int      pad_y;
	int      crop_y;          // linhas cortadas do topo pelo roi_top, para somar de volta
	int      original_width;
	int      original_height;
	int      valid;
} pending_frame_t;


// Tudo o que e' por camera. Cada camera tem seu proprio rastreador, sua
// propria contagem de fps e sua propria mensagem publicada - uma camera
// travada nao afeta as outras.
typedef struct
{
	char           *model;
	int             message_number;
	bool            active;

	// Faixa da base da imagem que nao vai para a rede - o capo do carro.
	// < 1 e' fracao da altura, >= 1 e' pixel, 0 desliga. Fica por camera
	// porque depende da montagem: so' a da frente enxerga o capo.
	double          ignore_bottom;

	// Recorte vertical de ROI: fracao (< 1) ou pixel (>= 1) cortado do TOPO
	// e da BASE da imagem ANTES do letterbox - ao contrario do
	// ignore_bottom, que so pinta de cinza (a imagem continua 640x640 com a
	// mesma resolucao efetiva), o roi_top/roi_bottom reduz a imagem de
	// verdade: sobra mais pixel da regiao que interessa depois do resize
	// para 640x640. E' o que rende o "+15% de deteccoes" medido no
	// README.md - o ganho e' maior quanto mais larga for a camera (uma
	// 1280x720 desperdica muito mais no letterbox que uma 640x480).
	double          roi_top;
	double          roi_bottom;

	double          next_send_time;  // proximo ponto da grade de -fps (ver handle_camera_image)
	pending_frame_t pending[PENDING_FRAMES];
	PiNitTracker   *tracker;

	Mat             work_image;      // scratch do handler
	Mat             last_shown_image;

	int             frames_sent_total;
	int             frames_received_total;
	int             frames_sent_window;
	int             frames_received_window;
	double          last_round_trip_ms;
	double          last_inference_ms;
	double          last_result_time;
	int             last_detection_count;
} camera_context_t;


// Parametros globais
static char  *pi_host = (char *) "192.168.1.20";
static int    frame_port = PI_NIT_DEFAULT_FRAME_PORT;
static int    result_port = PI_NIT_DEFAULT_RESULT_PORT;
static int    target_fps = 15;
static int    jpeg_quality = 80;
static double min_confidence = 0.4;
static int    image_index = 0;
static int    undistort_image = 1;
// Quem decide as classes e' o servidor (PI_NIT_CLASSES no Pi, --classes no
// simulador). O cliente publica o que vier; este filtro so' existe para cortar
// carros e motos quando o servidor esta devolvendo tudo e o consumidor quer
// so' pedestre.
static int    person_only = 0;
static int    use_tracker = 1;
static int    use_coco_ids = 0;
// Com -publish 0 o modulo detecta e mostra, mas nao publica a
// neural_detector_message. E' o modo de comparar com o neural_detector rodando
// ao lado: os dois na mesma mensagem fariam o MOT receber duas sequencias de
// track_id e os objetos trocariam de identidade a cada frame.
static int    publish_message = 1;
static double ignore_bottom_all = 0.0;   // -ignore_bottom: vale para todas as cameras
static double roi_top_all = 0.0;         // -roi_top: vale para todas as cameras
static double roi_bottom_all = 0.0;      // -roi_bottom: vale para todas as cameras
static int    show_output = 0;

// Estado
static PiNitZmqClient  zmq_client;
static char            detector_host[256] = "";
static camera_context_t cameras[MAX_CAMERAS];
static int             number_of_cameras = 0;
static uint64_t        next_frame_id = 1;
static double          window_start = 0.0;
static Mat             letterboxed_image;
static vector<uchar>   encoded_buffer;


// O frame_id e' global (unico entre todas as cameras), entao o resultado que
// volta identifica sozinho a camera de origem. Este indice evita ter que
// varrer o vetor a cada resultado.
static int
camera_index_of(int message_number)
{
	for (int i = 0; i < number_of_cameras; i++)
	{
		if (cameras[i].message_number == message_number)
			return (i);
	}

	return (-1);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


// O MOT e o camera_viewer escolhem COMO ler a caixa pelo nome do modulo que vem
// dentro do campo host da mensagem:
//
//   multiple_object_tracker_main.cpp:235   camera_viewer_draw_functions.cpp:523
//     "neural_image_tracker" -> passa por rectify_bbox()
//     "neural_detector"      -> usa a caixa como veio
//     qualquer outro nome    -> nenhum dos dois ramos executa
//
// Nao ha 'else'. Com o host "pi_nit_client_driver@..." o MOT empurra um bbox_t
// com x/y/w/h NAO INICIALIZADOS para o vetor de predicoes, e o camera_viewer
// simplesmente nao desenha nada. Como publicamos exatamente a convencao do
// neural_detector (canto superior esquerdo, em pixels da imagem original da
// camera), anunciamos esse nome - mantendo "pi_nit" no texto para o modulo
// continuar identificavel no log e no print_ipc_message.
static void
build_detector_host(void)
{
	const char *full_host = carmen_get_host();      // "pi_nit_client_driver@MAQUINA"
	const char *machine = strchr(full_host, '@');

	snprintf(detector_host, sizeof(detector_host), "neural_detector_pi_nit@%s",
			machine ? machine + 1 : full_host);
}


static void
publish_detections(camera_context_t *camera, const vector<pi_nit_detection_t> &detections,
		const vector<int> &track_ids, double timestamp)
{
	neural_detector_message message;

	message.num_detected_objects = (int) detections.size();
	message.detected_objects = NULL;
	message.timestamp = timestamp;
	message.host = detector_host;

	vector<bbox_i> boxes(detections.size());
	for (size_t i = 0; i < detections.size(); i++)
	{
		double x1 = detections[i].x1;
		double y1 = detections[i].y1;
		double width = detections[i].x2 - detections[i].x1;
		double height = detections[i].y2 - detections[i].y1;

		boxes[i].x = (unsigned int) (x1 < 0.0 ? 0.0 : x1);
		boxes[i].y = (unsigned int) (y1 < 0.0 ? 0.0 : y1);
		boxes[i].w = (unsigned int) (width < 0.0 ? 0.0 : width);
		boxes[i].h = (unsigned int) (height < 0.0 ? 0.0 : height);
		boxes[i].prob = detections[i].score;

		// O MOT usa uma convencao propria de obj_id (multiple_object_tracker.cpp:1012):
		//   -1 pedestre | 1 Car | 2 motorcycle | 4 bus | 6 truck | resto "unknown"
		// que e' exatamente o id COCO menos 1 (pessoa 0 -> -1, carro 2 -> 1,
		// moto 3 -> 2, onibus 5 -> 4, caminhao 7 -> 6). O neural_detector faz a
		// mesma conta em neural_detector_main.cpp:135, entao somos um substituto
		// direto dele. Publicar o id COCO cru joga tudo no 'else' do MOT, com
		// dimensoes de veiculo e a estimativa de orientacao de veiculo ligada
		// (multiple_object_tracker.cpp:518).
		if (use_coco_ids)
			boxes[i].obj_id = detections[i].class_id;
		else
			boxes[i].obj_id = detections[i].class_id - 1;

		boxes[i].track_id = (i < track_ids.size()) ? track_ids[i] : 0;
	}

	if (!boxes.empty())
		message.detected_objects = &boxes[0];

	neural_detector_publish_message(camera->message_number, &message);
}


static void
publish_status(void)
{
	double now = carmen_get_time();
	double elapsed = now - window_start;

	if (elapsed < 1.0)
		return;

	for (int i = 0; i < number_of_cameras; i++)
	{
		camera_context_t *camera = &cameras[i];
		carmen_pi_nit_status_message message;

		message.camera_id = camera->message_number;
		message.connected = ((now - camera->last_result_time) < 1.0) ? 1 : 0;
		message.frames_sent = camera->frames_sent_total;
		message.frames_received = camera->frames_received_total;
		message.detections = camera->last_detection_count;
		message.fps_sent = camera->frames_sent_window / elapsed;
		message.fps_received = camera->frames_received_window / elapsed;
		message.round_trip_ms = camera->last_round_trip_ms;
		message.inference_ms = camera->last_inference_ms;
		message.timestamp = now;
		message.host = carmen_get_host();

		pi_nit_publish_status_message(&message);

		if (!message.connected)
			fprintf(stderr, "pi_nit: camera %d sem resposta do Raspberry %s ha %.1f s\n",
					camera->message_number, pi_host, now - camera->last_result_time);

		camera->frames_sent_window = 0;
		camera->frames_received_window = 0;
	}

	window_start = now;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Preparo da imagem e envio                                                                 //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


// Redimensiona mantendo a proporcao e completa com cinza ate 640x640
// (letterbox padrao do YOLO). Devolve a escala e o deslocamento aplicados,
// usados depois para levar as caixas de volta a imagem original.
// Uma imagem 640x480, que e' a do plano, vira 640x640 com 80 px de cinza em
// cima e embaixo - sem distorcer a pessoa.
static void
letterbox(const Mat &source, Mat &destination, double &scale, int &pad_x, int &pad_y)
{
	int size = PI_NIT_INFERENCE_SIZE;

	scale = min((double) size / source.cols, (double) size / source.rows);

	int new_width = (int) (source.cols * scale + 0.5);
	int new_height = (int) (source.rows * scale + 0.5);
	if (new_width > size)
		new_width = size;
	if (new_height > size)
		new_height = size;

	pad_x = (size - new_width) / 2;
	pad_y = (size - new_height) / 2;

	destination.create(size, size, source.type());
	destination.setTo(Scalar(114, 114, 114));

	Mat resized;
	resize(source, resized, Size(new_width, new_height), 0, 0, INTER_LINEAR);
	resized.copyTo(destination(Rect(pad_x, pad_y, new_width, new_height)));
}


// Corta as faixas do topo e/ou da base ANTES do letterbox - ao contrario do
// mask_ignored_region (que so pinta de cinza, mantendo os 640x640 cheios de
// area sem informacao), aqui a imagem enviada fica menor e mais quadrada,
// entao o resize para 640x640 usa mais pixel de verdade na regiao que
// interessa. Devolve o recorte e quantas linhas foram cortadas do topo (para
// desfazer o mapeamento das caixas depois).
//
// Desligado por padrao (roi_top = roi_bottom = 0): nao muda nada de quem
// nao configurar.
static void
crop_roi(camera_context_t *camera, const Mat &source, Mat &destination, int &crop_y)
{
	crop_y = 0;

	if ((camera->roi_top <= 0.0) && (camera->roi_bottom <= 0.0))
	{
		destination = source;
		return;
	}

	int top = (camera->roi_top < 1.0) ? (int) (camera->roi_top * source.rows + 0.5) : (int) camera->roi_top;
	int bottom = (camera->roi_bottom < 1.0) ? (int) (camera->roi_bottom * source.rows + 0.5) : (int) camera->roi_bottom;

	// Nunca corta a imagem inteira - se a soma passar da altura, corta so'
	// o que sobra (o cliente errou a conta no .ini, mas o modulo nao trava).
	if ((top + bottom) >= source.rows)
	{
		destination = source;
		return;
	}

	crop_y = top;
	destination = source(Rect(0, top, source.cols, source.rows - top - bottom));
}


// O capo do carro ocupa a base da imagem da camera da frente, e o detector o
// reconhece como veiculo em todo frame - uma caixa parada, colada na camera,
// que o MOT ainda tenta fundir com o LiDAR. Em vez de filtrar a deteccao
// depois, apagamos a faixa ANTES de enviar: a rede nem gasta tempo com ela, e
// a janela do -show mostra exatamente o que foi ignorado.
//
// O cinza e' o mesmo do letterbox de proposito: para a rede, e' area sem
// informacao, igual as bordas.
static void
mask_ignored_region(camera_context_t *camera, Mat &image)
{
	if (camera->ignore_bottom <= 0.0)
		return;

	double band = (camera->ignore_bottom < 1.0) ? camera->ignore_bottom * image.rows : camera->ignore_bottom;
	int first_row = image.rows - (int) (band + 0.5);

	if (first_row < 0)
		first_row = 0;
	if (first_row >= image.rows)
		return;

	image(Rect(0, first_row, image.cols, image.rows - first_row)).setTo(Scalar(114, 114, 114));
}


static Mat cropped_image;

static void
send_image_to_raspberry(camera_context_t *camera, Mat &image, double image_timestamp)
{
	double scale = 1.0;
	int pad_x = 0;
	int pad_y = 0;
	int crop_y = 0;

	crop_roi(camera, image, cropped_image, crop_y);
	mask_ignored_region(camera, cropped_image);
	letterbox(cropped_image, letterboxed_image, scale, pad_x, pad_y);

	const uint8_t *payload = NULL;
	size_t payload_len = 0;
	int format = PI_NIT_FORMAT_BGR8;

	if (jpeg_quality > 0)
	{
		vector<int> encode_parameters;
		encode_parameters.push_back(IMWRITE_JPEG_QUALITY);
		encode_parameters.push_back(jpeg_quality);

		if (!imencode(".jpg", letterboxed_image, encoded_buffer, encode_parameters))
		{
			fprintf(stderr, "pi_nit: falha ao codificar JPEG da camera %d\n", camera->message_number);
			return;
		}
		payload = &encoded_buffer[0];
		payload_len = encoded_buffer.size();
		format = PI_NIT_FORMAT_JPEG;
	}
	else
	{
		payload = letterboxed_image.data;
		payload_len = (size_t) letterboxed_image.total() * letterboxed_image.elemSize();
		format = PI_NIT_FORMAT_BGR8;
	}

	uint64_t frame_id = next_frame_id++;

	if (!zmq_client.send_frame(payload, payload_len, format, PI_NIT_INFERENCE_SIZE, PI_NIT_INFERENCE_SIZE,
			camera->message_number, frame_id, image_timestamp))
		return;   // fila cheia: frame descartado de proposito (o Pi esta atrasado)

	pending_frame_t *pending = &camera->pending[frame_id % PENDING_FRAMES];
	pending->frame_id = frame_id;
	pending->image_timestamp = image_timestamp;
	pending->sent_at = carmen_get_time();
	pending->scale = scale;
	pending->pad_x = pad_x;
	pending->pad_y = pad_y;
	pending->crop_y = crop_y;
	pending->original_width = image.cols;
	pending->original_height = image.rows;
	pending->valid = 1;

	camera->frames_sent_total++;
	camera->frames_sent_window++;

	if (show_output)
		camera->last_shown_image = image.clone();
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Recepcao dos resultados                                                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
draw_detections(camera_context_t *camera, const vector<pi_nit_detection_t> &detections,
		const vector<int> &track_ids, double fps)
{
	if (camera->last_shown_image.empty())
		return;

	Mat canvas = camera->last_shown_image.clone();

	for (size_t i = 0; i < detections.size(); i++)
	{
		Rect box((int) detections[i].x1, (int) detections[i].y1,
				(int) (detections[i].x2 - detections[i].x1), (int) (detections[i].y2 - detections[i].y1));
		rectangle(canvas, box, Scalar(0, 255, 0), 2);

		char label[64];
		snprintf(label, sizeof(label), "pessoa %d (%.2f)",
				(i < track_ids.size()) ? track_ids[i] : 0, detections[i].score);
		putText(canvas, label, Point(box.x, box.y - 6), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
	}

	char header[128];
	snprintf(header, sizeof(header), "cam %d | %.1f fps | rtt %.0f ms | hailo %.0f ms | %d pessoa(s)",
			camera->message_number, fps, camera->last_round_trip_ms, camera->last_inference_ms,
			(int) detections.size());
	putText(canvas, header, Point(10, 24), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);

	char window_name[32];
	snprintf(window_name, sizeof(window_name), "pi_nit cam %d", camera->message_number);
	imshow(window_name, canvas);
	waitKey(1);
}


static void
process_pending_results(void)
{
	pi_nit_result_header_t header;
	vector<pi_nit_detection_t> detections;

	while (zmq_client.receive_result(header, detections))
	{
		if (header.client_id != zmq_client.client_id())
		{
			fprintf(stderr, "pi_nit: resultado de outro cliente (%u) ignorado - ha outro pi_nit_client_driver "
					"usando o mesmo Raspberry?\n", header.client_id);
			continue;
		}

		int index = camera_index_of(header.camera_id);
		if (index < 0)
			continue;   // camera desconhecida: resultado de uma execucao anterior

		camera_context_t *camera = &cameras[index];
		pending_frame_t *pending = &camera->pending[header.frame_id % PENDING_FRAMES];
		if (!pending->valid || (pending->frame_id != header.frame_id))
			continue;   // frame antigo demais, ja sobrescrito no buffer circular

		double now = carmen_get_time();

		// Desfaz o letterbox: coordenadas 640x640 -> imagem original
		vector<pi_nit_detection_t> mapped;
		mapped.reserve(detections.size());
		for (size_t i = 0; i < detections.size(); i++)
		{
			if (detections[i].score < min_confidence)
				continue;
			if (person_only && (detections[i].class_id != PI_NIT_CLASS_PERSON))
				continue;

			pi_nit_detection_t detection = detections[i];
			detection.x1 = (float) ((detection.x1 - pending->pad_x) / pending->scale);
			detection.y1 = (float) ((detection.y1 - pending->pad_y) / pending->scale + pending->crop_y);
			detection.x2 = (float) ((detection.x2 - pending->pad_x) / pending->scale);
			detection.y2 = (float) ((detection.y2 - pending->pad_y) / pending->scale + pending->crop_y);

			detection.x1 = (float) max(0.0, min((double) detection.x1, (double) pending->original_width - 1.0));
			detection.y1 = (float) max(0.0, min((double) detection.y1, (double) pending->original_height - 1.0));
			detection.x2 = (float) max(0.0, min((double) detection.x2, (double) pending->original_width - 1.0));
			detection.y2 = (float) max(0.0, min((double) detection.y2, (double) pending->original_height - 1.0));

			if ((detection.x2 > detection.x1) && (detection.y2 > detection.y1))
				mapped.push_back(detection);
		}

		vector<int> track_ids;
		if (use_tracker)
			track_ids = camera->tracker->update(mapped);
		else
			track_ids.assign(mapped.size(), 0);

		if (publish_message)
			publish_detections(camera, mapped, track_ids, pending->image_timestamp);

		camera->last_round_trip_ms = (now - pending->sent_at) * 1000.0;
		camera->last_inference_ms = header.inference_ms;
		camera->last_detection_count = (int) mapped.size();
		camera->last_result_time = now;
		camera->frames_received_total++;
		camera->frames_received_window++;
		pending->valid = 0;

		if (show_output)
		{
			double elapsed = now - window_start;
			draw_detections(camera, mapped, track_ids,
					(elapsed > 0.0) ? (camera->frames_received_window / elapsed) : 0.0);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
handle_camera_image(int index, camera_message *message)
{
	camera_context_t *camera = &cameras[index];
	double now = carmen_get_time();

	// Limita a taxa por camera contra uma GRADE de tempo, nao contra o instante
	// do ultimo envio. Medir a partir do envio soma o preparo da imagem
	// (undistort + letterbox + JPEG) ao periodo: com a camera na mesma taxa do
	// alvo, o frame seguinte chega sempre alguns ms antes do prazo e e'
	// descartado. Era isso que transformava 15 Hz de camera em 9 fps no
	// detector. A folga de 10% absorve o jitter do IPC.
	if (target_fps > 0)
	{
		double period = 1.0 / target_fps;

		if ((now + period * 0.1) < camera->next_send_time)
			return;

		camera->next_send_time += period;
		if (camera->next_send_time < now)       // ficamos para tras: ressincroniza
			camera->next_send_time = now + period;
	}

	if ((message == NULL) || (message->number_of_images <= image_index))
		return;

	// process_image() desta arvore devolve so a imagem; as intrinsecas corrigidas nao fazem
	// parte da assinatura (e o pi_nit nao as usa -- trabalha em pixels da imagem original).
	process_image(message, image_index, camera->model, undistort_image, camera->work_image);

	if (message->undistorted == 2)   // retrocompatibilidade (mesmo tratamento do neural_image_tracker)
		cvtColor(camera->work_image, camera->work_image, cv::COLOR_RGB2BGR);

	if ((camera->work_image.cols <= 0) || (camera->work_image.rows <= 0))
		return;

	send_image_to_raspberry(camera, camera->work_image, message->timestamp);
}


// O IPC nao passa dado de usuario para o handler, entao e' preciso um por camera.
static void camera_image_handler_0(camera_message *message) { handle_camera_image(0, message); }
static void camera_image_handler_1(camera_message *message) { handle_camera_image(1, message); }
static void camera_image_handler_2(camera_message *message) { handle_camera_image(2, message); }
static void camera_image_handler_3(camera_message *message) { handle_camera_image(3, message); }
static void camera_image_handler_4(camera_message *message) { handle_camera_image(4, message); }

static carmen_handler_t camera_image_handlers[MAX_CAMERAS] =
{
	(carmen_handler_t) camera_image_handler_0,
	(carmen_handler_t) camera_image_handler_1,
	(carmen_handler_t) camera_image_handler_2,
	(carmen_handler_t) camera_image_handler_3,
	(carmen_handler_t) camera_image_handler_4,
};


static void
shutdown_module(int signal_number)
{
	if (signal_number == SIGINT)
	{
		printf("\npi_nit: encerrando\n");
		for (int i = 0; i < number_of_cameras; i++)
			printf("  camera %d: enviados %d, recebidos %d\n", cameras[i].message_number,
					cameras[i].frames_sent_total, cameras[i].frames_received_total);

		zmq_client.disconnect();
		carmen_ipc_disconnect();
		exit(0);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Inicializacao                                                                             //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
read_parameters(int argc, char **argv)
{
	if (argc < 3)
		carmen_die("%s: numero de parametros invalido.\n"
				"Uso: %s <camera_model> <msg_number> [<camera_model> <msg_number> ...] "
				"[-pi_host <ip>] [-frame_port <n>] [-result_port <n>] [-fps <n>] [-jpeg_quality <n>] "
				"[-confidence <f>] [-image <n>] [-undistort <0|1>] [-person_only <0|1>] [-track <0|1>] "
				"[-coco_ids <0|1>] [-publish <0|1>] [-ignore_bottom <f>] [-roi_top <f>] [-roi_bottom <f>] "
				"[-show <on|off>]\n", argv[0], argv[0]);

	// Pares <modelo> <numero>, ate encontrar a primeira opcao com '-'
	int argument_index = 1;
	while ((argument_index + 1 < argc) && (argv[argument_index][0] != '-') && (number_of_cameras < MAX_CAMERAS))
	{
		cameras[number_of_cameras].model = argv[argument_index];
		cameras[number_of_cameras].message_number = atoi(argv[argument_index + 1]);
		cameras[number_of_cameras].active = true;
		number_of_cameras++;
		argument_index += 2;
	}

	if (number_of_cameras == 0)
		carmen_die("%s: nenhuma camera informada. Use <camera_model> <msg_number>.\n", argv[0]);

	carmen_param_allow_unfound_variables(1);
	vector<carmen_param_t> param_list =
	{
		// Primeiro o arquivo de parametros do carro, na secao da primeira camera:
		//   <camera_model>_pi_nit_host      192.168.1.20
		//   <camera_model>_pi_nit_fps       15
		// Depois a linha de comando, que tem prioridade sobre o arquivo.
		{cameras[0].model,       (char *) "pi_nit_host",  CARMEN_PARAM_STRING, &pi_host,         0, NULL},
		{cameras[0].model,       (char *) "pi_nit_fps",   CARMEN_PARAM_INT,    &target_fps,      0, NULL},

		{(char *) "commandline", (char *) "pi_host",      CARMEN_PARAM_STRING, &pi_host,         0, NULL},
		{(char *) "commandline", (char *) "frame_port",   CARMEN_PARAM_INT,    &frame_port,      0, NULL},
		{(char *) "commandline", (char *) "result_port",  CARMEN_PARAM_INT,    &result_port,     0, NULL},
		{(char *) "commandline", (char *) "fps",          CARMEN_PARAM_INT,    &target_fps,      0, NULL},
		{(char *) "commandline", (char *) "jpeg_quality", CARMEN_PARAM_INT,    &jpeg_quality,    0, NULL},
		{(char *) "commandline", (char *) "confidence",   CARMEN_PARAM_DOUBLE, &min_confidence,  0, NULL},
		{(char *) "commandline", (char *) "image",        CARMEN_PARAM_INT,    &image_index,     0, NULL},
		{(char *) "commandline", (char *) "undistort",    CARMEN_PARAM_INT,    &undistort_image, 0, NULL},
		{(char *) "commandline", (char *) "person_only",  CARMEN_PARAM_INT,    &person_only,     0, NULL},
		{(char *) "commandline", (char *) "track",        CARMEN_PARAM_INT,    &use_tracker,     0, NULL},
		{(char *) "commandline", (char *) "coco_ids",     CARMEN_PARAM_INT,    &use_coco_ids,    0, NULL},
		{(char *) "commandline", (char *) "publish",      CARMEN_PARAM_INT,    &publish_message, 0, NULL},
		{(char *) "commandline", (char *) "ignore_bottom", CARMEN_PARAM_DOUBLE, &ignore_bottom_all, 0, NULL},
		{(char *) "commandline", (char *) "roi_top",       CARMEN_PARAM_DOUBLE, &roi_top_all,       0, NULL},
		{(char *) "commandline", (char *) "roi_bottom",    CARMEN_PARAM_DOUBLE, &roi_bottom_all,    0, NULL},
		{(char *) "commandline", (char *) "show",         CARMEN_PARAM_ONOFF,  &show_output,     0, NULL},
	};

	// A faixa ignorada tem uma entrada POR camera: quem enxerga o capo e' so'
	// a da frente, e a altura dele muda com a montagem. No carmen.ini:
	//   intelbras1_pi_nit_ignore_bottom   0.13
	for (int i = 0; i < number_of_cameras; i++)
		param_list.push_back({cameras[i].model, (char *) "pi_nit_ignore_bottom", CARMEN_PARAM_DOUBLE,
				&cameras[i].ignore_bottom, 0, NULL});

	// O ROI tambem e' por camera - a que tem o campo de visao mais largo
	// (ex.: 1280x720) e' a que ganha mais recortando antes do letterbox. No
	// carmen.ini:
	//   intelbras1_pi_nit_roi_top      0.15
	//   intelbras1_pi_nit_roi_bottom   0.10
	for (int i = 0; i < number_of_cameras; i++)
	{
		param_list.push_back({cameras[i].model, (char *) "pi_nit_roi_top", CARMEN_PARAM_DOUBLE,
				&cameras[i].roi_top, 0, NULL});
		param_list.push_back({cameras[i].model, (char *) "pi_nit_roi_bottom", CARMEN_PARAM_DOUBLE,
				&cameras[i].roi_bottom, 0, NULL});
	}

	carmen_param_install_params(argc, argv, &param_list[0], (int) param_list.size());

	// A linha de comando vale para todas as cameras e tem prioridade sobre o
	// arquivo, igual ao resto dos parametros.
	if (ignore_bottom_all > 0.0)
		for (int i = 0; i < number_of_cameras; i++)
			cameras[i].ignore_bottom = ignore_bottom_all;
	if (roi_top_all > 0.0)
		for (int i = 0; i < number_of_cameras; i++)
			cameras[i].roi_top = roi_top_all;
	if (roi_bottom_all > 0.0)
		for (int i = 0; i < number_of_cameras; i++)
			cameras[i].roi_bottom = roi_bottom_all;

	if (jpeg_quality > 100)
		jpeg_quality = 100;

	printf("pi_nit: %d camera(s) -> %s:%d/%d | %d fps por camera | jpeg %d | confianca %.2f | %s\n",
			number_of_cameras, pi_host, frame_port, result_port, target_fps, jpeg_quality, min_confidence,
			person_only ? "so pedestre" : "todas as classes do servidor");
	for (int i = 0; i < number_of_cameras; i++)
	{
		printf("        camera %s, mensagem %d", cameras[i].model, cameras[i].message_number);
		if (cameras[i].ignore_bottom > 0.0)
		{
			if (cameras[i].ignore_bottom < 1.0)
				printf(" | ignorando os %.0f%% de baixo (capo)", cameras[i].ignore_bottom * 100.0);
			else
				printf(" | ignorando as %.0f linhas de baixo (capo)", cameras[i].ignore_bottom);
		}
		if ((cameras[i].roi_top > 0.0) || (cameras[i].roi_bottom > 0.0))
			printf(" | roi topo %.2f base %.2f", cameras[i].roi_top, cameras[i].roi_bottom);
		printf("\n");
	}
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	// 'cameras' e' estatico: os campos POD ja vem zerados e os Mat ja vem
	// construidos vazios. Nada de memset aqui - passar memset por cima de um
	// cv::Mat quebra o contador de referencias.
	read_parameters(argc, argv);

	window_start = carmen_get_time();
	for (int i = 0; i < number_of_cameras; i++)
	{
		cameras[i].tracker = new PiNitTracker();
		// so avisa "sem resposta" depois de 1 s de silencio real
		cameras[i].last_result_time = window_start;
	}

	// Um frame em voo por camera: se o Raspberry atrasa, o envio falha e o
	// frame e' descartado aqui mesmo, sem acumular latencia.
	if (!zmq_client.connect(pi_host, frame_port, result_port, number_of_cameras))
		carmen_die("pi_nit: nao foi possivel iniciar os sockets ZMQ com %s\n", pi_host);

	pi_nit_define_status_message();
	build_detector_host();

	signal(SIGINT, shutdown_module);

	for (int i = 0; i < number_of_cameras; i++)
	{
		neural_detector_define_message(cameras[i].message_number);
		camera_drivers_subscribe_message(cameras[i].message_number, NULL, camera_image_handlers[i],
				CARMEN_SUBSCRIBE_LATEST);
		if (publish_message)
			printf("pi_nit: publicando neural_detector_message_%d_name como host '%s'\n",
					cameras[i].message_number, detector_host);
		else
			printf("pi_nit: -publish 0: camera %d so' detecta e mostra, nao publica\n",
					cameras[i].message_number);
	}

	while (true)
	{
		IPC_listen(2);            // dispara os handlers das cameras
		process_pending_results();
		publish_status();
	}

	return (0);
}
