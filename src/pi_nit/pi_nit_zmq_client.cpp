#include "pi_nit_zmq_client.hpp"

#include <zmq.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>


PiNitZmqClient::PiNitZmqClient() :
		context_(NULL), frame_socket_(NULL), result_socket_(NULL), client_id_(0)
{
	// Identificador do processo, ecoado pelo servidor. Serve para detectar
	// (e avisar) quando dois clientes disputam o mesmo Raspberry.
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	client_id_ = (uint32_t) (ts.tv_nsec ^ (getpid() << 16));
	if (client_id_ == 0)
		client_id_ = 1;
}


PiNitZmqClient::~PiNitZmqClient()
{
	disconnect();
}


bool
PiNitZmqClient::connect(const std::string &host, int frame_port, int result_port, int queue_depth)
{
	disconnect();

	context_ = zmq_ctx_new();
	if (context_ == NULL)
	{
		fprintf(stderr, "pi_nit: zmq_ctx_new falhou: %s\n", zmq_strerror(zmq_errno()));
		return (false);
	}

	frame_socket_ = zmq_socket(context_, ZMQ_PUSH);
	result_socket_ = zmq_socket(context_, ZMQ_PULL);
	if ((frame_socket_ == NULL) || (result_socket_ == NULL))
	{
		fprintf(stderr, "pi_nit: zmq_socket falhou: %s\n", zmq_strerror(zmq_errno()));
		disconnect();
		return (false);
	}

	int zero = 0;
	int rcvhwm = 4;
	int reconnect_ivl_max = 2000;   // backoff de reconexao ate 2 s

	// Um frame em voo por camera. Nada de ZMQ_CONFLATE: ele guardaria "a
	// ultima mensagem" do socket inteiro e comeria os frames das outras
	// cameras (ver comentario no .hpp).
	int sndhwm = (queue_depth > 0) ? queue_depth : 1;
	zmq_setsockopt(frame_socket_, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));
	zmq_setsockopt(frame_socket_, ZMQ_LINGER, &zero, sizeof(zero));
	zmq_setsockopt(frame_socket_, ZMQ_RECONNECT_IVL_MAX, &reconnect_ivl_max, sizeof(reconnect_ivl_max));

	zmq_setsockopt(result_socket_, ZMQ_RCVHWM, &rcvhwm, sizeof(rcvhwm));
	zmq_setsockopt(result_socket_, ZMQ_LINGER, &zero, sizeof(zero));
	zmq_setsockopt(result_socket_, ZMQ_RECONNECT_IVL_MAX, &reconnect_ivl_max, sizeof(reconnect_ivl_max));

	char endpoint[256];

	snprintf(endpoint, sizeof(endpoint), "tcp://%s:%d", host.c_str(), frame_port);
	if (zmq_connect(frame_socket_, endpoint) != 0)
	{
		fprintf(stderr, "pi_nit: nao foi possivel conectar em %s: %s\n", endpoint, zmq_strerror(zmq_errno()));
		disconnect();
		return (false);
	}
	printf("pi_nit: socket de frames conectado em %s\n", endpoint);

	snprintf(endpoint, sizeof(endpoint), "tcp://%s:%d", host.c_str(), result_port);
	if (zmq_connect(result_socket_, endpoint) != 0)
	{
		fprintf(stderr, "pi_nit: nao foi possivel conectar em %s: %s\n", endpoint, zmq_strerror(zmq_errno()));
		disconnect();
		return (false);
	}
	printf("pi_nit: socket de resultados conectado em %s (client_id %u)\n", endpoint, client_id_);

	return (true);
}


void
PiNitZmqClient::disconnect()
{
	if (frame_socket_ != NULL)
	{
		zmq_close(frame_socket_);
		frame_socket_ = NULL;
	}
	if (result_socket_ != NULL)
	{
		zmq_close(result_socket_);
		result_socket_ = NULL;
	}
	if (context_ != NULL)
	{
		zmq_ctx_term(context_);
		context_ = NULL;
	}
}


bool
PiNitZmqClient::send_frame(const uint8_t *payload, size_t payload_len, int format,
		int width, int height, int camera_id, uint64_t frame_id, double timestamp)
{
	if ((frame_socket_ == NULL) || (payload == NULL) || (payload_len == 0))
		return (false);

	// Header e imagem viajam em uma unica parte: ZMQ_CONFLATE nao suporta
	// mensagens multi-part.
	send_buffer_.resize(sizeof(pi_nit_frame_header_t) + payload_len);

	pi_nit_frame_header_t *header = (pi_nit_frame_header_t *) &send_buffer_[0];
	memcpy(header->magic, PI_NIT_FRAME_MAGIC, 4);
	header->version = PI_NIT_PROTOCOL_VERSION;
	header->format = (uint16_t) format;
	header->width = (uint32_t) width;
	header->height = (uint32_t) height;
	header->client_id = client_id_;
	header->camera_id = (int32_t) camera_id;
	header->frame_id = frame_id;
	header->timestamp = timestamp;
	header->payload_len = (uint32_t) payload_len;
	header->reserved = 0;

	memcpy(&send_buffer_[sizeof(pi_nit_frame_header_t)], payload, payload_len);

	int sent = zmq_send(frame_socket_, &send_buffer_[0], send_buffer_.size(), ZMQ_DONTWAIT);

	return (sent == (int) send_buffer_.size());
}


bool
PiNitZmqClient::receive_result(pi_nit_result_header_t &header, std::vector<pi_nit_detection_t> &detections)
{
	detections.clear();

	if (result_socket_ == NULL)
		return (false);

	zmq_msg_t msg;
	zmq_msg_init(&msg);

	int received = zmq_msg_recv(&msg, result_socket_, ZMQ_DONTWAIT);
	if (received < 0)
	{
		zmq_msg_close(&msg);
		return (false);   // EAGAIN: nada pendente
	}

	bool ok = false;
	const uint8_t *data = (const uint8_t *) zmq_msg_data(&msg);
	size_t size = zmq_msg_size(&msg);

	if ((size >= sizeof(pi_nit_result_header_t)) && (memcmp(data, PI_NIT_RESULT_MAGIC, 4) == 0))
	{
		memcpy(&header, data, sizeof(pi_nit_result_header_t));

		size_t expected = sizeof(pi_nit_result_header_t) + header.num_detections * sizeof(pi_nit_detection_t);
		if ((header.version == PI_NIT_PROTOCOL_VERSION) && (size == expected))
		{
			detections.resize(header.num_detections);
			if (header.num_detections > 0)
				memcpy(&detections[0], data + sizeof(pi_nit_result_header_t),
						header.num_detections * sizeof(pi_nit_detection_t));
			ok = true;
		}
		else
		{
			fprintf(stderr, "pi_nit: resultado descartado (versao %u, %zu bytes, esperado %zu)\n",
					header.version, size, expected);
		}
	}
	else
	{
		fprintf(stderr, "pi_nit: resultado com magic invalido descartado (%zu bytes)\n", size);
	}

	zmq_msg_close(&msg);

	return (ok);
}
