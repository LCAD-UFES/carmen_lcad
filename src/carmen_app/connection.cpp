#include "connection.h"

#include <QJsonDocument>
#include <zmq.h>
#include <chrono>

static double
steady_seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

Connection::Connection(const std::string &host, int pub_port, int cmd_port) : host(host)
{
	ctx = zmq_ctx_new();
	sub = zmq_socket(ctx, ZMQ_SUB);
	int hwm = 50, linger = 0;
	zmq_setsockopt(sub, ZMQ_RCVHWM, &hwm, sizeof(hwm));
	zmq_setsockopt(sub, ZMQ_LINGER, &linger, sizeof(linger));
	zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
	zmq_connect(sub, ("tcp://" + host + ":" + std::to_string(pub_port)).c_str());
	cmd_endpoint = "tcp://" + host + ":" + std::to_string(cmd_port);
	new_req();
}

Connection::~Connection()
{
	zmq_close(sub);
	if (req)
		zmq_close(req);
	zmq_ctx_term(ctx);
}

void
Connection::new_req()
{
	if (req)
		zmq_close(req);
	req = zmq_socket(ctx, ZMQ_REQ);
	/* o tempo de espera e' por pedido (zmq_poll em request()) */
	int linger = 0, one = 1;
	zmq_setsockopt(req, ZMQ_LINGER, &linger, sizeof(linger));
	zmq_setsockopt(req, ZMQ_REQ_RELAXED, &one, sizeof(one));
	zmq_setsockopt(req, ZMQ_REQ_CORRELATE, &one, sizeof(one));
	zmq_connect(req, cmd_endpoint.c_str());
}

bool
Connection::recv_all(void *socket, int flags, std::vector<QByteArray> *frames)
{
	int more = 1;
	size_t more_size = sizeof(more);
	bool first = true;
	while (more)
	{
		zmq_msg_t m;
		zmq_msg_init(&m);
		if (zmq_msg_recv(&m, socket, first ? flags : 0) < 0)
		{
			zmq_msg_close(&m);
			return !first;
		}
		frames->push_back(QByteArray((const char *) zmq_msg_data(&m), (int) zmq_msg_size(&m)));
		zmq_msg_close(&m);
		zmq_getsockopt(socket, ZMQ_RCVMORE, &more, &more_size);
		first = false;
	}
	return true;
}

bool
Connection::link_alive() const
{
	return last_rx > 0.0 && steady_seconds() - last_rx < 3.0;
}

bool
Connection::request(const QJsonObject &payload, QJsonObject *reply, QByteArray *blob, int timeout_ms)
{
	if (!link_alive())
		return false;	// sem robo: nem tenta (cada tentativa travaria a janela pelo timeout inteiro)
	if (timeout_ms < 0)
	{
		QString cmd = payload["cmd"].toString();
		bool heavy = cmd == "snapshot" || cmd == "full_map" || cmd == "get_map" || cmd == "car_model" || cmd == "missions";
		timeout_ms = heavy ? 8000 : 2500;
	}
	QByteArray out = QJsonDocument(payload).toJson(QJsonDocument::Compact);
	if (zmq_send(req, out.constData(), out.size(), ZMQ_DONTWAIT) < 0)
	{
		new_req();
		return false;
	}
	zmq_pollitem_t item = {req, 0, ZMQ_POLLIN, 0};
	std::vector<QByteArray> frames;
	if (zmq_poll(&item, 1, timeout_ms) <= 0 || !(item.revents & ZMQ_POLLIN) || !recv_all(req, ZMQ_DONTWAIT, &frames))
	{
		new_req();
		return false;
	}
	*reply = QJsonDocument::fromJson(frames[0]).object();
	if (blob)
		*blob = (frames.size() > 1) ? frames[1] : QByteArray();
	return true;
}

std::vector<Frames>
Connection::poll(int max_msgs)
{
	std::vector<Frames> msgs;
	for (int i = 0; i < max_msgs; i++)
	{
		std::vector<QByteArray> frames;
		if (!recv_all(sub, ZMQ_DONTWAIT, &frames))
			break;
		last_rx = steady_seconds();
		if (frames.size() < 2)
			continue;
		Frames f;
		f.topic = frames[0].toStdString();
		f.json = QJsonDocument::fromJson(frames[1]).object();
		if (frames.size() > 2)
			f.blob = frames[2];
		msgs.push_back(f);
	}
	return msgs;
}
