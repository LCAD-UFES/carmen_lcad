/*
 * Conexao com o carmen_server: SUB (dados) + REQ (comandos). Protocolo em src/carmen_server/PROTOCOLO.md.
 */
#ifndef CARMEN_APP_CONNECTION_H
#define CARMEN_APP_CONNECTION_H

#include <QByteArray>
#include <QJsonObject>
#include <string>
#include <vector>

struct Frames
{
	std::string topic;
	QJsonObject json;
	QByteArray blob;
};

class Connection
{
public:
	Connection(const std::string &host, int pub_port, int cmd_port);
	~Connection();

	/* Manda um comando; false se o robo nao respondeu (o socket REQ e' recriado).
	   Sem dados do robo no SUB ha' mais de 3 s, volta false NA HORA (nao trava a janela esperando).
	   timeout_ms < 0: 2,5 s, ou 8 s nos comandos pesados (snapshot, mapas, modelo do carro). */
	bool request(const QJsonObject &payload, QJsonObject *reply, QByteArray *blob = nullptr, int timeout_ms = -1);

	/* chegou alguma coisa do robo (hello sai 1x por segundo) nos ultimos 3 s? */
	bool link_alive() const;

	/* Tudo que chegou no SUB desde a ultima chamada (sem bloquear). */
	std::vector<Frames> poll(int max_msgs = 300);

	std::string host;

private:
	void new_req();
	static bool recv_all(void *socket, int flags, std::vector<QByteArray> *frames);

	void *ctx = nullptr, *sub = nullptr, *req = nullptr;
	double last_rx = 0.0;
	std::string cmd_endpoint;
};

#endif
