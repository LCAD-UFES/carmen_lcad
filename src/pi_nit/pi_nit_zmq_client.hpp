/*********************************************************
 Pi NIT - Cliente ZMQ (lado PC)

 Envia frames 640x640 para o Raspberry Pi e recebe as deteccoes.

 Sockets (o Raspberry faz bind, aqui apenas connect - assim o modulo do
 CARMEN pode ser reiniciado a vontade sem derrubar o servico do Pi):

   frame  : ZMQ_PUSH  -> tcp://<pi>:5560   SNDHWM = numero de cameras
   result : ZMQ_PULL  <- tcp://<pi>:5561   RCVHWM 4, leitura nao bloqueante

 IMPORTANTE - por que NAO usamos ZMQ_CONFLATE aqui:
 o CONFLATE guarda "a ultima mensagem" do socket inteiro, sem saber que as
 mensagens vem de cameras diferentes. Com 3 cameras compartilhando um socket
 ele descartaria 2 de cada 3 frames. Em vez disso limitamos a fila ao numero
 de cameras (SNDHWM) e enviamos com ZMQ_DONTWAIT: quando o Raspberry atrasa,
 o envio simplesmente falha e o frame e' descartado ali mesmo - mesmo efeito
 do CONFLATE, mas por camera. Quem descarta o frame velho de cada camera e'
 o servidor, que sabe ler o camera_id.

 A reconexao e' automatica (responsabilidade do proprio ZMQ): se o Pi cair
 e voltar, o envio volta a funcionar sozinho.
 *********************************************************/

#ifndef PI_NIT_ZMQ_CLIENT_HPP
#define PI_NIT_ZMQ_CLIENT_HPP

#include <stdint.h>
#include <string>
#include <vector>

#include "pi_nit_protocol.h"


class PiNitZmqClient
{
public:
	PiNitZmqClient();
	~PiNitZmqClient();

	// Cria contexto e sockets e conecta no Raspberry. Retorna false em erro.
	// queue_depth deve ser o numero de cameras: e' o maximo de frames que
	// podem estar em voo ao mesmo tempo (um por camera).
	bool connect(const std::string &host, int frame_port, int result_port, int queue_depth = 1);

	void disconnect();

	// Envia uma imagem ja no formato de entrada da rede (640x640).
	// payload -> bytes crus BGR ou buffer JPEG, conforme 'format'.
	// Nao bloqueia: se a fila de saida estiver cheia o frame e' descartado
	// (retorna false), que e' o comportamento desejado a 15 fps.
	bool send_frame(const uint8_t *payload, size_t payload_len, int format,
			int width, int height, int camera_id, uint64_t frame_id, double timestamp);

	// Le um resultado, se houver. Nao bloqueia.
	// Retorna false quando nao ha nada pendente ou a mensagem e' invalida.
	bool receive_result(pi_nit_result_header_t &header, std::vector<pi_nit_detection_t> &detections);

	uint32_t client_id() const { return client_id_; }

private:
	void    *context_;
	void    *frame_socket_;
	void    *result_socket_;
	uint32_t client_id_;

	std::vector<uint8_t> send_buffer_;   // header + payload, reutilizado a cada frame
};

#endif
