/*********************************************************
 Pi NIT - Rastreador simples por IoU

 O Hailo devolve apenas caixas por frame. O campo track_id da
 neural_detector_message e' preenchido aqui, no PC, por associacao guloso
 de IoU entre o frame atual e os tracks vivos (mesma ideia do SORT, sem o
 filtro de Kalman - suficiente para pessoas a 15 fps).

 track_id 0 e' reservado para "nao rastreado" (convencao do
 neural_detector_messages.h), entao os ids comecam em 1.
 *********************************************************/

#ifndef PI_NIT_TRACKER_HPP
#define PI_NIT_TRACKER_HPP

#include <vector>

#include "pi_nit_protocol.h"


class PiNitTracker
{
public:
	PiNitTracker(double iou_threshold = 0.3, int max_age_frames = 8);

	// Recebe as deteccoes do frame (em coordenadas da imagem original) e
	// devolve, na mesma ordem, o track_id de cada uma.
	std::vector<int> update(const std::vector<pi_nit_detection_t> &detections);

	void reset();

private:
	struct Track
	{
		int   id;
		int   class_id;
		float x1, y1, x2, y2;
		int   age;          // frames desde a ultima associacao
	};

	static double iou(const Track &track, const pi_nit_detection_t &detection);

	double             iou_threshold_;
	int                max_age_frames_;
	int                next_id_;
	std::vector<Track> tracks_;
};

#endif
