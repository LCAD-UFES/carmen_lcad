#ifndef __COMMON_H
#define __COMMON_H

// general includes
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

// specific includes
#include "IPM.h"

// general defines
#define CONFIG_DISPLAY false // controls image visualization (meant for debugging only)
#define CONFIG_VERBOSE false // controls console outputs

// temporary
// #define DATASETS_DIR = "/dados/berriel/MEGA/datasets/";
// #define DATA_DIR = "/dados/berriel/MEGA/projects/lane-research/data/";

#define BGR_BLUE cv::Scalar(255, 0, 0)
#define BGR_GREEN cv::Scalar(0, 255, 0)
#define BGR_RED cv::Scalar(0, 0, 255)

// Estrutura para armazenar os dados do XML
struct ConfigXML {

	bool verbose;
	bool display;

	// dataset info
	struct {
		std::string id;
		std::string name;
		std::string path;
		struct { int start, end; } FrameSequence;
		struct { int width, height; } FrameSize;
		struct { int x, y, width, height; } _ROI;
		struct { int tr, tl, bl, br; cv::Point2f vp; } _IPM;

	} dataset;

	IPM *ipm;
	cv::Rect roi;
	int numParticles;
	cv::Point carPosition, carPositionIPM;

	// paths
	std::string DATASETS_DIR, DATA_DIR;

};

extern ConfigXML * config;

// env var MEGA_DIR needs to bet set
/*
const std::tr2::sys::path MEGA_DIR = getenv("MEGA_DIR");
const std::tr2::sys::path DATASETS_DIR = MEGA_DIR / std::tr2::sys::path("datasets");
*/

// road signs symbols
struct viz_symbols { std::vector<cv::Point> region; int id; };

enum {
	LANE_LEFT = 0,
	LANE_RIGHT = 1,
	LANE_CENTER = 9
};

// Lane Markings Types
enum LMT {
	NONE = 0,
	SCB = 1, // LMS-1	-> [S]imples	[C]ont�nua		[B]ranca
	SSB = 2, // LMS-2	-> [S]imples	[S]eccionada	[B]ranca
	SCA = 3, // LFO-1	-> [S]imples	[C]ont�nua		[A]marela
	SSA = 4, // LFO-2	-> [S]imples	[S]eccionada	[A]marela
	DCA = 5, // LFO-3	-> [D]upla		[C]ont�nua		[A]marela
	DSC = 6, // LFO-4b	-> [D]upla		[S]eccionada	[C]ontinua		(esquerda [S] | [C] direita) Amarela
	DCS = 7 // LFO-4a	-> [D]upla		[C]ontinua		[S]eccionada	(esquerda [C] | [S] direita) Amarela
};

enum LMT_COLOR {
	WHITE = 0,
	YELLOW = 1
};

namespace ROAD_SIGN {
	enum {
		NONE = 0,
		FRENTE = 1,
		FRENTE_CURTA = 2,
		FRENTE_DIREITA = 3,
		FRENTE_ESQUERDA = 4,
		DIREITA = 5,
		ESQUERDA = 6,
		VOLTA_ESQUERDA = 7,
		VOLTA_DIREITA = 8,
		FAIXA_PEDESTRE = 9,
		BARRA_CONTENCAO = 10
	};
}

#endif
