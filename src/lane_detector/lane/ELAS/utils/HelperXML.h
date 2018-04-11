#ifndef __HELPER_XML_H
#define __HELPER_XML_H

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "../libs/tinyxml2/tinyxml2.h"
#include "IPM.h"
#include "common.h"
#include "Helper.h"

using namespace std;
using namespace cv;

// Carrega o XML do dataset
bool loadDatasetConfig(const string &filepath, ConfigXML &outConfigXML);

// load general config
bool loadConfig(const string &fname, ConfigXML &_config);

// Pega o IPM
void setIPM(ConfigXML &_config);

// Monta a ROI
void setROI(ConfigXML &_config);

// Pega uma IPM a partir de um Vanishing Point
IPM * AtualizaIPMLivre(ConfigXML &_config, Point2f vp, bool verbose = false);

// Pega uma IPM a partir de um Vanishing Point
IPM * AtualizaIPMRestricaoGaussiana2D(ConfigXML &_config, Point2f vp, bool verbose);

// Fun��o gaussiana com m�ximo em v = 0, com valor igual a 1
inline double getGaussianValue(double v, double sigma) { return exp(-(v*v) / (2.0 * sigma * sigma)); };

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
// from: http://stackoverflow.com/a/7448287/4228275
bool lineIntersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r);

struct OutputXML {
	// cabe�alho
	string id;
	Size frameSize;
	int nFrames;
	vector<int> frameNumber;
	// frame data
	vector<double> laneCenter;
	struct { vector<vector<double> > left, right; } position;
	struct { vector<int> left, right; } multipleLanes;
	struct { vector<LMT> left, right; } lmt;
	vector<int> laneChange;
	vector<vector<int> > roadSigns;
	vector<double> time;
	map<int, vector<double> > all_times;
};

// inicializa um XML de sa�da
OutputXML createOutputXML(string _id, Size _frameSize, int _nFrames);

// adiciona um <frame> no XML iniciado
bool appendFrameData(OutputXML *xml, double laneCenter, vector<double> posLeft, vector<double> posRight, int multipleLeft, int multipleRight, LMT lmtLeft, LMT lmtRight, int laneChange, vector<int> &roadSigns, double time);

// salva o xml no local indicado
bool saveOutputXML(OutputXML *xml, string path);

#endif // __HELPER_XML_H
