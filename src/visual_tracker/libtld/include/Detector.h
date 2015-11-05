/*
 * Copyright � 2011 Paul Nader 
 *
 * This file is part of QOpenTLD.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <opencv/cv.h>
#include <map>
#include <list>
#include <vector>
#include "Rect.h"

class Lk;
class Fern;

typedef std::pair<cv::Mat_<int>* /*img*/, cv::Mat_<int>* /*blur*/ >ImagePairType;

typedef struct { int x; int y; } TLDPoint;

typedef enum {TLD_RED=0, TLD_BLUE=1, TLD_GREEN=2} TLDPointColor;

class Detector
{

public:
	Detector();
	~Detector();
	bool init(IplImage *frame, TLDRect *box);
	bool initialized(void) { return m_initialized; };
	void process(IplImage *frame);
	void display(int runtime, int frameNumber = 0);
	TLDRect getOutputRect(void) const;
	double getOutputConfidence(void) const;

public:
	void onPexCount(int count);
	void onNexCount(int count);
	void onUpdateRect(TLDRect rect);
	void onUpdateInfo(const char *infoString);
	void onUpdateConfidence(double conf);
	void onUpdatePoints(const std::vector<TLDPoint> &points, const TLDPointColor &color);

public:
	void signalBoundingBox(bool val);
	void signalPoints(bool val);
	void signalConfidence(bool val);
	void signalDetections(bool val);
	void signalPositive(bool val);
	void signalNegative(bool val);

protected:
	typedef enum {FORREST=0} FeatureType;

	typedef struct {
		unsigned num_closest;
		unsigned num_warps;
		unsigned noise;
		unsigned angle;
		double shift;
		double scale;
	} ParOptionsType;

	typedef struct {
		double overlap;
		unsigned num_patches;
	} NParOptionsType;

	typedef struct {
		cv::Mat_<double> bb;
		cv::Mat_<double> idx;
		cv::Mat_<double> conf1;
		cv::Mat_<double> conf2;
		cv::Mat_<double> isin;
		cv::Mat_<double> patt;
		cv::Mat_<double> patch;
	} DtType;

	void reset();
	void var(const cv::Mat_<double>& inMat, cv::Mat_<double>& outMat);
	template<class T> double median(std::vector<T> v);
	template<class T> double median2(std::vector<T> v);
	bool bb_scan(const cv::Mat_<double>& bb, const CvSize& imsize, double min_win, cv::Mat_<double>& outGrid, cv::Mat_<double>& outScales);
	void generateFeatures(unsigned nTrees, unsigned nFeatures, cv::Mat_<double>& features, bool show = false);
	double bb_overlap(double *bb1, double *bb2);
	void bb_overlap(const cv::Mat_<double>& bb, cv::Mat_<double>& overlap);
	void bb_overlap(const cv::Mat_<double>& bb1, const cv::Mat_<double>& bb2, cv::Mat_<double>& overlap);
	void imagePatch(const cv::Mat_<int>& inMat, const cv::Mat_<double>& bb, cv::Mat_<unsigned char>& outMat, const ParOptionsType& init, double randomise = -1);
	void warp(const cv::Mat_<int>& im, const cv::Mat_<double>& H, const cv::Mat_<double>& B, cv::Mat_<double>& outMat);
	double bbHeight(const cv::Mat_<double>& bb);
	double bbWidth(const cv::Mat_<double>& bb);
	void bbSize(const cv::Mat_<double>& bb, cv::Mat_<double>& s);
	void bbCenter(const cv::Mat_<double>& bb, cv::Mat_<double>& center);
	void tldGeneratePositiveData(const cv::Mat_<double>& overlap, const ImagePairType& img, const ParOptionsType& init, cv::Mat_<double>& pX, cv::Mat_<double>& pEx, cv::Mat_<double>& bbP0);
	void tldGenerateNegativeData(const cv::Mat_<double>& overlap, const ImagePairType& img, cv::Mat_<double>& nX, cv::Mat_<double>& nEx);
	void randValues(const cv::Mat_<double>& in, double k, cv::Mat_<double>& out);
	void tldSplitNegativeData(const cv::Mat_<double>& nX, const cv::Mat_<double>& nEx, cv::Mat_<double>& nX1, cv::Mat_<double>& nX2, cv::Mat_<double>& nEx1, cv::Mat_<double>& nEx2);
	void bbHull(const cv::Mat_<double>& bb, cv::Mat_<double>& hull);
	void getPattern(const ImagePairType& img, const cv::Mat_<double>& bb, const unsigned patchSize, cv::Mat_<double>& pattern, bool flip= false);
	void patch2Pattern(const cv::Mat_<unsigned char>& patch, const unsigned patchSize, cv::Mat_<double>& pattern);
	template<class T> bool randperm(int n, cv::Mat_<T>& outMat);
	void tldTrainNN(const cv::Mat_<double>& pEx, const cv::Mat_<double>& nEx);
	void tldNN(const cv::Mat_<double>& x, cv::Mat_<double>& conf1, cv::Mat_<double>& conf2, cv::Mat_<double>& isin);
	IplImage* blur(IplImage& image, double sigma);
	void blur(IplImage& image, cv::Mat_<int>& outMat, double sigma = 1.5);
	void emitImage(cv::Mat& mat);
	void emitPex(cv::Mat& mat);
	void emitNex(cv::Mat& mat);
	void emitStatus(const char *msg);
	IplImage* resizeImage(IplImage *origImg, int newWidth, int newHeight, bool keepAspectRatio = false);
	void distance(const cv::Mat_<double>& x1, const cv::Mat_<double>& x2, int flag, cv::Mat_<double>& resp);
	void embedPex(void);
	void embedNex(void);
	void mat2img(const cv::Mat_<double> data, int no_row, cv::Mat_<double>& imgMat);
	void ind2sub(const cv::Mat_<double>& siz, cv::Mat_<int>& subs, int idx);
	template<class T> void cumprod(const cv::Mat_<T>& inMat, cv::Mat_<T>& outMat);
	void bb_rescale_relative(cv::Mat_<double> BB, cv::Mat_<double> s, cv::Mat_<double>& bb);
	void tracking(const cv::Mat_<double>& bb, int I, int J, cv::Mat_<double>& BB2, double& tConf, double& tValid);
	void bbPoints(const cv::Mat_<double>& bb, double numM, double numN, double margin, cv::Mat_<double>& pt);
	template<class T> bool bbIsOut(const cv::Mat_<T> bb, const cv::Mat_<T>& imsize);
	template<class T> void isFinite(const cv::Mat_<T>& bb, cv::Mat_<int>& outMat);
	template<class T> bool bbIsDef(const cv::Mat_<T>& bb);
	void bbPredict(const cv::Mat_<double>& BB0, const cv::Mat_<double>& pt0, const cv::Mat_<double>& pt1, cv::Mat_<double>& BB2, cv::Mat_<double>& shift);
	void pdist(const cv::Mat_<double>& inMat, cv::Mat_<double>& outMat);
	void detection(int I, cv::Mat_<double>& BB,  cv::Mat_<double>& Conf);
	void bb_cluster_confidence(const cv::Mat_<double>& iBB, const cv::Mat_<double> iConf, cv::Mat_<double>& oBB, cv::Mat_<double>& oConf, cv::Mat_<double>& oSize);
	double bb_distance(const cv::Mat_<double>& bb1, const cv::Mat_<double>* bb2 = NULL);
	void learning(int I);

private:
	double shift;
	double min_bb;
	const unsigned m_patchSize;
	TLDRect m_bb_source;
	TLDRect m_bb_output;
	cv::Mat_<double> m_source_bb;
	double m_min_win;
	double m_model_thr_fern;
	double m_model_thr_nn;
	double m_num_trees;
	unsigned m_num_init;
	double m_model_valid;
	double m_model_thr_nn_valid;
	double m_ncc_thesame;
	unsigned m_model_patchsize;
	bool m_plot_pex;
	bool m_plot_nex;
	bool m_plot_dt;
	//int m_plot_patch_rescale;
	bool m_plot_confidence;
	//bool m_plot_target;
	//int m_plot_replace;
	int m_plot_drawoutput;
	bool m_plot_bb;
	bool m_plot_draw;
	bool m_plot_pts;
	int m_plot_patch_rescale;
	//int m_plot_save;

	int m_control_maxbbox;
	bool m_control_update_detector;
	bool m_control_drop_img;

	IplImage* m_image;
	cv::Mat_<double> m_scale;
	cv::Mat_<double> m_grid;
	cv::Mat_<double> m_scales;
	int m_nGrid;
	FeatureType m_featureType;
	cv::Mat_<double> m_features;
	unsigned m_nTrees;
	unsigned m_nFeatures;
	Lk* m_lk;
	Fern* m_fern;
	cv::Mat_<double> m_tmpConf;
	cv::Mat_<double> m_tmpPatt;

	std::vector<ImagePairType> m_img;
//	std::list<cv::Mat_<double>*> m_snapshot;
	std::map<int, DtType> m_dt;

	cv::Mat_<double> m_bb;
	cv::Mat_<double> m_conf;
	cv::Mat_<double> m_valid;
	cv::Mat_<double> m_size;
	std::list<int *> m_trackerFailure;
	std::vector<cv::Mat_<double> > m_draw;
	cv::Mat_<double> m_pts;

	cv::Mat_<int> m_imgSize;
	std::vector<cv::Mat_<double> > m_X;
	std::vector<cv::Mat_<double> > m_Y;
	std::vector<cv::Mat_<double> > m_pEx;
	std::vector<cv::Mat_<double> > m_nEx;
	double m_var;

	cv::Mat_<double> m_overlap;
	cv::Mat_<unsigned char> m_target;
	ParOptionsType m_p_par_init;
	ParOptionsType m_p_par_update;
	NParOptionsType m_n_par;
	bool m_fliplr;

	cv::Mat_<double> m_pex;
	cv::Mat_<double> m_nex;
	int m_pexCol;
	int m_nexCol;

	bool m_initialized;
	int m_frameNumber;
	double m_confidence;

	cv::Mat_<double> m_xFJ;
	cv::Mat_<float> m_gaussMat;

	cv::RNG* m_rng;
	uint64 m_rngState;
};

#endif /* DETECTOR_H_ */
