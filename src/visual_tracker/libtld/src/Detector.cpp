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

#include "Utils.hpp"
#include "Detector.h"
#include "Lk.h"
#include "Fern.h"
#include <vector>
#include <limits>

const int MAX_FRAMES = 10000;

// Options
#define USE_CVSMOOTH_BLUR

Detector::Detector()
: shift(0.1)
, min_bb(12.0)
, m_patchSize(15)
, m_min_win(min_bb)
, m_model_thr_fern(0.5)
, m_model_thr_nn(0.65)
, m_num_trees(10)
, m_model_valid(0.5)
, m_model_thr_nn_valid(0.7)
, m_ncc_thesame(0.95)
, m_model_patchsize(m_patchSize)
, m_plot_pex(false)
, m_plot_nex(false)
, m_plot_dt(false)
, m_plot_confidence(false)
//, m_plot_target(true)
, m_plot_drawoutput(3)
, m_plot_bb(false)
, m_plot_draw(false)
, m_plot_pts(false)
, m_plot_patch_rescale(1)
, m_control_maxbbox(1)
, m_control_update_detector(true)
, m_control_drop_img(true)
, m_image(NULL)
, m_nGrid(0)
, m_nTrees(10)
, m_nFeatures(13)
, m_var(0)
, m_fliplr(true)
, m_pexCol(0)
, m_nexCol(0)
, m_initialized(false)
, m_frameNumber(0)
{
	LOG("Detector Ctr");

	m_rng = new cv::RNG(0);
	m_rngState = m_rng->state;

	m_source_bb.create(4, 1);
	m_scale.create(1, 21);
	for(int i=0; i < m_scale.cols; i++)
		m_scale.at<double>(0,i) = pow(1.2, i-10);

	m_p_par_init.num_closest = 10;
	m_p_par_init.num_warps = 20;
	m_p_par_init.noise = 5;
	m_p_par_init.angle = 20;
	m_p_par_init.shift = 0.02;
	m_p_par_init.scale = 0.02;

	m_p_par_update.num_closest = 10;
	m_p_par_update.num_warps = 10;
	m_p_par_update.noise = 5;
	m_p_par_update.angle = 10;
	m_p_par_update.shift = 0.02;
	m_p_par_update.scale = 0.02;

	m_n_par.overlap = 0.2;
	m_n_par.num_patches = 100;

	m_fern = new Fern();
	m_lk = new Lk();
}

Detector::~Detector()
{
	LOG("Detector Dtr");
	reset();
	delete m_lk;
	delete m_fern;
}

void Detector::reset()
{
	LOG("Detector::reset");
	m_rng->state = m_rngState;
	m_gaussMat.create(0,0);
	//m_source_bb.create(0,0);
	//m_scale.create(0,0);
	m_grid.create(0,0);
	m_scales.create(0,0);
	m_features.create(0,0);
	m_tmpConf.create(0,0);
	m_tmpPatt.create(0,0);
	//m_bb.create(0,0);
	m_draw.clear();
	m_pts.create(0,0);
	m_imgSize.create(0,0);
	m_overlap.create(0,0);
	m_target.create(0,0);
	m_pex.create(0,0);
	m_nex.create(0,0);
	m_xFJ.create(0,0);
	m_bb.create(0,0);
	m_conf.create(0,0);
	m_valid.create(0,0);
	m_size.create(0,0);

	if (m_image) cvReleaseImage(&m_image);

	std::vector<ImagePairType>::iterator iptlit;
	for (iptlit = m_img.begin(); iptlit != m_img.end(); ++iptlit)
	{
		ImagePairType& ipt = *iptlit;
		delete ipt.first;
		delete ipt.second;
	}
	m_img.clear();

//	std::list<cv::Mat_<double>*>::iterator dmlit;
//	for (dmlit = m_snapshot.begin(); dmlit != m_snapshot.end(); ++dmlit)
//	{
//		cv::Mat_<double>* dm = *dmlit;
//		delete dm;
//	}
//	m_snapshot.clear();

	m_dt.clear();

	//std::vector<DtType>::iterator dtvit;
	//for (dtvit = m_dt.begin(); dtvit != m_dt.end(); ++dtvit)
	//{
	//	DtType& dm = *dtvit;
	//	delete dm;
	//}
	m_trackerFailure.clear();

	m_X.clear();
	m_Y.clear();
	m_pEx.clear();
	m_nEx.clear();

	m_fern->reset();
}

void Detector::emitImage(cv::Mat& mat)
{
	mat = mat;
	LOG("Detector::emitImage");
}

void Detector::emitPex(cv::Mat& mat)
{
	mat = mat;
	LOG("Detector::emitPex");
}

void Detector::emitNex(cv::Mat& mat)
{
	mat = mat;
	LOG("Detector::emitPex");
}

void Detector::emitStatus(const char *msg)
{
	LOG(msg);
}

bool Detector::init(IplImage *frame, TLDRect *box)
{
	LOG("Detector::init");

	reset();

	if (!frame) return false;
	if (!box) return false;

	IplImage& image = *frame;
	TLDRect& region = *box;

	TLDRect regionRect(0, 0, image.width, image.height);
	if (!region.intersects(regionRect))
	{
		emitStatus("Region selected is outside frame bounds");
		return false;
	}

	m_bb_source = region.intersection(regionRect);
	if ((m_bb_source.width() < min_bb) || (m_bb_source.height() < min_bb))
	{
		emitStatus("Region selected is too small");
		return false;
	}

	m_bb = cv::Mat(4, MAX_FRAMES, CV_64F, nan);
	m_conf = cv::Mat(1, MAX_FRAMES, CV_64F, nan);
	m_valid = cv::Mat(1, MAX_FRAMES, CV_64F, nan);
	m_size = cv::Mat(1, MAX_FRAMES, CV_64F, nan);

	// lk(0)
	m_lk->init(); // Initialise LK tracker

	m_source_bb(0,0) = m_bb_source.x();
	m_source_bb(1,0) = m_bb_source.y();
	m_source_bb(2,0) = m_bb_source.x() + m_bb_source.width();
	m_source_bb(3,0) = m_bb_source.y() + m_bb_source.height();
	//printMatrix(m_source_bb, "m_source_bb");

	cv::Mat mono(&image);
	cv::cvtColor(mono, mono, CV_BGR2GRAY);
	IplImage monoImage = mono;
	CvSize imageSize = cvSize(monoImage.width, monoImage.height);
	m_image = cvCreateImage(imageSize, monoImage.depth, monoImage.nChannels);
	cvCopy(&monoImage, m_image);
	cv::Mat_<int> imageMat(m_image);
	//printMatrix(imageMat, "imageMat");

	//IplImage* m_imageBlur = blur(*m_image, 2);
	//cv::Mat_<int> blurMat(m_imageBlur);
	//cvReleaseImage(&m_imageBlur);
	cv::Mat_<int> blurMat;
	blur(*m_image, blurMat, 2);
	//printMatrix(blurMat, "blurMat");

	//loadMatrix(imageMat, "image.txt");
	//loadMatrix(blurMat, "blur.txt");

	// scanning grid
	if (!bb_scan(m_source_bb, imageSize, m_min_win, m_grid, m_scales))
		return false;
	//cv::Mat_<double> gridT = m_grid.t();
	//printMatrix(gridT, "m_gridT");
	//printMatrix(m_scales, "m_scales");

	// features
	m_nGrid = m_grid.cols;
	generateFeatures(m_nTrees, m_nFeatures, m_features);
	//loadMatrix(m_features, "features.txt");
	//printMatrix(m_features, "m_features");

	// initialise detector
	if (!m_fern->init(*m_image, m_grid, m_features, m_scales))
		return false;

	// temp sctructures
	// tld.tmp.conf = zeros(1,tld.nGrid);
	// tld.tmp.patt = zeros(tld.model.num_trees,tld.nGrid);
	m_tmpConf = cv::Mat_<double>::zeros(1, m_nGrid);
	m_tmpPatt = cv::Mat_<double>::zeros(m_nTrees, m_nGrid);

	//% Initialize Trajectory
	//tld.img     = cell(1,length(tld.source.idx));
	//tld.snapshot= cell(1,length(tld.source.idx));
	//tld.dt      = cell(1,length(tld.source.idx));
	//tld.bb      = nan(4,length(tld.source.idx));
	//tld.conf    = nan(1,length(tld.source.idx));
	//tld.valid   = nan(1,length(tld.source.idx));
	//tld.size    = nan(1,length(tld.source.idx));
	//tld.trackerfailure = nan(1,length(tld.source.idx));
	//tld.draw    = zeros(2,0);
	//tld.pts     = zeros(2,0);
	//% Fill first fields
	//tld.img{1}  = tld.source.im0;
	//tld.bb(:,1) = tld.source.bb;
	//tld.conf(1) = 1;
	//tld.valid(1)= 1;
	//tld.size(1) = 1;
	//m_draw.push_back(cv::Mat_<double>::zeros(2, 1));
	m_pts = cv::Mat_<double>::zeros(2, 0);
	ImagePairType img;
	//loadMatrix(imageMat, "image.txt");
	//loadMatrix(blurMat, "blur.txt");
	img.first = new cv::Mat_<int>(imageMat);
	img.second = new cv::Mat_<int>(blurMat);
	m_img.push_back(img);
	m_bb(0,0) = m_bb_source.x();
	m_bb(1,0) = m_bb_source.y();
	m_bb(2,0) = m_bb_source.x() + m_bb_source.width();
	m_bb(3,0) = m_bb_source.y() + m_bb_source.height();
	//printMatrix(m_bb, "m_bb");
	m_conf(0,0) = 1;
	m_valid(0,0) = 1;
	m_size(0,0) = 1;

	// Train Detector
	//tld.imgsize = size(tld.source.im0.input);
	//tld.X       = cell(1,length(tld.source.idx)); % training data for fern
	//tld.Y       = cell(1,length(tld.source.idx)); 
	//tld.pEx     = cell(1,length(tld.source.idx)); % training data for NN
	//tld.nEx     = cell(1,length(tld.source.idx));
	//overlap     = bb_overlap(tld.source.bb,tld.grid); % bottleneck
	m_imgSize.create(1, 2);
	m_imgSize(0,0) = m_image->height;
	m_imgSize(0,1) = m_image->width;
	//printMatrix(m_imgSize, "m_imgSize");

	bb_overlap(m_source_bb, m_grid, m_overlap);
	//printMatrix(m_overlap, "m_overlap");

	imagePatch(imageMat, m_source_bb, m_target, m_p_par_init);
	//printMatrix(m_target, "m_target");

	//% Generate Positive Examples
	//[pX,pEx,bbP] = tldGeneratePositiveData(tld,overlap,tld.img{1},tld.p_par_init);
	cv::Mat_<double> pX;
	cv::Mat_<double> pEx;
	cv::Mat_<double> bbP;
	tldGeneratePositiveData(m_overlap, m_img.at(0), m_p_par_init, pX, pEx, bbP);
	//loadMatrix(pX, "pX.txt");
	//loadMatrix(pEx, "pEx.txt");
	//loadMatrix(bbP, "bbP.txt");
	//printMatrix(pX, "pX");
	//printMatrix(pEx, "pEx");
	//printMatrix(bbP, "bbP");

	// pY = ones(1,size(pX,2));
	cv::Mat_<double> pY(1, pX.cols);
	pY = cv::Mat_<double>::ones(1, pX.cols);

	//% Correct initial bbox
	//tld.bb(:,1) = bbP(1:4,:);
	//printMatrix(m_bb, "m_bb");
	Mat col = m_bb.col(0);
	bbP.col(0).copyTo(col);
	//printMatrix(m_bb, "m_bb");
	
	//% Variance threshold
	//tld.var = var(pEx(:,1))/2;
	cv::Mat_<double> varMat;
	var(pEx.col(0), varMat);
	//printMatrix(pEx, "pEx");
	m_var = varMat(0,0)/2;
	//printMatrix(varMat, "varMat");

	//% Generate Negative Examples
	//[nX,nEx] = tldGenerateNegativeData(tld,overlap,tld.img{1});
	cv::Mat_<double> nX;
	cv::Mat_<double> nEx;
	tldGenerateNegativeData(m_overlap, m_img.at(0), nX, nEx);
	//loadMatrix(nX, "nX.txt");
	//loadMatrix(nEx, "nEx.txt");
	//printMatrix(nX, "nX");
	//printMatrix(nEx, "nEx");

	//% Split Negative Data to Training set and Validation set
	//[nX1,nX2,nEx1,nEx2] = tldSplitNegativeData(nX,nEx);
	//nY1  = zeros(1,size(nX1,2));
	cv::Mat_<double> nX1;
	cv::Mat_<double> nX2;
	cv::Mat_<double> nEx1;
	cv::Mat_<double> nEx2;
	tldSplitNegativeData(nX, nEx, nX1, nX2, nEx1, nEx2);
	//loadMatrix(nX1, "nX1.txt");
	//loadMatrix(nX2, "nX2.txt");
	//loadMatrix(nEx1, "nEx1.txt");
	//loadMatrix(nEx2, "nEx2.txt");
	//printMatrix(nX1, "nX1");
	//printMatrix(nX2, "nX2");
	//printMatrix(nEx1, "nEx1");
	//printMatrix(nEx2, "nEx2");
	cv::Mat_<double> nY1 = cv::Mat_<double>::zeros(1, nX1.cols);

	//tld.pEx{1} = pEx; % save positive patches for later
	m_pEx.push_back(pEx);

	//tld.nEx{1} = nEx; % save negative patches for later
	m_nEx.push_back(nEx);

	//tld.X{1} = [pX nX1];
	cv::Mat_<double> X(pX.rows, pX.cols + nX1.cols);
	for (int c=0; c < pX.cols; ++c)
	{
		Mat col = X.col(c);
		pX.col(c).copyTo(col);
	}
	for (int c=0; c < nX1.cols; ++c)
	{
		Mat col = X.col(c+pX.cols);
		nX1.col(c).copyTo(col);
	}
	m_X.push_back(X);
	//printMatrix(X, "X");

	//tld.Y{1}    = [pY nY1];
	cv::Mat_<double> Y(pY.rows, pY.cols + nY1.cols);
	for (int c=0; c < pY.cols; ++c)
	{
		Mat col = Y.col(c);
		pY.col(c).copyTo(col);
	}
	for (int c=0; c < nY1.cols; ++c)
	{
		Mat col = Y.col(c+pY.cols);
		nY1.col(c).copyTo(col);
	}
	m_Y.push_back(Y);
	//printMatrix(Y, "Y");

	//idx = randperm(size(tld.X{1},2));
	cv::Mat_<double> idx;
	randperm<double>((int) m_X.at(0).cols, idx);
	//printMatrix(idx, "idx");

	//tld.X{1}    = tld.X{1}(:,idx);
	cv::Mat_<double> tmp(X.rows, idx.cols);
	for (int c=0; c < idx.cols; ++c)
	{
		Mat col = tmp.col(c);
		X.col(idx(0, c)).copyTo(col);
	}
	tmp.copyTo(X);
	//printMatrix(X, "X");

	//tld.Y{1}    = tld.Y{1}(:,idx);
	tmp.create(Y.rows, idx.cols);
	for (int c=0; c < idx.cols; ++c)
	{
		Mat col = tmp.col(c);
		Y.col(idx(0, c)).copyTo(col);
	}
	tmp.copyTo(Y);
	//printMatrix(Y, "Y");

	//% Train using training set ------------------------------------------------

	//% Fern
	//
	//bootstrap = 2;
	//fern(2,tld.X{1},tld.Y{1},tld.model.thr_fern,bootstrap);
	//
	int bootstrap = 2;
	//loadMatrix(X, "X.txt");
	//loadMatrix(Y, "Y.txt");
	//printMatrix(X, "X");
	//printMatrix(Y, "Y");
	m_fern->update(X, Y, m_model_thr_fern, bootstrap);

	//% Nearest Neighbour 
	//
	//tld.pex = [];
	//tld.nex = [];
	//
	//tld = tldTrainNN(pEx,nEx1,tld);
	//tld.model.num_init = size(tld.pex,2);
	tldTrainNN(pEx, nEx1);
	//printMatrix(pEx, "pEx");
	//printMatrix(nEx1, "nEx1");

	// tld.model.num_init = size(tld.pex,2);
	m_num_init = m_pex.cols;

	// % Estimate thresholds on validation set  ----------------------------------
	//
	// % Fern
	// conf_fern = fern(3,nX2);
	// tld.model.thr_fern = max(max(conf_fern)/tld.model.num_trees,tld.model.thr_fern);
	cv::Mat_<double> conf_fern;
	//loadMatrix(nX2, "nX2.txt");
	m_fern->evaluate(nX2, conf_fern);
	//printMatrix(nX2, "nX2");
	//printMatrix(conf_fern, "conf_fern");
	cv::Mat_<double>::iterator pos = std::max_element(conf_fern.begin(), conf_fern.end());
	double conf_fern_max = *pos;
	m_model_thr_fern = max(conf_fern_max/m_num_trees, m_model_thr_fern);

	// % Nearest neighbor
	// conf_nn = tldNN(nEx2,tld);
	cv::Mat_<double> conf_nn;
	cv::Mat_<double> dummy;
	cv::Mat_<double> isin;
	//loadMatrix(nEx2, "nEx2.txt");
	//loadMatrix(m_pex, "m_pex.txt");
	//loadMatrix(m_nex, "m_nex.txt");
	//printMatrix(nEx2, "nEx2");
	tldNN(nEx2, conf_nn, dummy, isin);
	//printMatrix(conf_nn, "conf_nn");

	// tld.model.thr_nn = max(tld.model.thr_nn,max(conf_nn));
	int index;
	double max_conf_nn;
	maxMat<double>(conf_nn, index, max_conf_nn);
	m_model_thr_nn = max(m_model_thr_nn, max_conf_nn); 

	// tld.model.thr_nn_valid = max(tld.model.thr_nn_valid,tld.model.thr_nn);
	m_model_thr_nn_valid = max(m_model_thr_nn_valid, m_model_thr_nn);

	m_initialized = true;
	return true;
}

void Detector::var(const cv::Mat_<double>& inMat, cv::Mat_<double>& outMat)
{
	LOG("Detector::var");
	outMat.create(1, inMat.cols);

	for (int c=0; c < inMat.cols; ++c)
	{
		double val = 0;
		double sum = 0;
		double mean = 0;
		double sum_squared = 0;
		double variance = 0;
		if (inMat.rows > 1)
		{
			for (int r=0; r < inMat.rows; ++r)
			{
				val = inMat(r,c);
				sum = sum + val;
				sum_squared = sum_squared + (val * val);
			}
			mean = sum / inMat.rows;
			variance = (sum_squared - (sum * mean)) / (inMat.rows - 1);
		}
		outMat(0, c) = variance;
	}
}

template<class T>
double Detector::median(std::vector<T> v)
{
	LOG("Detector::median");
	if (v.empty()) return nan;

	double med;
	size_t size = v.size();
	std::sort(v.begin(), v.end());
	if (size  % 2 == 0)
		med = (v[size / 2 - 1] + v[size / 2]) / 2;
	else 
		med = v[size / 2];
	return med;
}

template<class T>
double Detector::median2(std::vector<T> v)
{
	LOG("Detector::median2");
	double med;
	std::vector<T> v1;
	typename std::vector<T>::iterator it;
	for (it=v.begin(); it != v.end(); ++it)
	{
		if (!isnan(*it))
			v1.push_back(*it);
	}
	if (!v1.empty()) med = median<T>(v1);
	else med = nan;
	return med;
}

bool Detector::bb_scan(const cv::Mat_<double>& bb, const CvSize& imSize, double min_win, Mat_<double>& bb_out, Mat_<double>& sca)
{
	LOG("Detector::bb_scan");
	//SHIFT = 0.1;
	//SCALE = 1.2.^[-10:10];
	//MINBB = min_win;

	const double minbb = min_win;
	if ((imSize.width < min_win) || (imSize.height < min_win))
		return false;

	static const double shift = 0.1;
	cv::Mat_<double> ident = cv::Mat_<double>::ones(m_scale.rows, m_scale.cols);

	//bbW   = round(bb_width(bb) .* SCALE);
	cv::Mat_<double> bbW(m_scale.rows, m_scale.cols);
	cv::multiply(m_scale, ident, bbW, bbWidth(bb));
	for (int r=0; r < bbW.rows; ++r)
		for (int c=0; c < bbW.cols; ++c)
			bbW(r, c) = cvRound(bbW(r, c));
	//printMatrix(bbW);
	
	//bbH   = round(bb_height(bb) .* SCALE);
	cv::Mat_<double> bbH(m_scale.rows, m_scale.cols);
	cv::multiply(m_scale, ident, bbH, bbHeight(bb));
	for (int r=0; r < bbH.rows; ++r)
		for (int c=0; c < bbH.cols; ++c)
			bbH(r, c) = cvRound(bbH(r, c));
	//printMatrix(bbH);

	//bbSHH = SHIFT * min(bbH,bbH);
	cv::Mat_<double> bbSHH(m_scale.rows, m_scale.cols);
	for (int r=0; r < bbSHH.rows; ++r)
		for (int c=0; c < bbSHH.cols; ++c)
			bbSHH(r, c) = shift * bbH(r, c);
	//printMatrix(bbSHH);

	//bbSHW = SHIFT * min(bbH,bbW);
	cv::Mat_<double> bbSHW(m_scale.rows, m_scale.cols);
	for (int r=0; r < bbSHW.rows; ++r)
		for (int c=0; c < bbSHW.cols; ++c)
			bbSHW(r, c) = shift * ((bbH(r, c) < bbW(r, c)) ? bbH(r, c) : bbW(r, c));
	//printMatrix(bbSHW);

	//bbF = [2 2 imsize(2) imsize(1)]';
	Mat_<double> bbF(4,1);
	bbF(0,0) = 2;
	bbF(1,0) = 2;
	bbF(2,0) = imSize.width;
	bbF(3,0) = imSize.height;
	//printMatrix(bbF);

	int idx = 1;
	vector<cv::Mat_<double> > bbs;
	for (int i=0; i < m_scale.cols; ++i)
	{
		// if bbW(i) < MINBB || bbH(i) < MINBB, continue; end
		if ((bbW(i) < minbb) || (bbH(i) < minbb))
			continue;

		// left  = round(bbF(1):bbSHW(i):bbF(3)-bbW(i)-1);
		vector<double> leftV;
		colon(bbF(0,0), bbSHW(0, i), bbF(2, 0)-bbW(0, i)-1, leftV);
		cv::Mat_<double> left(1, leftV.size());
		for (unsigned j=0; j < leftV.size(); ++j)
			left(0,j) = cvRound(leftV.at(j));
		//printMatrix(left);

		// top = round(bbF(2):bbSHH(i):bbF(4)-bbH(i)-1);
		vector<double> topV;
		colon(bbF(1,0), bbSHH(0, i), bbF(3, 0)-bbH(0, i)-1, topV);
		cv::Mat_<double> top(1, topV.size());
		for (unsigned j=0; j < topV.size(); ++j)
			top(0,j) = cvRound(topV.at(j));
		//printMatrix(top);

		//grid  = ntuples(top,left);
		cv::Mat_<double> grid;
		ntuples<cv::Mat_<double> >(top, left, grid);
		if (grid.cols == 0)
			continue;

		//bbs{end+1} = [grid(2,:); ...
		//			   grid(1,:); ...
		//			   grid(2,:)+bbW(i)-1; ...
		//			   grid(1,:)+bbH(i)-1; ...
		//			   idx*ones(1,size(grid,2));
		//			   length(left)*ones(1,size(grid,2));];
		cv::Mat_<double> bbsMat(6, grid.cols);
		Mat row = bbsMat.row(0); grid.row(1).copyTo(row);
		row = bbsMat.row(1); grid.row(0).copyTo(row);
		row = bbsMat.row(2); cv::Mat_<double> gridRow = grid.row(1)+bbW(0,i)-1; gridRow.copyTo(row);
		row = bbsMat.row(3); gridRow = grid.row(0)+bbH(0,i)-1; gridRow.copyTo(row);
		row = bbsMat.row(4); gridRow = cv::Mat_<double>::ones(1, grid.cols)*idx; gridRow.copyTo(row);
		row = bbsMat.row(5); gridRow = cv::Mat_<double>::ones(1, grid.cols)*left.cols; gridRow.copyTo(row);
		//printMatrix(bbsMat);
		bbs.push_back(bbsMat);

		//sca = [sca [bbH(i); bbW(i)]];
		cv::Mat_<double> tmp = cv::Mat_<double>::zeros(2, sca.cols+1);
		tmp(0, tmp.cols-1) = bbH(0,i);
		tmp(1, tmp.cols-1) = bbW(0,i);
		for (int c=0; c < sca.cols; ++c)
			tmp.col(c) += sca.col(c);
		tmp.copyTo(sca);
		//printMatrix(sca);

		//idx = idx + 1;
		idx++;
	}

	//bb_out = [];
	//for i = 1:length(bbs)
	//	bb_out = [bb_out bbs{i}];
	//end
	bb_out.create(0, 0);
	for (unsigned i=0; i < bbs.size(); ++i)
	{
		cv::Mat_<double>& bbsi = bbs.at(i);
		cv::Mat_<double> tmp = cv::Mat_<double>::zeros(bbsi.rows, bb_out.cols + bbsi.cols);
		for (int c=0; c < bb_out.cols; ++c)
			tmp.col(c) += bb_out.col(c);
		for (int c=0; c < bbsi.cols; ++c)
			tmp.col(c+bb_out.cols) += bbsi.col(c);
		tmp.copyTo(bb_out);
	}

	return true;
}

void Detector::generateFeatures(unsigned nTrees, unsigned nFeatures, cv::Mat_<double>& features, bool)
{
	LOG("Detector::generateFeatures");
	m_featureType = Detector::FORREST;
	static const double shift = 0.2;
	static const double sca = 1.0;
	static const double offset = shift;

	// x = repmat(ntuples(0:SHI:1,0:SHI:1),2,1);
	std::vector<double> v;
	colon<double>(0, shift, 1, v);  // 1 x 6
	cv::Mat_<double> tup(v); // 1 x 6
	//printMatrix(tup, "tup");
	tup = tup.t(); // 1 x 6
	//printMatrix(tup, "tupT");
	cv::Mat_<double> n;
	ntuples<cv::Mat_<double> >(tup, tup, n); // 2 x 36
	//printMatrix(n, "n");
	cv::Mat_<double> x;
	repmat<cv::Mat_<double> >(n, 2, x, 1); // 4 x 36
	//printMatrix(x, "x");

	// x = [x x + SHI/2]; 4 x 72
	cv::Mat_<double> tmp = cv::Mat_<double>::zeros(x.rows, 2 * x.cols);
	for (int c=0; c < x.cols; ++c)
		tmp.col(c) += x.col(c);
	for (int c=0; c < x.cols; ++c)
		tmp.col(c+x.cols) += (x.col(c) + (shift/2));
	tmp.copyTo(x);
	//printMatrix(x, "x");

	// k = size(x,2); 72
	int k = x.cols;

	//r = x; r(3,:) = r(3,:) + (SCA*rand(1,k)+OFF); 4 x 72
	cv::Mat_<double> rnd(1, k);
	cv::Mat_<double> r(x.rows, x.cols); x.copyTo(r);
	//printMatrix(r, "r");
	cv::randu(rnd, 0, 1);
	//printMatrix(rnd, "rnd");
	r.row(2) += ((sca*rnd) + offset);
	//printMatrix(r, "r");

	//l = x; l(3,:) = l(3,:) - (SCA*rand(1,k)+OFF); 4 x 72
	cv::Mat_<double> l(x.rows, x.cols); x.copyTo(l);
	cv::randu(rnd, 0, 1);
	l.row(2) -= ((sca*rnd) + offset);
	//printMatrix(l, "l");

	//t = x; t(4,:) = t(4,:) - (SCA*rand(1,k)+OFF); 4 x 72
	cv::Mat_<double> t(x.rows, x.cols); x.copyTo(t);
	cv::randu(rnd, 0, 1);
	t.row(3) -= ((sca*rnd) + offset);
	//printMatrix(t, "t");

	//b = x; b(4,:) = b(4,:) + (SCA*rand(1,k)+OFF); 4 x 72
	cv::Mat_<double> b(x.rows, x.cols); x.copyTo(b);
	cv::randu(rnd, 0, 1);
	b.row(3) += ((sca*rnd) + offset);
	//printMatrix(b, "b");

	// x = [r l t b]; 4 x 228
	tmp = cv::Mat_<double>::zeros(x.rows, r.cols+l.cols+t.cols+b.cols);
	for (int c=0; c < r.cols; ++c)
		tmp.col(c) += r.col(c);
	for (int c=0; c < l.cols; ++c)
		tmp.col(c+r.cols) += l.col(c);
	for (int c=0; c < t.cols; ++c)
		tmp.col(c+r.cols+l.cols) += t.col(c);
	for (int c=0; c < b.cols; ++c)
		tmp.col(c+r.cols+l.cols+t.cols) += b.col(c);
	tmp.copyTo(x);
	//printMatrix(x, "x = [r l t b]");
	cv::Mat_<double> xt = x.t();
	//printMatrix(xt, "xt = col[r l t b]");

	// could use cvReduce?
	// idx = all(x([1 2],:) < 1 & x([1 2],:) > 0,1); 1 x 228
	cv::Mat_<int> idx = cv::Mat_<int>::zeros(1, x.cols);
	for (int c=0; c < x.cols; ++c)
	{
		if ((x(0,c) < 1) && (x(1,c) < 1) && (x(0,c) >= 0.1) && (x(1,c) >= 0.1))
			idx(0, c) = 1;
	}
	//printMatrix(idx, "idx");

	// x = x(:,idx); 4 x 164
	int i=0;
	for (int c=0; c < idx.cols; ++c)
	{
		if (idx(c))
		{
			Mat col = tmp.col(i++);
			x.col(c).copyTo(col);
		}
	}
	if (i > 0)
	{
		x.create(x.rows, i);
		for (int c=0; c < i; ++c)
		{
			Mat col = x.col(c);
			(tmp.col(c)).copyTo(col);
		}
	}
	//printMatrix(x, "x = x(:,idx)");

	// x(x > 1) = 1; 4 x 164
	// x(x < 0) = 0; 4 x 164
	for (int r=0; r < x.rows; ++r)
		for (int c=0; c < x.cols; ++c)
		{
			if (x(r,c) > 1) x(r,c) = 1;
			if (x(r,c) < 0) x(r,c) = 0;
		}
	//printMatrix(x, "x(x > 1) = 1; x(x < 0) = 0");

	// numF = size(x,2); 164
	unsigned numF = x.cols;

	// x = x(:,randperm(numF)); 4 x 164
	i=0;
	tmp.create(x.rows, x.cols);
	vector<int> indexes;
	colon<int>(0, 1, numF-1, indexes);
	std::random_shuffle(indexes.begin(), indexes.end());
	for (vector<int>::const_iterator vit=indexes.begin(); vit != indexes.end(); ++vit)
	{
		cv::Mat col = tmp.col(i++);
		(x.col(*vit)).copyTo(col);
	}
	tmp.copyTo(x);
	//printMatrix(x, "x = x(:,randperm(numF))");

	// x = x(:,1:nFEAT*nTREES); 4 x 130
	tmp.create(x.rows, nTrees * nFeatures);
	for (int c=0; c < tmp.cols; ++c)
	{
		Mat col = tmp.col(c);
		(x.col(c)).copyTo(col);
	}
	//tmp.copyTo(x);
	//printMatrix(tmp, "x = x(:,1:nFEAT*nTREES)");

	// x = reshape(x,4*nFEAT,nTREES); 52 x 10
	features = reshape<double>(tmp, 4*nFeatures, nTrees);
	//printMatrix(features, "x = reshape(x,4*nFEAT,nTREES");
}

double Detector::bb_overlap(double *bb1, double *bb2)
{
	//LOG("Detector::bb_overlap");
	if (bb1[0] > bb2[2]) { return 0.0; }
	if (bb1[1] > bb2[3]) { return 0.0; }
	if (bb1[2] < bb2[0]) { return 0.0; }
	if (bb1[3] < bb2[1]) { return 0.0; }
	
	double colInt = std::min(bb1[2], bb2[2]) - std::max(bb1[0], bb2[0]) + 1;
	double rowInt = std::min(bb1[3], bb2[3]) - std::max(bb1[1], bb2[1]) + 1;

	double intersection = colInt * rowInt;
	double area1 = (bb1[2]-bb1[0]+1)*(bb1[3]-bb1[1]+1);
	double area2 = (bb2[2]-bb2[0]+1)*(bb2[3]-bb2[1]+1);
	return intersection / (area1 + area2 - intersection);
}

void Detector::bb_overlap(const cv::Mat_<double>& bb, cv::Mat_<double>& overlap)
{
	double *bbData = (double*) bb.data;
	int nBB = bb.cols;
	//int mBB = bb.rows;

	// Output
	overlap = cv::Mat_<double>::zeros(1, nBB*(nBB-1)/2);
	double *out = (double*) overlap.data;

	for (int i = 0; i < nBB-1; i++) {
		for (int j = i+1; j < nBB; j++) {
			*out++ = bb_overlap(bbData + i, bbData + j);
		}
	}
}

void Detector::bb_overlap(const cv::Mat_<double>& bb1, const cv::Mat_<double>& bb2, cv::Mat_<double>& overlap)
{
	//LOG("Detector::bb_overlap");
	int N1 = bb1.cols; // bb1 cols
	int N2 = bb2.cols; // bb2 cols

	if (N1 == 0 || N2 == 0) {
		N1 = 0; N2 = 0;
	}

	overlap.create(N1, N2);
	double *data = (double *) overlap.data;

	for (int j = 0; j < N2; j++)
	{
		for (int i = 0; i < N1; i++)
		{
			cv::Mat_<double> bb1Col(bb1.rows, 1);
			cv::Mat_<double> bb2Col(bb2.rows, 1);
			bb1.col(i).copyTo(bb1Col);
			bb2.col(j).copyTo(bb2Col);
			*data++ = bb_overlap((double*) bb1Col.data, (double*) bb2Col.data);
		}
	}
}

void Detector::imagePatch(const Mat_<int>& inMat, const cv::Mat_<double>& bb, cv::Mat_<unsigned char>& outMat, const ParOptionsType& p_par, double randomize)
{
	LOG("Detector::imagePatch");
	if (randomize > 0)
	{
		//   rand('state',randomize);
		//   randn('state',randomize);
		//   
		//   NOISE = p_par.noise;
		//   ANGLE = p_par.angle;
		//   SCALE = p_par.scale;
		//   SHIFT = p_par.shift;
		//   
		//   cp  = bb_center(bb)-1;
		//   Sh1 = [1 0 -cp(1); 0 1 -cp(2); 0 0 1];
		//   
		//   sca = 1-SCALE*(rand-0.5);
		//   Sca = diag([sca sca 1]);
		//   
		//   ang = 2*pi/360*ANGLE*(rand-0.5);
		//   ca = cos(ang);
		//   sa = sin(ang);
		//   Ang = [ca, -sa; sa, ca];
		//   Ang(end+1,end+1) = 1;
		//   
		//   shR  = SHIFT*bb_height(bb)*(rand-0.5);
		//   shC  = SHIFT*bb_width(bb)*(rand-0.5);
		//   Sh2 = [1 0 shC; 0 1 shR; 0 0 1];
		//   
		//   bbW = bb_width(bb)-1;
		//   bbH = bb_height(bb)-1;
		//   box = [-bbW/2 bbW/2 -bbH/2 bbH/2];
		//   
		//   H     = Sh2*Ang*Sca*Sh1;
		//   bbsize = bb_size(bb);
		//   patch = uint8(warp(img,inv(H),box) + NOISE*randn(bbsize(1),bbsize(2)));
		unsigned noise = p_par.noise;
		unsigned angle = p_par.angle;
		double scale = p_par.scale;
		double shift = p_par.shift;

		cv::Mat_<double> cp;
		bbCenter(bb, cp);
		//printMatrix(bb, "bb");
		cp -= 1;
		//printMatrix(cp, "cp");
		cv::Mat_<double> Sh1 = (cv::Mat_<double>(3, 3) << 1, 0, -cp(0, 0), 0, 1, -cp(1, 0), 0, 0, 1);
		//printMatrix(Sh1, "Sh1");

		double sca = 1 - scale*((double)(*m_rng) - 0.5);
		cv::Mat_<double> Sca = cv::Mat_<double>::zeros(3,3);
		Sca(0,0) = sca;
		Sca(1,1) = sca;
		Sca(2,2) = 1;
		//printMatrix(Sca, "Sca");

		double ang = 2*pi/360*angle*((double)(*m_rng)-0.5);
		double ca = cos(ang);
		double sa = sin(ang);
		cv::Mat_<double> Ang = (cv::Mat_<double>(3, 3) << ca, -sa, 0, sa, ca, 0, 0, 0, 1);
		//printMatrix(Ang, "Ang");

		double shR = shift*bbHeight(bb)*((double)(*m_rng)-0.5);
		double shC = shift*bbWidth(bb)*((double)(*m_rng)-0.5);
		cv::Mat_<double> Sh2 = (cv::Mat_<double>(3, 3) << 1, 0, shC, 0, 1, shR, 0, 0, 1);
		//printMatrix(Sh2, "Sh2");

		double bbW = bbWidth(bb)-1;
		double bbH = bbHeight(bb)-1;
		cv::Mat_<double> box = (cv::Mat_<double>(1, 4) << -bbW/2, bbW/2, -bbH/2, bbH/2);
		//printMatrix(box, "box");

		cv::Mat_<double> H = Sh2*Ang*Sca*Sh1;
		//printMatrix(H, "H");
		cv::Mat_<double> bbsize;
		bbSize(bb, bbsize);
		//printMatrix(bbsize, "bbsize");

		cv::Mat_<double> HInv = H.inv();
		//printMatrix(HInv, "HInv");
		cv::Mat_<double> warpMat;
		//printMatrix(inMat, "inMat");
		warp(inMat, HInv, box, warpMat);
		//printMatrix(warpMat, "warpMat");

		cv::Mat_<double> randnMat;
		randnMat = cv::Mat_<int>::zeros(bbsize(0,0),bbsize(1,0));
		m_rng->fill(randnMat, RNG::NORMAL, Scalar(0), Scalar(1));
		//randn(randnMat, Scalar(0), Scalar(1));
		//printMatrix(randnMat, "randnMat");
		warpMat = warpMat + (noise * randnMat);
		//printMatrix(warpMat, "warpMat");
		outMat.create(bbsize(0,0),bbsize(1,0));
		for (int i=0; i < warpMat.rows ;i++)
		{
			for (int j=0; j < warpMat.cols; j++)
				outMat(i,j) = (unsigned int) warpMat(i,j);
		}
		//printMatrix(outMat, "outMat");
	}
	else
	{
		bool isint = true;
		for (int r=0; r < bb.rows; ++r)
		{
			for (int c=0; c < bb.cols; ++c)
			{
				 if ((floor(bb(r,c)) - bb(r,c)) != 0.00000000000)
				 {
					 isint = false;
					 break;
				 }
			}
			if (!isint) break;
		}
		if (isint)
		{
			//L = max([1 bb(1)]);
			//T = max([1 bb(2)]);
			//R = min([size(img,2) bb(3)]);
			//B = min([size(img,1) bb(4)]);
			//patch = img(T:B,L:R);

			//printMatrix(bb, "bb");
			int L = max(0, (int) bb(0,0) - 1); // x0
			int T = max(0, (int) bb(1,0) - 1); // y0
			int R = min(inMat.cols, (int) bb(2,0)); // x1
			int B = min(inMat.rows, (int) bb(3,0)); // y1

			outMat.create(B-T, R-L);
			for (int i=T; i < B; ++i)
				for (int j=L; j < R; ++j)
					outMat(i-T, j-L) = inMat(i, j);
		}
		else
		{
			// Sub-pixel accuracy
			//cp = 0.5 * [bb(1)+bb(3); bb(2)+bb(4)]-1;
			cv::Mat_<double> cp(2,1);
			cp(0,0) = 0.5 * (bb(0) + bb(2)) - 1;
			cp(1,0) = 0.5 * (bb(1) + bb(3)) - 1;
			//printMatrix(cp, "cp");
			//H = [1 0 -cp(1); 0 1 -cp(2); 0 0 1];
			cv::Mat_<double> H(3,3);
			H(0,0) = 1; H(0,1) = 0; H(0,2) = -cp(0,0);
			H(1,0) = 0; H(1,1) = 1; H(1,2) = -cp(1,0);
			H(2,0) = 0; H(2,1) = 0; H(2,2) = 1;
			//printMatrix(H, "H");
			//
			//bbW = bb(3,:)-bb(1,:);
			double bbW = bb(2,0) - bb(0,0);
			//bbH = bb(4,:)-bb(2,:);
			double bbH = bb(3,0) - bb(1,0);
			//if bbW <= 0 || bbH <= 0
			//    patch = [];
			//    return;
			//end
			if ((bbW <= 0) || (bbH <= 0))
			{
				outMat.create(0,0);
				return;
			}
			//box = [-bbW/2 bbW/2 -bbH/2 bbH/2];
			cv::Mat_<double> box(1,4);
			box(0,0) = -bbW/2; box(0,1) = bbW/2;
			box(0,2) = -bbH/2; box(0,3) = bbH/2;
			//printMatrix(box, "box");
			
			//
			// Does the test below check for color images (RGB)?
			//
			//if size(img,3) == 3
			//    for i = 1:3
			//        P = warp(img(:,:,i),inv(H),box);
			//        patch(:,:,i) = uint8(P);
			//    end
			//else
			//    patch = warp(img,inv(H),box);
			//    patch = uint8(patch);
			//end
			cv::Mat_<double> patch;
			cv::Mat_<double> Hinv = H.inv();
			//printMatrix(Hinv, "Hinv");
			warp(inMat, Hinv, box, patch);
			//printMatrix(patch, "patch");
			patch.convertTo(outMat, CV_8UC1);
			//printMatrix(outMat, "outMat");
		}
	}

	//printMatrix(inMat, "inMat");
	//printMatrix(outMat, "outMat");
}

// rowwise access
#define coord(x, y, width, height) (x+y*width)
#define nextrow(tmp, width, height) ((tmp)+width)
#define nextcol(tmp, width, height) ((tmp)+1)
#define nextr_c(tmp, width, height) ((tmp)+width+1)
#define M(r, c) H[c+r*3]

void warp_image_roi(unsigned int *image, int w, int h, double *H,
                    double xmin, double xmax, double ymin, double ymax,
                    double fill, double *result)
{
	LOG("Detector::warp_image_roi");
	double curx, cury, curz, wx, wy, wz, ox, oy, oz;
	int x, y;
	unsigned int *tmp;
	double *output=result, i, j, xx, yy;
	/* precalulate necessary constant with respect to i,j offset 
	translation, H is column oriented (transposed) */   
	ox = M(0,2); //95.000
	oy = M(1,2); //90.49999
	oz = M(2,2); //1.000

	yy = ymin;  // -42.49999
	for (j=0; j<(int)(ymax-ymin+1); j++)
	{
		/* calculate x, y for current row */
		curx = M(0,1)*yy + ox;
		cury = M(1,1)*yy + oy;
		curz = M(2,1)*yy + oz;
		xx = xmin; 
		yy = yy + 1;
		for (i=0; i<(int)(xmax-xmin+1); i++)
		{
			/* calculate x, y in current column */
			wx = M(0,0)*xx + curx;
			wy = M(1,0)*xx + cury;
			wz = M(2,0)*xx + curz;
			//       printf("%g %g, %g %g %g\n", xx, yy, wx, wy, wz);
			wx /= wz; wy /= wz;
			xx = xx + 1;

			x = (int)floor(wx);
			y = (int)floor(wy);

			if (x>=0 && y>=0)
			{
				wx -= x; wy -= y; 
				if (x+1==w && wx==1)
					x--;
				if (y+1==h && wy==1)
					y--;
				if ((x+1)<w && (y+1)<h)
				{
					tmp = &image[coord(x,y,w,h)];
					/* image[x,y]*(1-wx)*(1-wy) + image[x+1,y]*wx*(1-wy) +
					image[x,y+1]*(1-wx)*wy + image[x+1,y+1]*wx*wy */
					*output++ = 
						(*(tmp) * (1-wx) + *nextcol(tmp, w, h) * wx) * (1-wy) +
						(*nextrow(tmp,w,h) * (1-wx) + *nextr_c(tmp,w,h) * wx) * wy;
				} else 
					*output++ = fill;
			} else 
				*output++ = fill;
		}
	}
}

void Detector::warp(const cv::Mat_<int>& imMat, const cv::Mat_<double>& HMat, const cv::Mat_<double>& B, cv::Mat_<double>& outMat)
{
	LOG("Detector::warp");
	int w, h;
	unsigned *im = (unsigned *) imMat.data;
	double *result;
	double *H = (double*) HMat.data;
	double xmin, xmax, ymin, ymax, fill;

	w = imMat.cols;
	h = imMat.rows;

	xmin = B(0,0); xmax = B(0,1);
	ymin = B(0,2); ymax = B(0,3);

	fill=0;

	result = new double[((int)(xmax-xmin+1)*(int)(ymax-ymin+1))];
	{
		warp_image_roi(im, w, h, H, xmin, xmax, ymin, ymax, fill, result);
	}

	int num_cols = (int)(xmax-xmin+1);
	int num_rows = (int)(ymax-ymin+1);

	const double* s_ptr = result;
	outMat.create(num_rows, num_cols);
	outMat = cv::Mat_<double>::zeros(num_rows, num_cols);
	for (int i=0;i<num_rows;i++)
	{
		for (int j=0; j<num_cols; j++)
			outMat(i,j) = *s_ptr++;
	}

	delete [] result;
}

void to_matlab(const double *image, int num_cols, int num_rows, cv::Mat_<double>& outMat)
{
	LOG("Detector::to_matlab");
	int i, j;
	const double* s_ptr = image;
	outMat.create(num_rows, num_cols);
	outMat = cv::Mat_<double>::zeros(num_rows, num_cols);
	for (i=0;i<num_rows;i++)
	{
		for (j=0; j<num_cols; j++)
			outMat(i,j) = *s_ptr++;
	}
}


double Detector::bbHeight(const cv::Mat_<double>& bb)
{
	LOG("Detector::bbHeight");
	//function bbH = bb_height(bb)
	//% Info
	//
	//bbH    = bb(4,:)-bb(2,:)+1;
	return bb(3,0) - bb(1,0) + 1;
}

double Detector::bbWidth(const cv::Mat_<double>& bb)
{
	LOG("Detector::bbWidth");
	//function bbW = bb_width(bb)
	//% Info
	//
	//bbW    = bb(3,:)-bb(1,:)+1;
	double w = bb(2,0) - bb(0,0) + 1;
	return w;
}

void Detector::bbSize(const cv::Mat_<double>& bb, cv::Mat_<double>& s)
{
	LOG("Detector::bbSize");
	//function s = bb_size(bb)
	//
	//s = [bb(4,:)-bb(2,:)+1; bb(3,:)-bb(1,:)+1];
	s = (cv::Mat_<double>(2, 1) << bb(3,0)-bb(1,0)+1, bb(2,0)-bb(0,0)+1);
}

void Detector::bbCenter(const cv::Mat_<double>& bb, cv::Mat_<double>& center)
{
	LOG("Detector::bbCenter");
	//function center = bb_center(bb)
	//% Info
	//
	//if isempty(bb)
	//	center = []; 
	//	return;
	//end
	//
	//center = 0.5 * [bb(1,:)+bb(3,:); bb(2,:)+bb(4,:)];
	if (bb.empty())
	{
		center.create(0,0);
		return;
	}

	center.create(2, bb.cols);
	center.row(0) = 0.5 * (bb.row(0) + bb.row(2));
	center.row(1) = 0.5 * (bb.row(1) + bb.row(3));
}

template <class T>
bool pairValueSortPredicate(T i, T j) {return (i.second > j.second);}

void Detector::tldGeneratePositiveData(const cv::Mat_<double>& overlap, const ImagePairType& img0, const ParOptionsType& init, cv::Mat_<double>& pX, cv::Mat_<double>& pEx, cv::Mat_<double>& bbP0)
{
	LOG("Detector::tldGeneratePositiveData");
	//% Get closest bbox
	//[dummy1,idxP] = max(overlap);
	//bbP0 =  tld.grid(1:4,idxP);
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(overlap, &minVal, &maxVal, &minLoc, &maxLoc);
	bbP0.create(4, 1);
	for (int r=0; r < 4; ++r)
		bbP0(r, 0) = m_grid(r, maxLoc.x);
	//printMatrix(bbP0, "bbP0");

	//% Get overlapping bboxes
	//idxP = find(overlap > 0.6); 1 x X indexes of elements > 0.6
	std::vector<std::pair<unsigned, double> > idx;
	for (int c=0; c < overlap.cols; ++c)
	{
		if (overlap(0, c) > 0.6)
			idx.push_back(std::make_pair(c, overlap(0, c)));
	}
	cv::Mat_<int> idxP(1, idx.size());
	unsigned *data = (unsigned *)idxP.data;
	std::vector<std::pair<unsigned, double> >::iterator it;
	for (it = idx.begin(); it != idx.end(); ++it)
		*data++ = it->first;
	//printMatrix(idxP, "idxP");

	//if length(idxP) > p_par.num_closest
	//    [dummy2,sIdx] = sort(overlap(idxP),'descend');    
	//    idxP = idxP(sIdx(1:p_par.num_closest));
	//end
	if ((unsigned)idxP.cols > init.num_closest)
	{
		idxP.create(1, init.num_closest);
		std::sort(idx.begin(), idx.end(), pairValueSortPredicate<std::pair<unsigned, double> >);
		idx.resize(init.num_closest);
		data = (unsigned *)idxP.data;
		for (it = idx.begin(); it != idx.end(); ++it)
			*data++ = it->first;
	}
	//printMatrix(idxP, "idxP");

	//bbP  = tld.grid(:,idxP);
	//if isempty(bbP), return; end
	cv::Mat_<double> bbP(m_grid.rows, idxP.cols);
	for (int c=0; c < idxP.cols; ++c)
	{
		Mat col = bbP.col(c);
		m_grid.col(idxP(0, c)).copyTo(col);
	}
	//printMatrix(bbP, "bbP");
	if (bbP.empty())
		return;

	//% Get hull
	//bbH  = bb_hull(bbP);
	//cols = bbH(1):bbH(3);
	//rows = bbH(2):bbH(4);
	cv::Mat_<double> bbH;
	bbHull(bbP, bbH);
	//printMatrix(bbH, "bbH");
	std::vector<double> cols;
	colon<double>(bbH(0,0), 1, bbH(2,0), cols);
	std::vector<double> rows;
	colon<double>(bbH(1,0), 1, bbH(3,0), rows);

	//im1 = im0;
	//pEx = tldGetPattern(im1,bbP0,tld.model.patchsize);
	//if tld.model.fliplr
	//	pEx = [pEx tldGetPattern(im1,bbP0,tld.model.patchsize,1)];
	ImagePairType im1;
	im1.first = new cv::Mat_<int>(img0.first->rows, img0.first->cols);
	img0.first->copyTo(*im1.first);
	im1.second = new cv::Mat_<int>(img0.second->rows, img0.second->cols);
	img0.second->copyTo(*im1.second);
	//printMatrix(*im1.second, "blur");

	getPattern(im1, bbP0, m_patchSize, pEx);
	//printMatrix(pEx, "pEx");
	if (m_fliplr)
	{
		cv::Mat_<double> pExFlip(pEx.rows, pEx.cols + 1);
		cv::Mat_<double> pExFlipCol = pExFlip.col(1);
		getPattern(im1, bbP0, m_patchSize, pExFlipCol, m_fliplr);
		Mat col = pExFlip.col(0);
		pEx.col(0).copyTo(col);
		pEx.create(pExFlip.rows, pExFlip.cols);
		pExFlip.copyTo(pEx);
	}
	//printMatrix(pEx, "pEx");

	//for i = 1:p_par.num_warps
	//    if i > 1
	//        randomize = rand; % Sets the internal randomizer to the same state
	//        %patch_input = img_patch(im0.input,bbH,randomize,p_par);
	//        patch_blur = img_patch(im0.blur,bbH,randomize,p_par);
	//        im1.blur(rows,cols) = patch_blur;
	//        %im1.input(rows,cols) = patch_input;
	//    end
	//    
	//    % Measures on blured image
	//    pX  = [pX fern(5,im1,idxP,0)]; 10x10 , 10x20 ...
	//    
	//    % Measures on input image
	//    %pEx(:,i) = tldGetPattern(im1,bbP0,tld.model.patchsize);
	//    %pEx = [pEx tldGetPattern(im1,tld.grid(1:4,idxP),tld.model.patchsize)];
	//    
	//end
	for (unsigned i=1; i <= init.num_warps; ++i)
	{
		if (i > 1)
		{
			static uint64 rngState = m_rng->state;
			m_rng->state = rngState;
			double randomize = (double)(*m_rng);
			cv::Mat_<unsigned char> patch_blur;
			//randomize(0,0) = 0.779079247486771;
			imagePatch(*(img0.second), bbH, patch_blur, init, randomize);
			for (unsigned r=0; r < rows.size(); ++r)
			{
				for (unsigned c=0; c < cols.size(); ++c)
					(*im1.second)(rows.at(r), cols.at(c)) = patch_blur(r, c);
			}
			//printMatrix(*im1.second, "blur");
		}

		cv::Mat_<double> patt;
		cv::Mat_<double> status;
		m_fern->getPatterns(im1, idxP, 0, patt, status);
		//printMatrix(patt, "patt");
		cv::Mat_<double> tmp(patt.rows, pX.cols + patt.cols);
		if (i>1)
			for (int c=0; c < pX.cols; ++c)
			{
				Mat col = tmp.col(c);
				pX.col(c).copyTo(col);
			}
		for (int c=0; c < patt.cols; ++c)
		{
			Mat col = tmp.col(c+pX.cols);
			patt.col(c).copyTo(col);
		}
		//printMatrix(tmp, "tmp");
		pX.create(tmp.rows, tmp.cols);
		tmp.copyTo(pX);
	}
	//printMatrix(pX, "pX");
}

void Detector::tldGenerateNegativeData(const cv::Mat_<double>& overlap, const ImagePairType& img, cv::Mat_<double>& nX, cv::Mat_<double>& nEx)
{
	LOG("Detector::tldGenerateNegativeData");
	//printMatrix(overlap, "grid");
	//printMatrix(grid, "overlap");
	//% Measure patterns on all bboxes that are far from initial bbox
	//idxN        = find(overlap<tld.n_par.overlap);
	//[nX,status] = fern(5,img,idxN,tld.var/2);
	std::vector<std::pair<unsigned, double> > idx;
	for (int c=0; c < overlap.cols; ++c)
	{
		if (overlap(0, c) < m_n_par.overlap)
			idx.push_back(std::make_pair(c, overlap(0, c)));
	}
	cv::Mat_<int> idxN(1, idx.size());
	unsigned *data = (unsigned *)idxN.data;
	std::vector<std::pair<unsigned, double> >::iterator it;
	for (it = idx.begin(); it != idx.end(); ++it)
		*data++ = it->first;
	//printMatrix(idxN, "idxN");

	cv::Mat_<double> status;
	m_fern->getPatterns(img, idxN, m_var/2, nX, status);
	//printMatrix(nX, "nX");
	//printMatrix(status, "status");

	//idxN        = idxN(status==1); % bboxes far and with big variance
	//nX          = nX(:,status==1);
	std::vector<std::pair<unsigned,unsigned> >tmp;
	for (int c=0; c < status.cols; ++c)
	{
		if (status(0, c) == 1)
			tmp.push_back(std::make_pair(c, idxN(0, c)));
	}
	idxN.create(1, tmp.size());
	data = (unsigned *)idxN.data;
	std::vector<std::pair<unsigned, unsigned> >::iterator tmpit;
	for (tmpit = tmp.begin(); tmpit != tmp.end(); ++tmpit)
		*data++ = tmpit->second;
	//printMatrix(idxN, "idxN");
	cv::Mat_<double> nX_(nX);
	nX.create(nX.rows, tmp.size());
	int i=0;
	for (tmpit = tmp.begin(); tmpit != tmp.end(); ++tmpit)
	{
		Mat col = nX.col(i++);
		nX_.col(tmpit->first).copyTo(col);
	}
	//printMatrix(nX, "nX");

	//% Randomly select 'num_patches' bboxes and measure patches
	//idx = randvalues(1:length(idxN),tld.n_par.num_patches);
	//bb  = tld.grid(:,idxN(idx));
	//nEx = tldGetPattern(img,bb,tld.model.patchsize);
	cv::Mat_<double> idx1;
	std::vector<double> indexes;
	colon<double>(0, 1, idxN.cols-1, indexes);
	cv::Mat_<double> inTmp(indexes, true);
	cv::Mat_<double> in = inTmp.t();
	randValues(in, m_n_par.num_patches, idx1);
	cv::Mat_<double> bb(m_grid.rows, idx1.cols); // 6 x 100
	for (int c=0; c < idx1.cols; ++c)
	{
		Mat col = bb.col(c);
		m_grid.col(idxN(0, idx1(0, c))).copyTo(col);
	}
	//printMatrix(bb, "bb");
	getPattern(img, bb, m_patchSize, nEx);
	//printMatrix(nEx, "nEx");
}

void Detector::randValues(const cv::Mat_<double>& in, double k, cv::Mat_<double>& out)
{
	LOG("Detector::randValues");
	//out = [];
	//N = size(in,2);
	//if k == 0
	//  return;
	//end
	//if k > N
	//  k = N;
	//end
	//if k/N < 0.0001
	// i1 = unique(ceil(N*rand(1,k)));
	// out = in(:,i1);
	//else
	// i2 = randperm(N);
	// out = in(:,sort(i2(1:k)));
	//end
	int N = in.cols;
	if (k == 0) return;
	if (k > N) k = N;
	if (k/N < 0.0001)
	{
		cv::Mat_<double> i1(1, k);
		cv::randu(i1, Scalar(0), Scalar(1));
		for (int i=0; i < k; ++i)
			i1(0, i) = ceil(N * i1(0, i));
		//printMatrix(i1, "i1");
		std::sort(i1.begin(), i1.end());
		//printMatrix(i1, "i1 sorted");
		std::vector<double> si1(i1.cols);
		std::vector<double>::iterator it;
		it=std::unique_copy(i1.begin(),i1.end(),si1.begin());
		si1.resize(it - si1.begin());
		out.create(1, si1.size());
		for (unsigned c=0; c < si1.size(); ++c)
		{
			Mat col = out.col(c);
			in.col(si1.at(c)).copyTo(col);
		}
		//printMatrix(out, "out");
	}
	else
	{
		std::vector<double> indexes;
		colon<double>(0, 1, N-1, indexes);
		std::random_shuffle(indexes.begin(), indexes.end());
		indexes.resize(k);
		std::sort(indexes.begin(), indexes.end());
		out.create(1, k);
		for (unsigned c=0; c < indexes.size(); ++c)
		{
			Mat col = out.col(c);
			in.col(indexes.at(c)).copyTo(col);
		}
		//printMatrix(out, "out");
	}
}

void Detector::tldSplitNegativeData(const cv::Mat_<double>& nX, const cv::Mat_<double>& nEx, cv::Mat_<double>& nX1, cv::Mat_<double>& nX2, cv::Mat_<double>& nEx1, cv::Mat_<double>& nEx2)
{
	LOG("Detector::tldSplitNegativeData");
	//N    = size(nX,2);
	//idx  = randperm(N);
	unsigned N = nX.cols;
	cv::Mat_<int> idx;
	randperm<int>(N, idx);
	//printMatrix(idx, "idx");

	//nX   = nX(:,idx);
	cv::Mat_<double> nX_(nX.rows, idx.cols);
	for (int c = 0; c < idx.cols; ++c)
	{
		Mat col = nX_.col(c);
		nX.col(idx(0 ,c)).copyTo(col);
	}
	//printMatrix(nX_, "nX_");

	//nX1  = nX(:,1:N/2); 
	nX1.create(nX_.rows, N/2);
	for (int c=0; c < nX1.cols; ++c)
	{
		Mat col = nX1.col(c);
		nX_.col(c).copyTo(col);
	}
	//printMatrix(nX1, "nX1");

	//nX2  = nX(:,N/2+1:end);
	nX2.create(nX_.rows, N/2);
	for (int c=0; c < nX2.cols; ++c)
	{
		Mat col = nX2.col(c);
		nX_.col(c+(N/2)).copyTo(col);
	}
	//printMatrix(nX2, "nX2");

	//N    = size(nEx,2);
	N = nEx.cols;

	//idx  = randperm(N);
	randperm<int>(N, idx);
	//printMatrix(idx, "idx");

	//nEx  = nEx(:,idx);
	cv::Mat_<double> nEx_(nEx.rows, nEx.cols);
	for (int c = 0; c < idx.cols; ++c)
	{
		Mat col = nEx_.col(c);
		nEx.col(idx(0 ,c)).copyTo(col);
	}
	//printMatrix(nEx_, "nEx_");

	//nEx1 = nEx(:,1:N/2);
	nEx1.create(nEx_.rows, N/2);
	for (int c=0; c < nEx1.cols; ++c)
	{
		Mat col = nEx1.col(c);
		nEx_.col(c).copyTo(col);
	}
	//printMatrix(nEx1, "nEx1");

	//nEx2 = nEx(:,N/2+1:end);
	nEx2.create(nEx_.rows, N/2);
	for (int c=0; c < nEx2.cols; ++c)
	{
		Mat col = nEx2.col(c);
		nEx_.col(c+(N/2)).copyTo(col);
	}
	//printMatrix(nEx2, "nEx2");
}

void Detector::bbHull(const cv::Mat_<double>& bb0, cv::Mat_<double>& hull)
{
	LOG("Detector::bbHull");
	//bb=[min(bb0(1,:)); ...
	//	min(bb0(2,:)); ...
	//	max(bb0(3,:)); ...
	//	max(bb0(4,:))];
	hull.create(4,1);
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(bb0.row(0), &minVal, &maxVal, &minLoc, &maxLoc);
	hull(0, 0) = minVal;
	cv::minMaxLoc(bb0.row(1), &minVal, &maxVal, &minLoc, &maxLoc);
	hull(1, 0) = minVal;
	cv::minMaxLoc(bb0.row(2), &minVal, &maxVal, &minLoc, &maxLoc);
	hull(2, 0) = maxVal;
	cv::minMaxLoc(bb0.row(3), &minVal, &maxVal, &minLoc, &maxLoc);
	hull(3, 0) = maxVal;
}

void Detector::getPattern(const ImagePairType& img, const cv::Mat_<double>& bb, const unsigned patchSize, cv::Mat_<double>& pattern, bool flip)
{
	LOG("Detector::getPattern");
	//function pattern = tldGetPattern(img,bb,patchsize,flip)
	//% get patch under bounding box (bb), normalize it size, reshape to a column
	//% vector and normalize to zero mean and unit variance (ZMUV)
	//
	//% initialize output variable
	//nBB = size(bb,2);
	//pattern = zeros(prod(patchsize),nBB);
	//if ~exist('flip','var')
	//    flip= 0;
	//end
	//
	//% for every bounding box
	//for i = 1:nBB
	//    
	//    % sample patch
	//    patch = img_patch(img.input,bb(:,i));
	//    
	//    % flip if needed
	//    if flip
	//        patch = fliplr(patch);
	//    end
	//    
	//    % normalize size to 'patchsize' and nomalize intensities to ZMUV
	//    pattern(:,i) = tldPatch2Pattern(patch,patchsize);
	//end
	//printMatrix(bb, "bb");
	unsigned nBB = (unsigned) bb.cols;
	pattern.create(patchSize*patchSize, nBB);
	pattern = cv::Mat_<double>::zeros(pattern.size());
	for (unsigned c=0; c < nBB; ++c)
	{
		cv::Mat_<unsigned char> patch;
		imagePatch(*img.first, bb.col(c), patch, m_p_par_init);
		//printMatrix(patch, "patch");

		if (flip)
			cv::flip(patch, patch, 1);

		cv::Mat_<double> patCol = pattern.col(c);
		patch2Pattern(patch, patchSize, patCol);
	}
}

// Returns a new image that is a cropped version (rectangular cut-out)
// of the original image.
IplImage* cropImage(const IplImage *img, const CvRect region)
{
	LOG("Detector::cropImage");
	IplImage *imageCropped;
	CvSize size;

	if (img->width <= 0 || img->height <= 0
		|| region.width <= 0 || region.height <= 0) {
		//cerr << "ERROR in cropImage(): invalid dimensions." << endl;
		exit(1);
	}

	if (img->depth != IPL_DEPTH_8U) {
		//cerr << "ERROR in cropImage(): image depth is not 8." << endl;
		exit(1);
	}

	// Set the desired region of interest.
	cvSetImageROI((IplImage*)img, region);
	// Copy region of interest into a new iplImage and return it.
	size.width = region.width;
	size.height = region.height;
	imageCropped = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageCropped);	// Copy just the region.

	return imageCropped;
}

// Creates a new image copy that is of a desired size. The aspect ratio will
// be kept constant if 'keepAspectRatio' is true, by cropping undesired parts
// so that only pixels of the original image are shown, instead of adding
// extra blank space.
// Remember to free the new image later.
IplImage* Detector::resizeImage(IplImage *origImg, int newWidth, int newHeight, bool keepAspectRatio)
{
	LOG("Detector::resizeImage");

	// Affected by cvResize bug
	IplImage *outImg = 0;
        int origWidth = 0;
        int origHeight = 0;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0
		|| origWidth <= 0 || origHeight <= 0) {
		//cerr << "ERROR: Bad desired image size of " << newWidth
		//	<< "x" << newHeight << " in resizeImage().\n";
		exit(1);
	}

	if (keepAspectRatio) {
		// Resize the image without changing its aspect ratio,
		// by cropping off the edges and enlarging the middle section.
		CvRect r;
		// input aspect ratio
		float origAspect = (origWidth / (float)origHeight);
		// output aspect ratio
		float newAspect = (newWidth / (float)newHeight);
		// crop width to be origHeight * newAspect
		if (origAspect > newAspect) {
			int tw = (origHeight * newWidth) / newHeight;
			r = cvRect((origWidth - tw)/2, 0, tw, origHeight);
		}
		else {	// crop height to be origWidth / newAspect
			int th = (origWidth * newHeight) / newWidth;
			r = cvRect(0, (origHeight - th)/2, origWidth, th);
		}
		IplImage *croppedImg = cropImage(origImg, r);

		// Call this function again, with the new aspect ratio image.
		// Will do a scaled image resize with the correct aspect ratio.
		outImg = resizeImage(croppedImg, newWidth, newHeight, false);
		cvReleaseImage( &croppedImg );

	}
	else {

		// Scale the image to the new dimensions,
		// even if the aspect ratio will be changed.
		outImg = cvCreateImage(cvSize(newWidth, newHeight),
			origImg->depth, origImg->nChannels);
		if (newWidth > origImg->width && newHeight > origImg->height) {
			// Make the image larger
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_LINEAR: good at enlarging.
			// CV_INTER_CUBIC: good at enlarging.
			cvResize(origImg, outImg, CV_INTER_CUBIC);
		}
		else {
			// Make the image smaller
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_AREA: good at shrinking (decimation) only.
			cvResize(origImg, outImg, CV_INTER_AREA);
		}

	}

	return outImg;
}

void Detector::patch2Pattern(const cv::Mat_<unsigned char>& patch, const unsigned patchSize, cv::Mat_<double>& pattern)
{
	LOG("Detector::patch2Pattern");
	//function pattern = tldPatch2Pattern(patch,patchsize)
	//
	//patch   = imresize(patch,patchsize); % 'bilinear' is faster
	CvSize imageSize;
	imageSize.width = patch.cols;
	imageSize.height = patch.rows;
	IplImage* image = cvCreateImageHeader(imageSize, 8, 1);
	image->imageData = (char *) patch.data;
	image->imageDataOrigin = image->imageData;
	image->nChannels = 1;
	IplImage* resizedImage = resizeImage(image, 15, 15);
	resizedImage->nChannels = 1;
	cvReleaseImageHeader(&image);
	cv::Mat_<uchar> patch_ = (cv::Mat) resizedImage;
	//printMatrix(patch_, "patch_");
	cvReleaseImageHeader(&resizedImage);
	//loadMatrix(patch_, "resizedPatch.txt");
	//printMatrix<uchar>(patch_, "patch_");
	//static int initPatch = 0;
	//if (initPatch == 0)
	//{
	//	//loadMatrix(patch_, "patch.txt");
	//	//printMatrix<uchar>(patch_, "patch_");
	//	initPatch++;
	//}

	//pattern = double(patch(:));
	//pattern = pattern - mean(pattern);
	std::vector<double> patchVector;
	pattern.create(patchSize*patchSize, 1);
	
	for (int c=0; c < patch_.cols; ++c)
	{
		for (int r=0; r < patch_.rows; ++r)
		{
			pattern((r+(patch_.rows*c)), 0) = (double) patch_.at<uchar>(r,c);
			patchVector.push_back(patch_.at<uchar>(r,c));
		}
	}
	double mean = std::accumulate(patchVector.begin(), patchVector.end(), 0.0) / patchVector.size();
	//printMatrix(pattern, "pattern");
	pattern = pattern - mean;
	//printMatrix(pattern, "pattern-mean");
}

template <class T>
bool Detector::randperm(int n, cv::Mat_<T>& outMat)
{
	LOG("Detector::randperm");
	if (n <= 0) return false;
	outMat.create(1, n);
	for (int c=0; c < n; ++c)
		outMat(0, c) = c;
	std::random_shuffle(outMat.begin(), outMat.end());
	return true;
}

template <class T, class U>
void convolve1D(const cv::Mat_<T>& iVec, const cv::Mat_<U>& kVec, cv::Mat_<U>& oRow)
{
	int iC = iVec.cols;
	int kC = kVec.cols;

	int ic0 = 0;
	int ic1 = 0;
	int kc0 = kC - 1;
	int kc1 = kC - 1;
	for (int oc = 0; oc < (iC + kC - 1); ++oc)
	{
		ic0 = (oc < kC) ? 0 : (ic0 + 1);
		kc1 = (oc < iC) ? (kC - 1) : (kc1 - 1);

		int kc = kc0;
		int ic = ic0;
		for (int p=0; p < (ic1 - ic0 + 1); p++)
		{
			oRow(0,oc) = oRow(0,oc) + (iVec(0,ic++) * kVec(0,kc++));
		}

		if (ic1 < (iC - 1)) ic1++;
		if (kc0 > 0) kc0--;
	}
    return;
}

template <class T, class U>
void convolve2D(const cv::Mat_<T>& inMat, const cv::Mat_<U>& kernel, cv::Mat_<U>& outMat)
{
	int iR = inMat.rows;
	int iC = inMat.cols;
	int kR = kernel.rows;
	int kC = kernel.cols;

	outMat = cv::Mat_<U>::zeros(iR + kR - 1, iC + kC - 1);

	int kr = kR - 1;
	int ir = 0;

	int ir0 = 0;
	int ir1 = 0;
	int kr0 = kR - 1;
	int kr1 = kR - 1;
	for (int ori = 0; ori < (iR + kR - 1); ++ori)
	{
		ir0 = (ori < kR) ? 0 : (ir0 + 1);
		kr1 = (ori < iR) ? (kR - 1) : (kr1 - 1);

		kr = kr0;
		ir = ir0;
		for (int p=0; p < (ir1 - ir0 + 1); p++)
		{
			convolve1D<T, U>(inMat.row(ir++), kernel.row(kr++), outMat.row(ori));
		}

		if (ir1 < (iR - 1)) ir1++;
		if (kr0 > 0) kr0--;
	}

	//printMatrix<U>(outMat, "outMat");
    return;
}

IplImage* Detector::blur(IplImage& image, double sigma)
{
	IplImage *blurred = cvCreateImage(cvGetSize(&image), image.depth, image.nChannels);
	cvSmooth(&image, blurred, CV_GAUSSIAN, 0, 0, sigma, sigma);
	return blurred;	
}

void Detector::blur(IplImage& image, cv::Mat_<int>& outMat, double sigma)
{
	LOG("Detector::blur");

#ifdef USE_CVSMOOTH_BLUR
	IplImage *blurred = cvCreateImage(cvGetSize(&image), image.depth, image.nChannels);
	cvSmooth(&image, blurred, CV_GAUSSIAN, 0, 0, sigma, sigma);
	outMat = blurred;
	cvReleaseImage(&blurred);
#else
	//csize = 6*sigma;
	float csize = 6 * sigma;
	//
	//shift = (csize - 1)/2;
	int shift = (csize - 1)/2;
	//
	//h = FSPECIAL('gaussian',csize,sigma);
	if (m_gaussMat.empty())
	{
		//cv::Mat_<double> gaussMat;
		gaussMatrix<float>(csize, sigma, m_gaussMat);
		//printMatrix<double>(gaussMat, "gaussMat");
	}

	cv::SVD svd;
	cv::Mat_<float> S;
	cv::Mat_<float> U;
	cv::Mat_<float> V;
	svd.compute(m_gaussMat, S, U , V, cv::SVD::FULL_UV);
	//printMatrix<double>(S, "S");
	//printMatrix<double>(U, "U");
	//printMatrix<double>(V, "V");
	//cv::Mat_<double> g;
	//g = U * cv::Mat::diag(S) * V;
	//printMatrix<double>(g, "g");

	cv::Mat_<float> v;
	U.col(0).copyTo(v);
	v = v * sqrt(S(0,0));
	//printMatrix<double>(v, "v");

	cv::Mat_<float> h;
	V.row(0).copyTo(h);
	h = h * sqrt(S(0,0));
	//printMatrix<double>(h, "h");

	//M = conv2(M,h);
	cv::Mat_<float> tmpMat1;
	cv::Mat_<float> tmpMat2;
	cv::Mat_<int> inMat(&image);
	convolve2D<unsigned, float>(inMat, v, tmpMat1);
	//printMatrix<double>(tmpMat1, "tmpMat1");
	convolve2D<float, float>(tmpMat1, h, tmpMat2);
	//printMatrix<double>(tmpMat2, "tmpMat2");
	//convolve2D<unsigned, double>(inMat, gaussMat, tmpMat);

	//M = M(1+shift:end-shift,1+shift:end-shift);
	//M = uint8(M);
	outMat.create(inMat.rows, inMat.cols);
	for (int r = 0; r < outMat.rows; ++r)
		for (int c = 0; c < outMat.cols; ++c)
			outMat(r,c) = (unsigned) tmpMat2(r+shift+1, c+shift+1);

	//printMatrix<unsigned>(outMat, "outMat");
#endif
	return;
}

void Detector::tldTrainNN(const cv::Mat_<double>& pEx, const cv::Mat_<double>& nEx)
{
	LOG("Detector::tldTrainNN");
	//nP = size(pEx,2); % get the number of positive example 
	//nN = size(nEx,2); % get the number of negative examples
	int nP = pEx.cols;
	int nN = nEx.cols;

	//x = [pEx,nEx];
	//printMatrix(pEx, "pEx");
	//printMatrix(nEx, "nEx");
	cv::Mat_<double> x(pEx.rows, nP + nN);
	cv::Mat range = x.colRange(0,nP);
	pEx.colRange(0, nP).copyTo(range);
	//printMatrix(x, "x");
	range = x.colRange(nP, x.cols);
	nEx.colRange(0, nN).copyTo(range);
	//printMatrix(x, "x");

	//y = [ones(1,nP), zeros(1,nN)];
	cv::Mat_<double> y(1, nP + nN);
	cv::Mat_<double> ones = cv::Mat_<double>::ones(1, nP);
	cv::Mat_<double> zeros = cv::Mat_<double>::zeros(1, nN);
	range = y.colRange(0, nP);
	ones.colRange(0, nP).copyTo(range);
	range = y.colRange(nP, nP + nN);
	zeros.colRange(0, nN).copyTo(range);
	//printMatrix(y, "y");

	//% Permutate the order of examples
	//idx = randperm(nP+nN); %
	cv::Mat_<int> idx;
	randperm<int>(nP+nN, idx);
	//printMatrix(idx, "idx");

	//if ~isempty(pEx)
	//    x   = [pEx(:,1) x(:,idx)]; % always add the first positive patch as the first (important in initialization)
	//    y   = [1 y(:,idx)];
	//end
	if ((pEx.rows != 0) && (pEx.cols != 0))
	{
		cv::Mat_<double> x_(x.rows, x.cols + 1);
		Mat col = x_.col(0);
		pEx.col(0).copyTo(col);
		//printMatrix(x_, "x_");
		for (int i=0; i < idx.cols; ++i)
		{
			col = x_.col(i+1);
			x.col(idx(0,i)).copyTo(col);
		}
		//printMatrix(x_, "x_");
		x.create(x_.rows, x_.cols);
		x_.copyTo(x);
		//printMatrix(x, "x");

		cv::Mat_<double> y_(1, y.cols + 1);
		y_(0,0) = 1;
		for (int i=0; i < idx.cols; ++i)
		{
			col = y_.col(i+1);
			y.col(idx(0,i)).copyTo(col);
		}
		//printMatrix(y_, "y_");
		y.create(y_.rows, y_.cols);
		y_.copyTo(y);
		//printMatrix(y, "y");
	}
  
	//for k = 1:1 % Bootstrap // not used?
	//   for i = 1:length(y)
	for (int i=0; i != y.cols; ++i)
	{
		//     [conf1,dummy1,isin] = tldNN(x(:,i),tld); % measure Relative similarity
		cv::MatExpr r;
		//double conf1;
		//double dummy1;
		cv::Mat_<double> conf1;
		cv::Mat_<double> dummy1;
		cv::Mat_<double> isin;
		tldNN(x.col(i), conf1, dummy1, isin);
		//printMatrix(isin, "isin");
		//printMatrix(conf1, "conf1");
   
		// % Positive
		// if y(i) == 1 && conf1 <= tld.model.thr_nn % 0.65
		if ((y(0,i) == 1) && (conf1(0,0) <= m_model_thr_fern))
		{
			// if isnan(isin(2))
			//   tld.pex = x(:,i);
			//   continue;
			// end
			if (isnan(isin(1,0)))
			{
				m_pex.create(x.rows, 1);
				x.col(i).copyTo(m_pex);
				continue;
			}

			// tld.pex = [tld.pex(:,1:isin(2)) x(:,i) tld.pex(:,isin(2)+1:end)]; % add to model
			//
			// a = tld.pex(:,1:isin(2));
            // b = x(:,i);
            // c = tld.pex(:,isin(2)+1:end);
			// tld.pex = [a b c]
			//
			//printMatrix(m_pex, "m_pex");
			cv::Mat_<double> a(x.rows, isin(1, 0) + 1);
			m_pex.colRange(0, isin(1, 0) + 1).copyTo(a);
			//printMatrix(a, "a");

			cv::Mat_<double> b(x.rows, 1);
			Mat col = b.col(0);
			x.col(i).copyTo(col);
			//printMatrix(b, "b");

			cv::Mat_<double> c(0,0);
			if (m_pex.cols >= (isin(1, 0) + 1))
			{
				c.create(x.rows, m_pex.cols - isin(1, 0));
				m_pex.colRange(isin(1, 0), m_pex.cols).copyTo(c);
			}
			//printMatrix(c, "c");

			m_pex.create(x.rows, a.cols + b.cols + c.cols);
			if (a.cols > 0)
			{
				for (int c=0; c < a.cols; c++)
				{
					Mat col = m_pex.col(c);
					a.col(c).copyTo(col);
				}
			}
			//printMatrix(m_pex, "m_pex");
			if (b.cols > 0)
			{
				for (int c=0; c < b.cols; c++)
				{
					Mat col = m_pex.col(c + a.cols);
					b.col(c).copyTo(col);
				}
			}
			//printMatrix(m_pex, "m_pex");
			if (c.cols > 0)
			{
				for (int ci=0; ci < c.cols; ci++)
				{
					Mat col = m_pex.col(ci + a.cols + b.cols);
					c.col(ci).copyTo(col);
				}
			}
			//printMatrix(m_pex, "m_pex");
		}
     
		// % Negative
		// if y(i) == 0 && conf1 > 0.5
		//     tld.nex = [tld.nex x(:,i)];
		// end
		if ((y(0,i) == 0) && (conf1(0,0) > 0.5))
		{
			Mat col;
			cv::Mat_<double> nex(x.rows, m_nex.cols + 1);
			for (int c=0; c < m_nex.cols; ++c)
			{
				col = nex.col(c);
				m_nex.col(c).copyTo(col);
			}
			col = nex.col(m_nex.cols);
			x.col(i).copyTo(col);
			m_nex.create(nex.rows, nex.cols);
			nex.copyTo(m_nex);
			//printMatrix(m_nex, "m_nex");
		}
	}
	//end
}

bool any(const cv::Mat_<bool>& mat)
{
	LOG("Detector::any");
	for (int r=0; r < mat.rows; ++r)
		for (int c=0; c < mat.cols; ++c)
			if (mat(r,c)) return true;
	return false;
}

void Detector::tldNN(const cv::Mat_<double>& x, cv::Mat_<double>& conf1, cv::Mat_<double>& conf2, cv::Mat_<double>& isin)
{
	LOG("Detector::tldNN");
	//% 'conf1' ... full model (Relative Similarity)
	//% 'conf2' ... validated part of model (Conservative Similarity)
	//% 'isnin' ... inside positive ball, id positive ball, inside negative ball
	//
	//isin = nan(3,size(x,2));
	isin = cv::Mat_<double>(3, x.cols, nan); // second column is NaN flag
	//
	//if isempty(tld.pex) % IF positive examples in the model are not defined THEN everything is negative
	//    conf1 = zeros(1,size(x,2));
	//    conf2 = zeros(1,size(x,2));
	//    return;
	//end
	if (m_pex.empty())
	{
		conf1 = cv::Mat_<double>::zeros(1, x.cols);
		conf2 = cv::Mat_<double>::zeros(1, x.cols);
		return;
	}

	//
	//if isempty(tld.nex) % IF negative examples in the model are not defined THEN everything is positive
	//    conf1 = ones(1,size(x,2));
	//    conf2 = ones(1,size(x,2));
	//    return;
	//end
	if (m_nex.empty())
	{
		conf1 = cv::Mat_<double>::ones(1, x.cols);
		conf2 = cv::Mat_<double>::ones(1, x.cols);
		return;
	}

	//conf1 = nan(1,size(x,2));
	//conf2 = nan(1,size(x,2));
	conf1 = cv::Mat_<double>(1, x.cols, nan);
	conf2 = cv::Mat_<double>(1, x.cols, nan);

	//for i = 1:size(x,2) % fore every patch that is tested
	//printMatrix(x, "x");
	for (int i=0; i < x.cols; ++i)
	{
		//    
		//    nccP = distance(x(:,i),tld.pex,1); % measure NCC to positive examples
		//    nccN = distance(x(:,i),tld.nex,1); % measure NCC to negative examples
		cv::Mat_<double> nccP;
		cv::Mat_<double> patch;
		x.col(i).copyTo(patch);
		//printMatrix(patch, "patch");
		distance(patch, m_pex, 1, nccP);
		//printMatrix<double>(nccP, "nccP");
		cv::Mat_<double> nccN;
		distance(patch, m_nex, 1, nccN);
		//printMatrix<double>(nccN, "nccN");

		//    
		//    % set isin
		//    if any(nccP > tld.model.ncc_thesame), isin(1,i) = 1;  end % IF the query patch is highly correlated with any positive patch in the model THEN it is considered to be one of them;
		cv::Mat e = (Mat) (nccP > m_ncc_thesame);
		//printMatrix<bool>(e, "e");
		if (any(e)) isin(0, i) = 1;
		//printMatrix(isin, "isin");

		//    [dummy1,isin(2,i)] = max(nccP); % get the index of the maximal correlated positive patch
		int index = 0;
		double max_nccP;
		maxMat<double>(nccP, index, max_nccP);
		isin(1, i) = index;
		//printMatrix(isin, "isin");

		//    if any(nccN > tld.model.ncc_thesame), isin(3,i) = 1;  end % IF the query patch is highly correlated with any negative patch in the model THEN it is considered to be one of them
		e = (Mat) (nccN > m_ncc_thesame);
		if (any(e)) isin(2,i) = 1;
		//printMatrix(isin, "isin");

		//    % measure Relative Similarity
		//    dN = 1 - max(nccN);
		//    dP = 1 - max(nccP);
		//    conf1(i) = dN / (dN + dP);
		double max_nccN;
		maxMat<double>(nccN, index, max_nccN);
		double dN = 1 - max_nccN;
		double dP = 1 - max_nccP;
		conf1(0, i) = dN / (dN + dP);
		//printMatrix(conf1, "conf1");

		//
		//    % measure Conservative Similarity
		//    maxP = max(nccP(1:ceil(tld.model.valid*size(tld.pex,2))));
		//    dP = 1 - maxP;
		//    conf2(i) = dN / (dN + dP);
		double maxP;
		double h = ceil(m_model_valid * m_pex.cols);
		if (h > 1) maxMat<double>(nccP.colRange(0, h - 1), index, maxP);
		else  maxMat<double>(nccP.col(0), index, maxP);
		dP = 1 - maxP;
		conf2(0, i) = dN / (dN + dP);
		//printMatrix(conf2, "conf2");
	}
	//end
}

// correlation
double ccorr(double *f1,double *f2,int numDim)
{
	LOG("Detector::ccorr");
	double f = 0;
	for (int i = 0; i<numDim; i++) {
		f += f1[i]*f2[i];
	}
	return f;
}

// correlation normalized
double ccorr_normed(double *f1, int off1, double *f2, int off2, int numDim)
{
	LOG("Detector::ccorr_normed");
	double corr = 0;
	double norm1 = 0;
	double norm2 = 0;

	for (int i = 0; i<numDim; i++) {
		corr += f1[i*off1]*f2[i*off2];
		norm1 += f1[i*off1]*f1[i*off1];
		norm2 += f2[i*off2]*f2[i*off2];
	}
	// normalization to <0,1>
	return (corr / sqrt(norm1*norm2) + 1) / 2.0;
}

// euclidean distance
double euclidean(double *f1, int off1, double *f2, int off2, int numDim = 2)
{
	LOG("Detector::euclidean");
	double sum = 0;
	for (int i = 0; i < numDim; i++)
		sum += (f1[i*off1]-f2[i*off2])*(f1[i*off1]-f2[i*off2]);
	return sqrt(sum);
}

void Detector::distance(const cv::Mat_<double>& x1, const cv::Mat_<double>& x2, int flag, cv::Mat_<double>& resp)
{
	LOG("Detector::distance");
	double *x1Data = (double*) x1.data; int N1 = x1.cols; int M1 = x1.rows;
	double *x2Data = (double*) x2.data; int N2 = x2.cols; //int M2 = x2.rows;

	//printMatrix(x1, "x1");
	//printMatrix(x2, "x2");

	resp.create(N1, N2);
	resp = cv::Mat_<double>::zeros(N1, N2);
	double *respData = (double *) resp.data;
	//printMatrix(resp, "resp");

	switch (flag)
	{
	case 1 :
		for (int i = 0; i < N2; i++) {
			for (int ii = 0; ii < N1; ii++) {
				*respData++ = ccorr_normed(x1Data+ii,N1,x2Data+i,N2,M1);
			}
		}

		return;
	case 2 :

		for (int i = 0; i < N2; i++) {
			for (int ii = 0; ii < N1; ii++) {
				*respData++ = euclidean(x1Data+ii,N1,x2Data+i,N2,M1);
			}
		}

		return;
	}
}

void Detector::display(int runtime, int frameNumber)
{
	LOG("Detector::display");
	if (!runtime)
	{
		if (m_plot_bb)
		{
			onUpdateRect(
					TLDRect(m_source_bb(0,0) , m_source_bb(1,0),
					m_source_bb(2,0) - m_source_bb(0,0),
					m_source_bb(3,0) - m_source_bb(1,0))
					);
		}
	
		//% Pex
		//	if tld.plot.pex == 1
		//		img = embedPex(img,tld);
		//end
		if (m_plot_pex)
			embedPex();

		//% Nex
		//	if tld.plot.nex == 1
		//		img = embedNex(img,tld);
		//end
		if (m_plot_nex)
			embedNex();

        //% Info
        //string = [num2str(tld.control.maxbbox) '/' num2str(tld.nGrid)];
        //text(10,200,string,'color','white');
		char infoString[255];
		sprintf(infoString, "%d / %d", m_control_maxbbox, m_nGrid);
		onUpdateInfo(infoString);
	}
	else
	{
		//    tld = varargin{2};
		//    i = tld.source.idx(varargin{3});
		int i = frameNumber;
		//    
		//    
		//    h = get(gca,'Children'); delete(h(1:end-1));
		//    if nargin == 4, text(10,10,varargin{4},'color','white'); end
		//    
		//    % Draw image
		//    img = tld.img{i}.input;
		//    [H,W] = size(img);
		//    
		//    % Pex
		//    if tld.plot.pex == 1
		//        img = embedPex(img,tld);
		//    end
		if (m_plot_pex)
			embedPex();
		//    
		//    % Nex
		//    if tld.plot.nex == 1
		//        img = embedNex(img,tld);
		//    end
		if (m_plot_nex)
			embedNex();
		//    
		//    % Target
		//    Size = 100;
		//    if tld.plot.target && ~isnan(tld.bb(1,i))
		//        bb = bb_rescale_relative(tld.bb(:,i),4*[1 1]);
		//        patch = img_patch(tld.img{i}.input,bb);
		//        patch = imresize(patch,[Size Size]);
		//        img(1:Size,1:Size) = patch;
		//    end
		//    %rectangle('Position',[0 0 400 400],'edgecolor','k');
		//    
		//    % Replace
		//    if tld.plot.replace && ~isnan(tld.bb(1,i))
		//        bb = round(tld.bb(:,i));
		//        if bb_isin(bb,size(tld.img{i}.input))
		//            patch = imresize(tld.target,[bb(4)-bb(2)+1, bb(3)-bb(1)+1]);
		//            img(bb(2):bb(4),bb(1):bb(3)) = patch;
		//        end
		//    end
		//    
		//    
		//    set(tld.handle,'cdata',img); hold on;
		//    
		//    % Draw Detections
		//    if tld.plot.dt && ~isempty(tld.dt{i})
		if (m_plot_dt && (m_dt.find(frameNumber) != m_dt.end()))
		{
			//        % Fern detections
			//        %bb = tld.dt{i}.bb(:,:); if tld.plot.confidence, bb = [bb; tld.dt{i}.conf1]; end
			//        %bb_draw(bb,'edgecolor',0.5*[1 1 1]);
			//        % NN detections
			//        %idx = tld.dt{i}.conf1 > tld.model.thr_nn;
			//        %bb = tld.dt{i}.bb(:,idx); if tld.plot.confidence, bb = [bb; tld.dt{i}.conf1(idx)]; end
			//        %bb_draw(bb,'edgecolor','red');
			//        cp = bb_center(tld.dt{i}.bb);
			cv::Mat_<double> cp;
			bbCenter(m_dt[i].bb, cp);
			//        if ~isempty(cp)
			//            plot(cp(1,:),cp(2,:),'.','color',0.25*[1 1 1]);
			//        end
			if (! cp.empty())
			{
				vector<TLDPoint> points;
				for (int c=0; c < cp.cols; ++c)
				{
					points.push_back((TLDPoint){cp(0,c), cp(1,c)});
				}
				if (!points.empty())
					onUpdatePoints(points, TLD_GREEN);
			}

			//        idx = tld.dt{i}.conf1 > tld.model.thr_nn; 1 x N
			cv::Mat_<double>& conf1 = m_dt[i].conf1;
			std::vector<int> idx;
			for (int c=0; c < conf1.cols; ++c)
			{
				if (conf1(0,c) > m_model_thr_nn)
				{
					idx.push_back(c);
				}
			}
			//        cp = bb_center(tld.dt{i}.bb(:,idx));
			cv::Mat_<double>& bb = m_dt[i].bb;
			cv::Mat_<double> bbCol(bb.rows, idx.size());
			for (unsigned c=0; c < idx.size(); ++c)
			{
				Mat col = bbCol.col(c);
				m_dt[i].bb.col(idx.at(c)).copyTo(col);
			}
			bbCenter(bbCol, cp);
			//        if ~isempty(cp)
			//        plot(cp(1,:),cp(2,:),'.','color','red');
			//        end
			if (! cp.empty())
			{
				vector<TLDPoint> points;
				for (int c=0; c < cp.cols; ++c)
				{
					points.push_back((TLDPoint){cp(0,c), cp(1,c)});
				}
				if (!points.empty())
					onUpdatePoints(points, TLD_RED);
			}
		}

		//    % Draw Track
		//    linewidth = 2; if tld.valid(i) == 1, linewidth = 4; end;
		//    color = 'y'; %if tld.conf(i) > tld.model.thr_nn_valid, color = 'b'; end
		//    
		//    bb = tld.bb(:,i);
		cv::Mat_<double>& bb = m_dt[i].bb;
		//    switch tld.plot.drawoutput
		switch (m_plot_drawoutput)
		{   
		//        case 1
		//            bb = bb_rescale_relative(bb_square(bb),[1.2 1.2]);
		//            if tld.plot.confidence, bb = [bb; tld.conf(i)]; end
		//            bb_draw(bb,'linewidth',linewidth,'edgecolor',color,'curvature',[1 1]);
		//        case 2
		//            cp = bb_center(bb);
		//            plot(cp(1),cp(2),'.r','markersize',20);
		//            if tld.plot.confidence, text(cp(1),cp(2),num2str(tld.conf(i))); end
		//            %bb_draw(bb,'linewidth',linewidth,'edgecolor',color,'curvature',[1 1]);
		//        case 3
		//            if tld.plot.confidence, bb = [bb; tld.conf(i)]; end
		//            bb_draw(bb,'linewidth',linewidth,'edgecolor',color,'curvature',[0 0]);
		case 3:
			{
				if (!bb.empty())
				{
					if (m_plot_bb)
					{
						onUpdateRect(
								TLDRect(bb(0,0) , bb(1,0),
								bb(2,0) - bb(0,0),
								bb(3,0) - bb(1,0))
								);
					}

					if (m_plot_confidence)
					{
						onUpdateConfidence(m_conf(0,i));
					}
				}
				else
				{
					if (m_plot_bb)
						onUpdateRect(TLDRect(0,0,0,0));
					if (m_plot_confidence)
						onUpdateConfidence(0);
				}
			}
			break;
		}
		//    
		//    % Info
		//    
		//    %string = ['#' num2str(i) ', fps:' num2str(1/toc,2) ', ' num2str(tld.control.maxbbox) '/' num2str(tld.nGrid) ', Fern: ' num2str(tld.model.thr_fern,4) ', NN: ' num2str(tld.model.thr_nn,3) '/' num2str(tld.model.thr_nn_valid,3)];
		//    string = ['#' num2str(i) ', fps:' num2str(1/toc,3) ', ' num2str(tld.control.maxbbox) '/' num2str(tld.nGrid)];
		//    text(10,H-10,string,'color','white','backgroundcolor','k');
		//    %if tld.control.update_detector
		//    %    text(10,H-30,'Learning','color','white','backgroundcolor','k');
		//    %end
		//    
		//    if tld.trackerfailure(i)==1
		//        text(10,H-30,'Tracker failure','color','white','backgroundcolor','k');
		//    end
		//    
		//    % Draw
		//    if tld.plot.draw
		//       plot(tld.draw(1,:),tld.draw(2,:),'r','linewidth',2);    
		//    end
		//    
		//    if tld.plot.pts
		//        plot(tld.xFJ(1,:),tld.xFJ(2,:),'.');
		//    end
		if (m_plot_pts)
		{
			vector<TLDPoint> points;
			for (int c=0; c < m_xFJ.cols; ++c)
			{
				points.push_back((TLDPoint){m_xFJ(0,c), m_xFJ(1,c)});
			}
			if (!points.empty())
				onUpdatePoints(points, TLD_BLUE);
		}
		//    
		//       
		//    if tld.plot.help
		//        
		//        k = 12;
		//        text(10,1*k,'n ... shows negative examples in online model (default on)');
		//        text(10,2*k,'p ... shows positive examples in online model (default on)');
		//        text(10,3*k,'i ... initialization of different target');
		//        text(10,4*k,'c ... show confidence score (default on)');
		//        text(10,5*k,'o ... show output as circle/dot/no output (default circle)');
		//        text(10,6*k,'d ... show detections (default on)');
		//        text(10,7*k,'t ... show target in top left corner (default off)');
		//        text(10,8*k,'r ... replace target with first patch (default off)');
		//        text(10,9*k,'# ... draw trajectory of target (default off)');
		//        text(10,10*k,'1 ... mode without learning');
		//        text(10,11*k,'2 ... mode with learning');
		//        text(10,12*k,'q ... finish application');
		//        text(10,13*k,'space ... save current image');
		//        
		//    end
		//    
		//    
		//    drawnow;
		//    tic;
		//    
		//    % Save
		//    %if tld.plot.save == 1
		//    %    img = getframe(1);
		//    %    imwrite(img.cdata,[tld.name '/' num2str(i,'%05d') '.png']);
		//    %end
	}
}

void Detector::embedPex(void)
{
	LOG("Detector::embedPex");
	// Rescale = tld.plot.patch_rescale;
	// no_row = floor(tld.imgsize(1)/(Rescale*tld.model.patchsize(1)));
	// no_col = floor(tld.imgsize(2)/(Rescale*tld.model.patchsize(2)));
	// tmp = size(tld.pex,2);
	// if size(tld.pex,2) > no_row*no_col
	//    pex = mat2img(tld.pex(:,1:no_row*no_col),no_row);
	// else
	//    pex = mat2img(tld.pex,no_row);
	// end
	// pex = uint8(imresize(255*pex,Rescale));
	// [pH,pW] = size(pex);
	// img(1:pH,end-pW+1:end) = pex;
	int rescale = m_plot_patch_rescale;
	int no_row = (int) floor((double) m_imgSize(0,0)/(rescale*m_model_patchsize));
	int no_col = (int) floor((double) m_imgSize(0,1)/(rescale*m_model_patchsize));
	cv::Mat_<double> pex;
	if (m_pex.cols > m_pexCol)
	{
		if (m_pex.cols > (no_row * no_col))
		{
			mat2img(m_pex.colRange(m_pexCol, no_row*no_col - 1), no_row, pex);
			m_pexCol = no_row*no_col;
		}
		else
		{
			mat2img(m_pex, no_row, pex);
			m_pexCol = m_pex.cols;
		}
	}
	else return;

	//printMatrix(pex, "pex");
	pex = pex * 255;
	//printMatrix(pex, "pex");

	int xoff = 0;
	int yoff = 0;
	int numImages = m_pex.cols;
	int ph = rescale*m_model_patchsize;
	int pw = rescale*m_model_patchsize;
	int imagesPerRow = pex.cols / pw;
	int imagesPerCol = pex.rows / ph;
	onPexCount(numImages);
	for (int i=0; i < numImages; ++i)
	{
		cv::Mat_<double> img(rescale*m_model_patchsize, rescale*m_model_patchsize);

		if ((i % imagesPerRow) != 0) yoff += pw;
		else yoff = 0;

		if ((i > 1) && (imagesPerCol > 1) && ((i % imagesPerRow) == 0))
			xoff += ph;

		for (int r=0; r < pw; ++r)
		{
			for (int c=0; c < ph; ++c)
			{
				img(r, c) = pex(r+xoff, c+yoff);
			}
		}

		//printMatrix(img, "img");
		emitPex(img);
	}
}

void Detector::embedNex(void)
{
	LOG("Detector::embedNex");
	//function img = embedNex(img,tld)
	//Rescale = tld.plot.patch_rescale;
	//no_row = floor(tld.imgsize(1)/(Rescale*tld.model.patchsize(1)));
	//no_col = floor(tld.imgsize(2)/(Rescale*tld.model.patchsize(2)));
	//if size(tld.nex,2) > no_row*no_col
	//    nex = mat2img(tld.nex(:,1:no_row*no_col),no_row);
	//else
	//    nex = mat2img(tld.nex,no_row);
	//end
	//nex = uint8(imresize(255*nex,Rescale));
	//[pH,pW] = size(nex);
	//img(1:pH,1:pW)= nex;
	//
	//end
	int rescale = m_plot_patch_rescale;
	int no_row = (int) floor((double) m_imgSize(0,0)/(rescale*m_model_patchsize));
	int no_col = (int) floor((double) m_imgSize(0,1)/(rescale*m_model_patchsize));
	cv::Mat_<double> nex;
	//printMatrix(m_nex, "m_nex");
	if (m_nex.cols > m_nexCol)
	{
		if (m_nex.cols > (no_row * no_col))
		{
			mat2img(m_nex.colRange(m_nexCol, no_row*no_col - 1), no_row, nex);
			m_nexCol = no_row*no_col;
		}
		else
		{
			mat2img(m_nex, no_row, nex);
			m_nexCol = m_nex.cols;
		}
	}
	else return;

	//printMatrix(nex, "nex");
	nex = nex * 255;
	//printMatrix(nex, "nex");

	int xoff = 0;
	int yoff = 0;
	int numImages = m_nex.cols;
	int ph = rescale*m_model_patchsize;
	int pw = rescale*m_model_patchsize;
	int imagesPerRow = nex.cols / pw;
	int imagesPerCol = nex.rows / ph;
	onNexCount(numImages);
	for (int i=0; i < numImages; ++i)
	{
		cv::Mat_<double> img(rescale*m_model_patchsize, rescale*m_model_patchsize);

		if ((i % imagesPerRow) != 0) yoff += pw;
		else yoff = 0;

		if ((i > 1) && (imagesPerCol > 1) && ((i % imagesPerRow) == 0))
			xoff += ph;

		for (int r=0; r < pw; ++r)
		{
			for (int c=0; c < ph; ++c)
			{
				img(r, c) = nex(r+xoff, c+yoff);
			}
		}

		//printMatrix(img, "img");
		emitNex(img);
	}
}

void Detector::mat2img(const cv::Mat_<double> data, int no_row, cv::Mat_<double>& imgMat)
{
	LOG("Detector::mat2img");
	//function img = mat2img(data,no_row)
	//% 'Data' contains square images stored in columns.
	//
	//if ~exist('RATIO','var')
	//    RATIO = 1;
	//end
	//printMatrix(data, "data");
	//const int ratio = 1;

	//if isempty(data)
	//    img = []; return;
	//end
	if (data.empty())
		return;

	//[M,N] = size(data);
	int M = data.rows;
	int N = data.cols;

	//
	//sM = sqrt(M);
	//if ceil(sM) ~= sM
	//    img = []; disp('Wrong input!'); return;
	//end
	double sM = sqrt((double) M);
	if (ceil(sM) != sM)
	{
		emitStatus("Wrong input!");
		return;
	}

	//W     = sM;
	//H     = sM;
	int W = sM;
	int H = sM;

	//if no_row > N, no_row = N; end
	//no_col = ceil(N/no_row);
	//%no_row = ceil(N/no_col);
	//img    = zeros(no_row*H,no_col*W);
	if (no_row > N)
		no_row = N;
	int no_col = (int) ceil((double) N/no_row);
	imgMat = cv::Mat_<double>::zeros(no_row*H, no_col*W);

	//for i = 1:N
	for (int i=0; i < N; ++i)
	{
		//    [I, J] = ind2sub([no_row, no_col], i);
		cv::Mat_<int> subs(1, 2);
		cv::Mat_<double> siz(1, 2);
		siz(0,0) = no_row;
		siz(0,1) = no_col;
		//printMatrix(siz, "siz");
		ind2sub(siz, subs, i+1);
		//printMatrix(subs, "subs");

		//    row = 1+(I-1)*H:(I-1)*H+H;
		//    col = 1+(J-1)*W:(J-1)*W+W;
		vector<int> row;
		colon<int>(1+(subs(0,0)-1)*H, 1, (subs(0,0)-1)*H+H, row);
		vector<int> col;
		colon<int>(1+(subs(0,1)-1)*W, 1, (subs(0,1)-1)*W+W, col);

		//    img0 = reshape(data(:,i),sM,sM);
		cv::Mat_<double> dataCol(data.rows, 1);
		data.col(i).copyTo(dataCol);
		cv::Mat_<double> img0 = dataCol.reshape(sM);
		//printMatrix(img0, "img0");
		//    img0 = (img0 - min(img0(:))) / (max(img0(:)) - min(img0(:)));
		double minVal, maxVal;
		cv::minMaxIdx(img0, &minVal, &maxVal);
		img0 = (img0 - minVal) / (maxVal - minVal);
		//printMatrix(img0, "img0");
		//    img(row, col) = img0;
		for (unsigned r=0; r < row.size(); ++r)
			for (unsigned c=0; c< col.size(); ++c)
				imgMat(row[r]-1, col[c]-1) = img0(r, c);
		//printMatrix(imgMat, "imgMat");
		//end
	}
	imgMat = imgMat.t();
	//printMatrix(imgMat, "imgMat");
}

void Detector::ind2sub(const cv::Mat_<double>& siz, cv::Mat_<int>& subs, int ndx)
{
	LOG("Detector::ind2sub");
	//nout = max(nargout,1);
	int nout = max(subs.rows*subs.cols, 1);
	//siz = double(siz);

	//if length(siz)<=nout,
	//  siz = [siz ones(1,nout-length(siz))];
	//else
	//  siz = [siz(1:nout-1) prod(siz(nout:end))];
	//end
	cv::Mat_<double> tmp;
	if (siz.cols <= nout)
	{
		tmp.create(1, nout);
		for (int c=0; c < siz.cols; ++c)
		{
			Mat col = tmp.col(c);
			siz.col(c).copyTo(col);
		}
		for (int i=0; i < (nout - siz.cols); i++)
			tmp(0, i + siz.cols) = 1;
	}
	else
	{
		double prod = 1;
		tmp.create(1, nout);
		for (int c=0; c < nout-1; ++c)
		{
			Mat col = tmp.col(c);
			siz.col(c).copyTo(col);
		}
		for (int i = nout; i < siz.cols; ++i)
			prod *= siz(0, i - 1);
		tmp(0, nout - 1) = prod;
	}
	//printMatrix(tmp, "tmp");

	//n = length(siz);
	int n = tmp.cols;

	//k = [1 cumprod(siz(1:end-1))];
	cv::Mat_<double> tmp2 = siz.colRange(0, siz.cols - 1);
	//printMatrix(tmp2, "tmp2");
	cv::Mat_<double> sizProd(1, tmp2.cols);
	cumprod<double>(tmp2, sizProd);
	//printMatrix(sizProd, "sizProd");
	cv::Mat_<double> k(1, siz.cols);
	k(0, 0) = 1;
	for (int c=0; c < sizProd.cols; ++c)
	{
		Mat col = k.col(c+1);
		sizProd.col(c).copyTo(col);
	}
	//printMatrix(k, "k");

	//for i = n:-1:1,
	//  vi = rem(ndx-1, k(i)) + 1;         
	//  vj = (ndx - vi)/k(i) + 1; 
	//  varargout{i} = vj; 
	//  ndx = vi;     
	//end
	for (int i=n; i >= 1; i--)
	{
		int vi = 1 + ((ndx-1) % ((int) k(0, i-1))); // AKI rem and mod differe if signes nek
		double vj = 1 + ((ndx - vi)/k(0, i-1));
		subs(0, i-1) = vj;
		ndx = vi;
	}
}

template<class T>
void Detector::cumprod(const cv::Mat_<T>& inMat, cv::Mat_<T>& outMat)
{
	LOG("Detector::cumprod");
	if (inMat.rows == 1)
		for (int r=0; r < inMat.rows; ++r)
			for (int c=0; c < inMat.cols; ++c)
				if ((r == 0) && (c == 0)) outMat(r, c) = inMat(r, c);
				else outMat(r, c) = inMat(r, c-1) * inMat(r, c);

}

void Detector::bb_rescale_relative(cv::Mat_<double> BB, cv::Mat_<double> s, cv::Mat_<double>&/* bb */)
{
	LOG("Detector::bb_rescale_relative");
	//function BB = bb_rescale_relative(BB,s)
	//% Change
	//BB = BB(1:4);
	cv::Mat_<double> bbTmp = BB.colRange(0, 3);
	//if length(s) == 1
	//	s = s*[1 1];
	//end
	//if isempty(BB), BB = []; return; end

	//s1 = 0.5*(s(1)-1)*bb_width(BB);
	//s2 = 0.5*(s(2)-1)*bb_height(BB);
	//BB = BB + [-s1; -s2; s1; s2];
}

void Detector::process(IplImage* f)
{
	if (!f)
		return;

	IplImage& frame = *f;

	//I = tld.source.idx(i); % get current index
	int I = ++m_frameNumber;

	//tld.img{I} = img_get(tld.source,I); % grab frame from camera / load image
	cv::Mat mono(&frame);
	cv::cvtColor(mono, mono, CV_BGR2GRAY);
	IplImage monoImage = mono;
	CvSize imageSize = cvSize(monoImage.width, monoImage.height);
	cvReleaseImage(&m_image);
	m_image = cvCreateImage(imageSize, monoImage.depth, monoImage.nChannels);
	cvCopy(&monoImage, m_image);
	cv::Mat_<int> imageMat(m_image);
	//printMatrix(imageMat, "imageMat");


	//IplImage* m_imageBlur = blur(*m_image, 2);
	//cv::Mat_<int> blurMat(m_imageBlur);
	//cvReleaseImage(&m_imageBlur);
	cv::Mat_<int> blurMat;
	blur(*m_image, blurMat, 2);
	ImagePairType img;
	img.first = new cv::Mat_<int>(imageMat);
	img.second = new cv::Mat_<int>(blurMat);
	m_img.push_back(img);
	//
	//% TRACKER  ----------------------------------------------------------------
	//
	//[tBB tConf tValid tld] = tldTracking(tld,tld.bb(:,I-1),I-1,I); % frame-to-frame tracking (MedianFlow)

	//printMatrix(m_bb, "m_bb");
	//cv::Mat_<double> bb;
	//m_bb.col(I-1).copyTo(bb);
	//printMatrix(bb, "bb");
	cv::Mat_<double> tBB;
	double tConf;
	double tValid = 0;
	tracking(m_bb.col(I-1), I-1, I, tBB, tConf, tValid);
	//printMatrix(tBB, "tBB");

	//% DETECTOR ----------------------------------------------------------------
	//
	//[dBB dConf tld] = tldDetection(tld,I); % detect appearances by cascaded detector (variance filter -> ensemble classifier -> nearest neightbour)
	//tldDetection(I, dBB, dConf);
	cv::Mat_<double> dBB;
	cv::Mat_<double> dConf;
	detection(I, dBB, dConf);
	//printMatrix(dBB, "dBB");
	//printMatrix(dConf, "dConf");

	//% INTEGRATOR --------------------------------------------------------------
	//
	//DT = 1; if isempty(dBB), DT = 0; end % is detector defined?
	bool DT = !dBB.empty();
	//TR = 1; if isempty(tBB), TR = 0; end % is tracker defined?
	bool TR = !tBB.empty();
	//
	//if TR % if tracker is defined
	if (TR)
	{
		//    % copy tracker's result
		//    tld.bb(:,I)  = tBB;
		//printMatrix(m_bb, "m_bb");
		Mat col = m_bb.col(I);
		tBB.col(0).copyTo(col);
		//printMatrix(m_bb, "m_bb");
		//    tld.conf(I)  = tConf;
		m_conf(0,I) = tConf;
		//    tld.size(I)  = 1;
		m_size(0,I) = 1;
		//    tld.valid(I) = tValid;
		m_valid(0,I) = tValid;
		//
		//    if DT % if detections are also defined
		if (DT)
		{
			//        [cBB,cConf,cSize] = bb_cluster_confidence(dBB,dConf); % cluster detections
			cv::Mat_<double> cBB;
			cv::Mat_<double> cConf;
			cv::Mat_<double> cSize;
			bb_cluster_confidence(dBB, dConf, cBB, cConf, cSize);
			//printMatrix(cBB, "cBB");
			//printMatrix(cConf, "cConf");
			//printMatrix(cSize, "cSize");

			//        id = bb_overlap(tld.bb(:,I),cBB) < 0.5 & cConf > tld.conf(I); % get indexes of all clusters that are far from tracker and are more confident then the tracker
			cv::Mat_<double> overlap;
			bb_overlap(m_bb.col(I), cBB, overlap);
			//printMatrix(overlap, "overlap");
			//printMatrix(m_conf, "m_conf");
			cv::Mat_<int> id(overlap.rows, overlap.cols);
			for (int r=0; r < overlap.rows; ++r)
			{
				if ((overlap(r,0) < 0.5) && (m_conf(0,I) < cConf(r,0)))
					id(r,0) = 1;
				else
					id(r,0) = 0;
			}
			//printMatrix(id, "id");
			int sum = cv::sum(id)(0);

			//        if sum(id) == 1 % if there is ONE such a cluster, re-initialize the tracker
			if (sum == 1)
			{
				for (int r=0; r < id.rows; ++r)
				{
					// tld.bb(:,I)  = cBB(:,id);
					// tld.conf(I)  = cConf(:,id);
					// tld.size(I)  = cSize(:,id);
					// tld.valid(I) = 0;
					//printMatrix(cBB, "cBB");
					//printMatrix(id, "id");
					if (id(r,0))
					{
						Mat col = m_bb.col(I);
						cBB.col(r).copyTo(col);
						col = m_conf.col(I);
						cConf.col(r).copyTo(col);
						col = m_size.col(I);
						cSize.col(r).copyTo(col);
					}
					m_valid(0,I) = 0;
				}
			}
			else // else % otherwise adjust the tracker's trajectory
			{
				// idTr = bb_overlap(tBB,tld.dt{I}.bb) > 0.7;  % get indexes of close detections
				cv::Mat_<double> overlap;
				bb_overlap(tBB, m_dt[I].bb, overlap);
				//printMatrix(overlap, "overlap");
				cv::Mat_<int> idTr(overlap.rows, overlap.cols);
				for (int c=0; c < overlap.cols; ++c)
				{
					if (overlap(0,c) > 0.7) idTr(0,c) = 1;
					else idTr(0,c) = 0;
				}
				//printMatrix(idTr, "idTr");
				// tld.bb(:,I) = mean([repmat(tBB,1,10) tld.dt{I}.bb(:,idTr)],2);  % weighted average trackers trajectory with the close detections
				cv::Mat_<double> rtBB;
				repmat(tBB, 1, rtBB, 10);
				//printMatrix(rtBB, "rtBB");
				cv::Mat_<double> bbC(m_dt[I].bb.rows, cv::sum(idTr)(0));
				int idx = 0;
				for (int c=0; c < idTr.cols; ++c)
				{
					if (idTr(0,c))
					{
						Mat col = bbC.col(idx++);
						m_dt[I].bb.col(c).copyTo(col);
					}
				}
				//printMatrix(bbC, "bbC");
				cv::Mat_<double> aBB(rtBB.rows, rtBB.cols + bbC.cols);
				for (int c=0; c < rtBB.cols; ++c)
				{
					Mat col = aBB.col(c);
					rtBB.col(c).copyTo(col);
				}
				for (int c=0; c < bbC.cols; ++c)
				{
					Mat col = aBB.col(rtBB.cols+c);
					bbC.col(c).copyTo(col);
				}
				//printMatrix(aBB, "aBB");
				for (int r=0; r < aBB.rows; ++r)
					m_bb(r, I) = cv::mean(aBB.row(r))(0);
				//printMatrix(m_bb, "m_bb");
			}
		}
	}
	//else % if tracker is not defined
	else
	{
		//    if DT % and detector is defined
		if (DT)
		{
			//        [cBB,cConf,cSize] = bb_cluster_confidence(dBB,dConf); % cluster detections
			cv::Mat_<double> cBB;
			cv::Mat_<double> cConf;
			cv::Mat_<double> cSize;
			bb_cluster_confidence(dBB, dConf, cBB, cConf, cSize);
			//        if length(cConf) == 1 % and if there is just a single cluster, re-initalize the tracker
			if (cConf.cols == 1)
			{
				//            tld.bb(:,I)  = cBB;
				Mat col = m_bb.col(I);
				cBB.copyTo(col);
				//            tld.conf(I)  = cConf;
				m_conf(0, I) = cConf(0,0);
				//            tld.size(I)  = cSize;
				m_size(0,I) = cSize(0,0);
				//            tld.valid(I) = 0;
				m_valid(0,I) = 0;

			}
		}
	}
	//% LEARNING ----------------------------------------------------------------
	//
	//if tld.control.update_detector && tld.valid(I) == 1
	//    tld = tldLearning(tld,I);
	//end
	if (m_control_update_detector && (m_valid(0, I) == 1))
		learning(I);

	//% display drawing: get center of bounding box and save it to a drawn line
	//if ~isnan(tld.bb(1,I))
	if (!isnan(m_bb(0,I)))
	{
		//    tld.draw(:,end+1) = bb_center(tld.bb(:,I));
		cv::Mat_<double> center;
		bbCenter(m_bb.col(I), center);
		m_draw.push_back(center);
		//    if tld.plot.draw == 0, tld.draw(:,end) = nan; end
		if (m_plot_draw)
		{
			cv::Mat_<double> center(2,1,nan);
			m_draw.push_back(center);
		}
	}
	else
	{
		//    tld.draw = zeros(2,0);
		m_draw.clear();
	}

	//if tld.control.drop_img && I > 2, tld.img{I-1} = {}; end % forget previous image
	if (m_control_drop_img && (I > 2))
	{
		delete m_img[I-1].first; m_img[I-1].first = NULL;
		delete m_img[I-1].second; m_img[I-1].second = NULL;

	}

	//	tldDisplay(1,tld,i); % display results on frame i
	display(1, I);
}

void Detector::tracking(const cv::Mat_<double>& BB1, int I, int J, cv::Mat_<double>& BB2, double& Conf, double& Valid)
{
	LOG("Detector::tracking");
	//function [BB2 Conf Valid tld] = tldTracking(tld,BB1,I,J)
	//% Estimates motion of bounding box BB1 from frame I to frame J

	//% initialize output variables
	//BB2    = []; % estimated bounding 
	//Conf   = []; % confidence of prediction
	//Valid  = 0;  % is the predicted bounding box valid? if yes, learning will take place ...
	BB2.create(0,0);
	Conf = 0;
	Valid = 0;

	//if isempty(BB1) || ~bb_isdef(BB1), return; end % exit function if BB1 is not defined
	if ((BB1.cols == 0) || (BB1.rows == 0) || !bbIsDef(BB1))
		return;

	//% estimate BB2
	//xFI    = bb_points(BB1,10,10,5); % generate 10x10 grid of points within BB1 with margin 5 px
	cv::Mat_<double> xFI;
	//printMatrix(BB1, "BB1");
	bbPoints(BB1, 10.0, 10.0, 5.0, xFI);
	//printMatrix(xFI, "xFI"); // 2 x 100

	//xFJ    = lk(2,tld.img{I}.input,tld.img{J}.input,xFI,xFI); % track all points by Lucas-Kanade tracker from frame I to frame J, estimate Forward-Backward error, and NCC for each point
	cv::Mat_<double> xFJ;
	//printMatrix(*m_img.at(I).first, "m_img.at(I).first");
	//printMatrix(*m_img.at(J).first, "m_img.at(J).first");
	m_lk->track(*m_img.at(I).first, *m_img.at(J).first, xFI, xFI, xFJ);
	//printMatrix(xFJ, "xFJ"); // 4 x 100

	//medFB  = median2(xFJ(3,:)); % get median of Forward-Backward error
	cv::Mat_<double> xFKr2 = xFJ.row(2);
	std::vector<double> xFKr2V(xFKr2.begin(), xFKr2.end());
	double medFB = median2<double>(xFKr2V);
	//medNCC = median2(xFJ(4,:)); % get median for NCC
	cv::Mat_<double> xFKr4 = xFJ.row(3);
	std::vector<double> xFKr4V(xFKr4.begin(), xFKr4.end());
	double medNCC = median2<double>(xFKr4V);
	//idxF   = xFJ(3,:) <= medFB & xFJ(4,:)>= medNCC; % get indexes of reliable points
	int idxFt = 0; // Number of true elements in idxF
	cv::Mat_<bool> idxF(1, xFJ.cols, false);
	for (int c=0; c < xFJ.cols; ++c)
	{
		if ((xFJ.row(2)(c) <= medFB) && (xFJ.row(3)(c) >= medNCC))
		{
			idxF(c) = true;
			idxFt++;
		}
	}
	//printMatrix(idxF, "idxF");

	//BB2    = bb_predict(BB1,xFI(:,idxF),xFJ(1:2,idxF)); % estimate BB2 using the reliable points only
	int idx=0;
	cv::Mat_<double> tmp1(xFI.rows, idxFt);
	cv::Mat_<double> tmp2(2, idxFt);
	for (int c=0; c < idxF.cols; ++c)
	{
		if (idxF(0, c))
		{
			Mat col = tmp1.col(idx);
			xFI.col(c).copyTo(col);
			tmp2(0, idx) = xFJ(0, c);
			tmp2(1, idx) = xFJ(1, c);
			idx++;
		}
	}
	//printMatrix(tmp1, "tmp1");
	//printMatrix(tmp2, "tmp2");
	cv::Mat_<double> shift;
	bbPredict(BB1, tmp1, tmp2, BB2, shift);
	//printMatrix(BB2, "BB2");
	//printMatrix(shift, "shift");

	//tld.xFJ = xFJ(:,idxF); % save selected points (only for display purposes)
	cv::Mat_<double> tmp = xFJ;
	m_xFJ.create(tmp.rows, idx);
	int i=0;
	for (int c=0; c < idxF.cols; ++c)
	{
		if (idxF(0,c))
		{
			Mat col = m_xFJ.col(i++);
			tmp.col(c).copyTo(col);
		}
	}
	//printMatrix(m_xFJ, "m_xFJ");

	//% detect failures
	//if ~bb_isdef(BB2) || bb_isout(BB2,tld.imgsize), BB2 = []; return; end % bounding box out of image
	if (!bbIsDef<double>(BB2) || bbIsOut<double>(BB2, m_imgSize))
	{
		BB2.create(0,0);
		return;
	}
	//if tld.control.maxbbox > 0 && medFB > 10, BB2 = []; return; end  % too unstable predictions
	if ((m_control_maxbbox > 0) && (medFB > 10))
	{
		BB2.create(0,0);
		return;
	}

	//% estimate confidence and validity
	//patchJ   = tldGetPattern(tld.img{J},BB2,tld.model.patchsize); % sample patch in current image
	cv::Mat_<double> patchJ;
	getPattern(m_img.at(J), BB2, m_model_patchsize, patchJ);
	//printMatrix(patchJ, "patchJ");
	//[dummy1,Conf] = tldNN(patchJ,tld); % estimate its Conservative Similarity (considering 50% of positive patches only)
	cv::Mat_<double> dummy1;
	cv::Mat_<double> confMat;
	cv::Mat_<double> isin;
	tldNN(patchJ, dummy1, confMat, isin);
	//printMatrix(confMat, "confMat");
	//printMatrix(isin, "isin");
	Conf = confMat(0,0);

	//% Validity
	//Valid    = tld.valid(I); % copy validity from previous frame
	Valid = m_valid(0, I);
	//if Conf > tld.model.thr_nn_valid, Valid = 1; end % tracker is inside the 'core'
	if (Conf > m_model_thr_nn_valid)
		Valid = 1;
}

template<class T>
bool Detector::bbIsOut(const cv::Mat_<T> bb, const cv::Mat_<T>& imsize)
{
	LOG("Detector::bbIsOut");
	//function idx_out = bb_isout(bb,imsize)
	//
	//idx_out = bb(1,:) > imsize(2) | ...
	//bb(2,:) > imsize(1) | ...
	//bb(3,:) < 1 | ...
	//bb(4,:) < 1;
	//printMatrix(bb, "bb");
	//printMatrix(imsize, "imsize");
	bool idx_out = ((bb(0,0) > imsize(0,1)) || (bb(1,0) > imsize(0,0)) ||
		(bb(2,0) < 1) || (bb(3,0) < 1));
	return idx_out;
}

template<class T>
void Detector::isFinite(const cv::Mat_<T>& bb, cv::Mat_<int>& outMat)
{
	LOG("Detector::isFinite");
	outMat.create(bb.rows, bb.cols);
	outMat = cv::Mat_<bool>::ones(bb.rows, bb.cols);
	for (int r=0; r < bb.rows; ++r)
		for (int c=0; c < bb.cols; ++c)
			if (isnan(bb(r,c)) || (bb(r,c) == inf) || (bb(r,c) == -inf))
				outMat(r,c) = 0;

}

template<class T>
bool Detector::bbIsDef(const cv::Mat_<T>& bb)
{
	LOG("Detector::bbIsDef");
	//function id = bb_isdef(bb)
	//% Info
	//id = isfinite(bb(1,:));
	cv::Mat_<int> outMat;
	isFinite<T>(bb.row(1), outMat);
	for (int r=0; r < outMat.rows; ++r)
		for (int c=0; c < outMat.cols; ++c)
			if (outMat(r,c) == 0)
				return false;
	return true;
}

void Detector::bbPoints(const cv::Mat_<double>& bb, double numM, double numN, double margin, cv::Mat_<double>& pt)
{
	LOG("Detector::bbPoints");
	//function pt = bb_points(bb,numM,numN,margin)
	//% Generates numM x numN points on BBox.

	//bb(1:2) = bb(1:2)+margin;
	//bb(3:4) = bb(3:4)-margin;
	//printMatrix(bb, "bb");
	cv::Mat_<double> BB(4,1);
	BB(0,0) = bb(0,0) + margin;
	BB(1,0) = bb(1,0) + margin;
	BB(2,0) = bb(2,0) - margin;
	BB(3,0) = bb(3,0) - margin;
	//printMatrix(BB, "BB");

	//if (numM == 1 && numN ==1)
	//	pt = bb_center(bb);
	//	return;
	//end
	if ((numM == 1) && (numN == 1))
	{
		bbCenter(BB, pt);
		//printMatrix(pt, "pt");
		return;
	}

	//if (numM == 1 && numN > 1)
	//	c = bb_center(bb);
	//	stepW = (bb(3)-bb(1)) / (numN - 1);
	//	pt = ntuples(bb(1):stepW:bb(3),c(2));
	//	return;
	//end
	if ((numM == 1) && (numN > 1))
	{
		cv::Mat_<double> c;
		bbCenter(BB, c);
		//printMatrix(c, "c");
		int stepW = (BB(2,0) - BB(0,0)) / (numN - 1);
		std::vector<double> v;
		colon<double>(BB(0,0), stepW, BB(2,0), v);
		cv::Mat_<double> tmp(v, true);
		tmp = tmp.t();
		//printMatrix(tmp, "tmp");
		cv::Mat_<double> row = c.row(1);
		ntuples<cv::Mat_<double> >(tmp, row, pt);
		//printMatrix(pt, "pt");
		return;
	}

	//if (numM > 1 && numN == 1)
	//	c = bb_center(bb);
	//	stepH = (bb(4)-bb(2)) / (numM - 1);
	//	pt = ntuples(c(1),(bb(2):stepH:bb(4)));
	//	return;
	//end
	if ((numM > 1) && (numN == 1))
	{
		cv::Mat_<double> c;
		bbCenter(BB, c);
		//printMatrix(c, "c");
		int stepH = (BB(3,0) - BB(1,0)) / (numM - 1);
		std::vector<double> v;
		colon<double>(BB(1,0), stepH, BB(3,0), v);
		cv::Mat_<double> tmp(v, true);
		tmp = tmp.t();
		//printMatrix(tmp, "tmp");
		cv::Mat_<double> row = c.row(0);
		ntuples<cv::Mat_<double> >(row, tmp, pt);
		//printMatrix(pt, "pt");
		return;
	}
   
	//stepW = (bb(3)-bb(1)) / (numN - 1);
	//stepH = (bb(4)-bb(2)) / (numM - 1);
	//pt = ntuples(bb(1):stepW:bb(3),(bb(2):stepH:bb(4)));
	double stepW = (BB(2,0) - BB(0,0)) / (numN-1);
	double stepH = (BB(3,0) - BB(1,0)) / (numM-1);
	cv::Mat_<double> vwMat;
	colon<double>(BB(0,0), stepW, BB(2,0), vwMat);
	//printMatrix(vwMat, "vwMat");
	cv::Mat_<double> vhMat;
	colon<double>(BB(1,0), stepH, BB(3,0), vhMat);
	//printMatrix(vhMat, "vhMat");
	ntuples<cv::Mat_<double> >(vwMat, vhMat, pt);
	//printMatrix(pt, "pt");
}

void Detector::bbPredict(const cv::Mat_<double>& BB0, const cv::Mat_<double>& pt0, const cv::Mat_<double>& pt1, cv::Mat_<double>& BB1, cv::Mat_<double>& shift)
{
	LOG("Detector::bbPredict");
	//function [BB1 shift] = bb_predict(BB0,pt0,pt1)
	//
	//of  = pt1 - pt0
	//printMatrix(BB0, "BB0");
	//printMatrix(pt0, "pt0");
	//printMatrix(pt1, "pt1");
	cv::Mat_<double> of = pt1 - pt0;
	//printMatrix(of, "of");
	//dx  = median(of(1,:));
	double dx = median<double>(of.row(0));
	//dy  = median(of(2,:));
	double dy = median<double>(of.row(1));
	//
	//d1  = pdist(pt0','euclidean');
	cv::Mat_<double> d1;
	pdist(pt0, d1);
	//printMatrix(d1, "d1");
	//d2  = pdist(pt1','euclidean');
	cv::Mat_<double> d2;
	pdist(pt1, d2);
	//printMatrix(d2, "d2");
	//s   = median(d2./d1);
	double s;
	cv::Mat_<double> tmp = d2 / d1;
	s = median<double>((vector<double>) tmp);
	//s1  = 0.5*(s-1)*bb_width(BB0);
	double s1 = 0.5*(s-1)*bbWidth(BB0);
	//s2  = 0.5*(s-1)*bb_height(BB0);
	double s2  = 0.5*(s-1)*bbHeight(BB0);
	//BB1  = [BB0(1)-s1; BB0(2)-s2; BB0(3)+s1; BB0(4)+s2] + [dx; dy; dx; dy];
	BB1.create(4,1);
	BB1(0,0) = BB0(0,0) - s1 + dx;
	BB1(1,0) = BB0(1,0) - s2 + dy;
	BB1(2,0) = BB0(2,0) + s1 + dx;
	BB1(3,0) = BB0(3,0) + s2 + dy;
	//printMatrix(BB1, "BB1");

	//shift = [s1; s2];
	shift.create(2,1);
	shift(0,0) = s1;
	shift(1,0) = s2;
	//printMatrix(shift, "shift");
}

void Detector::pdist(const cv::Mat_<double>& inMat, cv::Mat_<double>& outMat)
{
	LOG("Detector::pdist");
	double* inData = (double*) inMat.data;
	outMat.create(1, inMat.cols*(inMat.cols-1)/2);
	double *outData = (double*) outMat.data;
	for (int c=0; c < inMat.cols-1; ++c)
	{
		for (int c1=c+1; c1 < inMat.cols; ++c1)
		{
			*outData++ = euclidean(inData+c, inMat.cols, inData+c1, inMat.cols);
		}
	}
}

void Detector::detection(int I, cv::Mat_<double>& BB,  cv::Mat_<double>& Conf)
{
	//function [BB Conf tld] = tldDetection(tld,I)
	//% scanns the image(I) with a sliding window, returns a list of bounding
	//% boxes and their confidences that match the object description
	//
	//BB        = [];
	BB.create(0,0);
	//Conf      = [];
	Conf.create(0,0);
	//dt        = struct('bb',[],'idx',[],'conf1',[],'conf2',[],'isin',nan(3,1),'patt',[],'patch',[]);
	DtType dt;
	dt.isin.create(3,1);
	dt.isin(0,0) = nan;
	dt.isin(1,0) = nan;
	dt.isin(2,0) = nan;
	//img  = tld.img{I};
	const ImagePairType& img = m_img.at(I);
	//printMatrix(*m_img.at(I).first, "m_img.at(I).first");

	//fern(4,img,tld.control.maxbbox,tld.var,tld.tmp.conf,tld.tmp.patt); % evaluates Ensemble Classifier: saves sum of posteriors to 'tld.tmp.conf', saves measured codes to 'tld.tmp.patt', does not considers patches with variance < tmd.var
	m_fern->detect(img, m_control_maxbbox, m_var, m_tmpConf, m_tmpPatt);
	//printMatrix(m_tmpConf, "m_tmpConf");
	//printMatrix(m_tmpPatt, "m_tmpPatt");

	//idx_dt = find(tld.tmp.conf > tld.model.num_trees*tld.model.thr_fern); % get indexes of bounding boxes that passed throu the Ensemble Classifier
	double thr = m_num_trees * m_model_thr_fern;
	std::vector<double> idx_dt;
	std::vector<std::pair<int, double> > val_dt;
	for (int c=0; c < m_tmpConf.cols; ++c)
	{
		if (m_tmpConf(0,c) > thr)
		{
			idx_dt.push_back(c);
			val_dt.push_back(std::make_pair(c, m_tmpConf(0,c)));
		}
	}

	//if length(idx_dt) > 2 % speedup: if there are more than 100 detections, pict 100 of the most confident only
	//    [dummy1,sIdx] = sort(tld.tmp.conf(idx_dt),'descend');
	//    idx_dt = idx_dt(sIdx(1:2));
	//end
	if (idx_dt.size() > 2)
	{
		std::sort(val_dt.begin(), val_dt.end(), pairValueSortPredicate<std::pair<int, double> >);
		val_dt.resize(2);
		idx_dt.clear();
		std::vector<std::pair<int, double> >::reverse_iterator vit;
		for (vit = val_dt.rbegin(); vit != val_dt.rend(); ++vit)
			idx_dt.push_back((*vit).first);
	}

	//num_dt = length(idx_dt); % get the number detected bounding boxes so-far
	int num_dt = (int) idx_dt.size();
	//if num_dt == 0, tld.dt{I} = dt; return; end % if nothing detected, return
	if (num_dt == 0)
	{
		m_dt[I] = dt;
		return;
	}

	//% initialize detection structure
	//dt.bb     = tld.grid(1:4,idx_dt); % bounding boxes
	//dt.patt   = tld.tmp.patt(:,idx_dt); % corresponding codes of the Ensemble Classifier
	//dt.idx    = find(idx_dt); % indexes of detected bounding boxes within the scanning grid
	//dt.conf1  = nan(1,num_dt); % Relative Similarity (for final nearest neighbour classifier)
	//dt.conf2  = nan(1,num_dt); % Conservative Similarity (for integration with tracker)
	//dt.isin   = nan(3,num_dt); % detected (isin=1) or rejected (isin=0) by nearest neighbour classifier
	//dt.patch  = nan(prod(tld.model.patchsize),num_dt); % Corresopnding patches
	dt.bb.create(4, num_dt);
	dt.patt.create(m_tmpPatt.rows, num_dt);
	dt.idx.create(1, num_dt);
	dt.conf1.create(1, num_dt);
	dt.conf2.create(1, num_dt);
	dt.isin.create(3, num_dt);
	dt.patch.create(m_model_patchsize*m_model_patchsize, num_dt);
	for (int c=0; c < num_dt; ++c)
	{
		int idx = idx_dt.at(c);
		dt.bb(0, c) = m_grid(0, idx);
		dt.bb(1, c) = m_grid(1, idx);
		dt.bb(2, c) = m_grid(2, idx);
		dt.bb(3, c) = m_grid(3, idx);
		Mat col = dt.patt.col(c);
		m_tmpPatt.col(idx).copyTo(col);
		if (idx > 0) dt.idx(c) = c;
		dt.conf1(0,c) = nan;
		dt.conf2(0,c) = nan;
		dt.isin(0,c) = nan;
		dt.isin(1,c) = nan;
		dt.isin(2,c) = nan;
		for (unsigned r=0; r < m_model_patchsize*m_model_patchsize; ++r)
			dt.patch(r,c) = nan;
	}
	//printMatrix(dt.bb, "dt.bb");
	//printMatrix(dt.patt, "dt.patt");
	
	//for i = 1:num_dt % for every remaining detection
	for (int i=0; i < num_dt; ++i)
	{
		//    ex   = tldGetPattern(img,dt.bb(:,i),tld.model.patchsize); % measure patch
		//    [conf1, conf2, isin] = tldNN(ex,tld); % evaluate nearest neighbour classifier
		cv::Mat_<double> ex;
		getPattern(img, dt.bb.col(i), m_model_patchsize, ex);
		//loadMatrix(ex, "ex.txt");
		//printMatrix(ex, "ex");
		cv::Mat_<double> conf1;
		cv::Mat_<double> conf2;
		cv::Mat_<double> isin;
		tldNN(ex, conf1, conf2, isin);
		//printMatrix(conf1, "conf1");
		//printMatrix(conf2, "conf2");
		//printMatrix(isin, "isin");

		//    % fill detection structure
		//    dt.conf1(i)   = conf1;
		//    dt.conf2(i)   = conf2;
		//    dt.isin(:,i)  = isin;
		//    dt.patch(:,i) = ex;
		dt.conf1(0,i) = conf1(0,0); 
		dt.conf2(0,i) = conf2(0,0);
		Mat col = dt.isin.col(i);
		isin.col(0).copyTo(col);
		col = dt.patch.col(i);
		ex.col(0).copyTo(col);
	}
	//printMatrix(dt.conf1, "dt.conf1");
	//printMatrix(dt.isin, "dt.isin");
	//printMatrix(dt.patch, "dt.patch");

	//idx = dt.conf1 > tld.model.thr_nn; % get all indexes that made it through the nearest neighbour
	int idxCount = 0;
	std::vector<bool> idx;
	for (int c=0; c < dt.conf1.cols; ++c)
	{
		if (dt.conf1(0,c) > m_model_thr_nn)
		{
			idx.push_back(true);
			idxCount++;
		}
		else idx.push_back(false);
	}

	//% output
	//BB    = dt.bb(:,idx); % bounding boxes
	//Conf  = dt.conf2(:,idx); % conservative confidences
	if (idxCount)
	{
		Conf.create(dt.conf2.rows, idxCount);
		BB.create(dt.bb.rows, idxCount);
		int colIdx = 0;
		for (int c = 0; c < idxCount; ++c)
		{
			if (idx.at(c))
			{
				Mat col = BB.col(colIdx);
				dt.bb.col(c).copyTo(col);
				col = Conf.col(colIdx++);
				dt.conf2.col(c).copyTo(col);
			}
		}
		//printMatrix(BB, "BB");
		//printMatrix(Conf, "Conf");
	}

	//tld.dt{I} = dt; % save the whole detection structure
	m_dt[I] = dt;
}

template<class T>
bool equalsCompPredicate(T i, T j)
{
  return (i==j);
}

template<class T>
void unique(const std::vector<T>& inv, std::vector<T>& outv)
{
	if (!inv.empty())
	{
		if (inv.size() == 1)
		{
			outv.push_back(inv.front());
			return;
		}
		std::list<T> l(inv.begin(), inv.end());
		l.sort();
		l.unique();
		outv.insert(outv.begin(), l.begin(), l.end());
	}
}

void Detector::bb_cluster_confidence(const cv::Mat_<double>& iBB, const cv::Mat_<double> iConf, cv::Mat_<double>& oBB, cv::Mat_<double>& oConf, cv::Mat_<double>& oSize)
{
	//function [oBB,oConf,oSize] = bb_cluster_confidence(iBB, iConf)
	//% Clusterering of tracker and detector responses
	//% First cluster returned corresponds to the tracker
	//
	//SPACE_THR = 0.5;
	double SPACE_THR = 0.5;
	//
	//oBB    = [];
	//oConf  = [];
	//oSize  = [];
	oBB.create(0,0);
	oConf.create(0,0);
	oSize.create(0,0);
	//
	//if isempty(iBB)
	//    return;
	//end
	if (iBB.empty())
		return;

	cv::Mat_<double> T;
	cv::Mat_<double> idx_cluster;
	//switch size(iBB,2)
	switch (iBB.cols)
	{
		//    case 0, T = [];
	case 0:
		T.create(0,0);
		break;
		//    case 1, T = 1;
	case 1:
		T.create(1,1);
		T(0,0) = 1;
		break;
		//    case 2
	case 2:
		//        T = ones(2,1);
		T = cv::Mat_<double>::ones(2,1);
		//        if bb_distance(iBB) > SPACE_THR, T(2) = 2; end
		if (bb_distance(iBB) > SPACE_THR)
			T(1,0) = 2;
		break;
	default:
#ifdef WIN32
#ifdef DEBUG
		// TBD. Cannpt get matlab version to drop in here.
		__asm __emit 0xF1
#endif
#endif
		//
		//    otherwise
		//        bbd = bb_distance(iBB);
		//double bbd = bb_distance(iBB);
		//double Z = linkage(bbd, "si");
		//z
		//        T = cluster(Z,'cutoff', SPACE_THR,'criterion','distance');
		break;
		//end
	}

	//idx_cluster  = unique(T);
	//num_clusters = length(idx_cluster);
	//printMatrix(T.col(0), "T.col(0)");
	std::vector<double> v;
	if (T.cols)
		::unique((std::vector<double>) T.col(0), v);
	if (!v.empty())
	{
		cv::Mat_<double> tmp(v, true);
		idx_cluster = tmp;
	}
	//printMatrix(idx_cluster, "idx_cluster");
	int num_clusters = (int) v.size();
	
	//oBB    = nan(4,num_clusters);
	//oConf  = nan(1,num_clusters);
	//oSize  = nan(1,num_clusters);
	oBB = cv::Mat(4, num_clusters, CV_64F, nan);
	oConf = cv::Mat(1, num_clusters, CV_64F, nan);
	oSize = cv::Mat(1, num_clusters, CV_64F, nan);

	//for i = 1:num_clusters
	for (int i=0; i < num_clusters; ++i)
	{
		//    idx = T == idx_cluster(i);
		//printMatrix(T, "T");
		//printMatrix(idx_cluster.col(0), "idx_cluster.col(i)");
		cv::Mat_<int> idx(T.rows, T.cols);
		for (int r=0; r < T.rows; ++r)
		{
			if (T(r,0) == idx_cluster(r,0)) idx(r,0) = 1; // TBR
			else idx(r,0) = 0;	
		}
		//printMatrix(idx, "idx");
   
		//    oBB(:,i)  = mean(iBB(1:4,idx),2);
		int nrows = 0;
		for (int r=0; r < idx.rows; ++r)
		{
			if (idx(r, 0))
				nrows++;
		}
		cv::Mat_<double> iBBc(4, nrows);
		int ir = 0;
		for (int r=0; r < idx.rows; ++r)
		{
			if (idx(r,0))
			{
				Mat col = iBBc.col(ir++);
				iBB.col(r).copyTo(col);
			}
		}
		//printMatrix(iBB, "iBB");
		//printMatrix(iBBc, "iBBc");

		cv::Mat_<double> meanMat;
		mean<double>(iBBc, 2, meanMat);
		//printMatrix(meanMat, "meanMat");
		Mat col = oBB.col(i);
		meanMat.col(0).copyTo(col);
		//printMatrix(oBB, "oBB");

		//    oConf(i)  = mean(iConf(idx));
		cv::Scalar m = cv::mean(iConf.col(i));
		oConf(0, i) = m(0);
		//printMatrix(oConf, "oConf");
		
		//    oSize(i)  = sum(idx);
		oSize(0,i) = cv::sum(idx)(0);
		//
		//end
	}
}


double Detector::bb_distance(const cv::Mat_<double>& bb1, const cv::Mat_<double>* bb2)
{
	//function d = bb_distance(bb1,bb2)
	//% Info
	//
	//switch nargin
	//    case 1 
	//        d = 1 - bb_overlap(bb1);
	//    case 2
	//        d = 1 - bb_overlap(bb1,bb2);
	//end
	cv::Mat_<double> dMat;
	if (!bb2)
		bb_overlap(bb1, dMat);
	else
		bb_overlap(bb1, *bb2, dMat);
	double d = 1 - dMat(0,0);
	return d;
}

void Detector::learning(int I)
{
	//function tld = tldLearning(tld,I)
	//
	//bb    = tld.bb(:,I); % current bounding box
	cv::Mat_<double> bb(m_bb.col(I));
	//printMatrix(bb, "bb");

	//img   = tld.img{I}; % current image
	const ImagePairType& img = m_img.at(I);

	//% Check consistency -------------------------------------------------------
	//
	//pPatt  = tldGetPattern(img,bb,tld.model.patchsize); % get current patch
	cv::Mat_<double> pPatt;
	getPattern(img, bb, m_model_patchsize, pPatt);
	//printMatrix(pPatt, "pPatt");

	//[pConf1,dummy1,pIsin] = tldNN(pPatt,tld); % measure similarity to model
	cv::Mat_<double> pConf1;
	cv::Mat_<double> dummy1;
	cv::Mat_<double> pIsin;
	tldNN(pPatt, pConf1, dummy1, pIsin);
	//printMatrix(pConf1, "pConf1");
	//printMatrix(pIsin, "pIsin");

	//if pConf1 < 0.5, disp('Fast change.'); tld.valid(I) = 0; return; end % too fast change of appearance
	if (pConf1(0,0) < 0.5)
	{
		emitStatus("Fast change.");
		m_valid(0,I) = 0;
		return;
	}

	//if var(pPatt) < tld.var, disp('Low variance.'); tld.valid(I) = 0; return; end % too low variance of the patch
	cv::Mat_<double> patVar;
	var(pPatt, patVar);
	//printMatrix(pPatt, "pPatt");
	//printMatrix(patVar, "patVar");
	if (patVar(0,0) < m_var)
	{
		emitStatus("Low variance.");
		m_valid(0,I) = 0;
		return;
	}

	//if pIsin(3) == 1, disp('In negative data.'); tld.valid(I) = 0; return; end % patch is in negative data
	if (pIsin(2,0) == 1)
	{
		emitStatus("In negative data.");
		m_valid(0,I) = 0;
		return;
	}

	//% Update ------------------------------------------------------------------
	//
	//% generate positive data
	//overlap  = bb_overlap(bb,tld.grid); % measure overlap of the current bounding box with the bounding boxes on the grid
	cv::Mat_<double> overlap;
	//printMatrix(bb, "bb");
	//printMatrix(m_grid, "m_grid");
	bb_overlap(bb, m_grid, overlap);
	//printMatrix(overlap, "overlap");

	//[pX,pEx] = tldGeneratePositiveData(tld,overlap,img,tld.p_par_update); % generate positive examples from all bounding boxes that are highly overlappipng with current bounding box
	cv::Mat_<double> pX;
	cv::Mat_<double> pEx;
	cv::Mat_<double> bbP;
	tldGeneratePositiveData(overlap, img, m_p_par_update, pX, pEx, bbP);
	//printMatrix(pX, "pX");
	//printMatrix(pEx, "pEx");
	//printMatrix(bbP, "bbP");

	//pY       = ones(1,size(pX,2)); % labels of the positive patches
	cv::Mat_<double> pY = cv::Mat_<double>::ones(1, pX.cols);

	//% generate negative data
	//idx      = overlap < tld.n_par.overlap & tld.tmp.conf >= 1; % get indexes of negative bounding boxes on the grid (bounding boxes on the grid that are far from current bounding box and which confidence was larger than 0)
	cv::Mat_<int> idx(1, overlap.cols, 0);
	for (int c=0; c < overlap.cols; ++c)
	{
		if ((overlap(0,c) < m_n_par.overlap) && (m_tmpConf(0,c) >= 1))
		{
			idx(0,c) = 1;
		}
	}
	//printMatrix(idx, "idx");

	//overlap  = bb_overlap(bb,tld.dt{I}.bb); % measure overlap of the current bounding box with detections
	//printMatrix(bb, "bb");
	//printMatrix(m_dt[I].bb, "m_dt[I].bb");
	bb_overlap(bb, m_dt[I].bb, overlap);
	//printMatrix(overlap, "overlap");

	//nEx      = tld.dt{I}.patch(:,overlap < tld.n_par.overlap); % get negative patches that are far from current bounding box
	std::list<int> oidx;
	for (int c=0; c < overlap.cols; ++c)
	{
		if (overlap(0,c) < m_n_par.overlap)
			oidx.push_back(c);
	}
	cv::Mat_<double> nEx(m_dt[I].patch.rows, 0);
	if (!oidx.empty())
	{
		int c=0;
		nEx.create(m_dt[I].patch.rows, oidx.size());
		for (std::list<int>::iterator oidxit = oidx.begin(); oidxit != oidx.end(); ++oidxit)
		{
			Mat col = nEx.col(c++);
			m_dt[I].patch.col(*oidxit).copyTo(col);
		}
	}
	//printMatrix(nEx, "nEx");

	//fern(2,[pX tld.tmp.patt(:,idx)],[pY zeros(1,sum(idx))],tld.model.thr_fern,2); % update the Ensemble Classifier (reuses the computation made by detector)
	cv::Mat_<double> x(pX.rows, pX.cols + cv::sum(idx)(0));
	for (int c =0; c < pX.cols; ++c)
	{
		Mat col = x.col(c);
		pX.col(c).copyTo(col);
	}
	int j=0;
	for (int c=0; c < idx.cols; ++c)
	{
		if (idx(0,c) == 1)
		{
			Mat col = x.col(pX.cols + j++);
			m_tmpPatt.col(c).copyTo(col);
		}
	}
	//printMatrix(x, "x");
	int sumIdx = sum(idx)(0);
	cv::Mat_<double> y(pY.rows, pY.cols + sumIdx, 0.0);
	for (int c =0; c < pY.cols; ++c)
	{
		Mat col = y.col(c);
		pY.col(c).copyTo(col);
	}
	//printMatrix(y, "y");
	m_fern->update(x, y, m_model_thr_fern, 2);

	//tld = tldTrainNN(pEx,nEx,tld); % update nearest neighbour
	//printMatrix(pEx, "pEx");
	//printMatrix(nEx, "nEx");
	tldTrainNN(pEx, nEx);
}

void Detector::signalBoundingBox(bool val)
{
	m_plot_bb = val;
}

void Detector::signalPoints(bool val)
{
	m_plot_pts = val;
}

void Detector::signalConfidence(bool val)
{
	m_plot_confidence = val;
}

void Detector::signalDetections(bool val)
{
	m_plot_dt = val;
}

void Detector::signalPositive(bool val)
{
	m_plot_pex = val;
}

void Detector::signalNegative(bool val)
{
	m_plot_nex = val;
}

void Detector::onUpdatePoints(const vector<TLDPoint> &points, const TLDPointColor &color)
{

}
void Detector::onUpdateInfo(const char *infoString)
{

}
void Detector::onUpdateConfidence(double conf)
{
	m_confidence = conf;
}
void Detector::onPexCount(int conf)
{

}
void Detector::onNexCount(int conf)
{

}

void Detector::onUpdateRect(TLDRect rect)
{
	m_bb_output.setHeight(rect.height());
	m_bb_output.setWidth(rect.width());
	m_bb_output.setX(rect.x());
	m_bb_output.setY(rect.y());
}

TLDRect Detector::getOutputRect(void) const
{
	return m_bb_output;
}

double Detector::getOutputConfidence(void) const
{
	return m_confidence;
}
