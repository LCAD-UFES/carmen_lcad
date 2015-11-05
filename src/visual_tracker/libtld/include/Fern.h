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

#ifndef FERNH_
#define FERNH_

#include <vector>
#include <opencv/cv.h>

using namespace cv;

typedef std::pair<cv::Mat_<int>* /*img*/, cv::Mat_<int>* /*blur*/ >ImagePairType;

class Fern
{

public:
	Fern();
	~Fern();
	void reset();
	bool init(const IplImage& image, const Mat_<double>& grid, const Mat_<double>& features, const Mat_<double>& scales);
	void getPatterns(const ImagePairType& input, const cv::Mat_<double>& idx, double var, cv::Mat_<double>& patt, cv::Mat_<double>& status);
	void update(const cv::Mat_<double>& x, const cv::Mat_<double>& y, double thr_fern, int bootstrap, const cv::Mat_<double>* idx = NULL);
	void evaluate(const cv::Mat_<double>& X, cv::Mat_<double>& resp0);
	void detect(const ImagePairType& img, double maxBBox, double minVar, const cv::Mat_<double>& conf, const cv::Mat_<double>& patt); 

public:
	void emitStatus(const char *message);

protected:
	void update(double *x, int C, int N, int offset);
	double measure_forest(double *idx, int offset);
	int measure_tree_offset(unsigned *img, int idx_bbox, int idx_tree);
	double measure_bbox_offset(unsigned *blur, int idx_bbox, double minVar, double *tPatt);
	int* create_offsets(double *scale0, double *x0);
	int* create_offsets_bbox(double *bb0);
	double randdouble();
	double bbox_var_offset(double *ii,double *ii2, int *off);
	void iimg(unsigned *in, double *ii, int imH, int imW);
	void iimg2(unsigned *in, double *ii2, int imH, int imW);
	int row2col(int ci);

private:
	double thrN;
	int nBBOX;
	int mBBOX;
	int nTREES;
	int nFEAT;
	int nSCALE;
	int iHEIGHT;
	int iWIDTH;
	int *BBOX;
	int *OFF;
	double *IIMG;
	double *IIMG2;
	std::vector<std::vector <double> > WEIGHT;
	std::vector<std::vector <int> > nP;
	std::vector<std::vector <int> > nN;
	int BBOX_STEP;
	int nBIT; // number of bits per feature
};

#endif /* FERNH_ */
