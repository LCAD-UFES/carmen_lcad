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

#ifndef LK_
#define LK_

#include <opencv/cv.h>

class Lk
{

public:
	Lk();
	~Lk();
	void init();
	void track(cv::Mat& imgI, cv::Mat& imgJ, const cv::Mat_<double>& ptsIMat, const cv::Mat_<double>& ptsJMat, cv::Mat_<double>& outMat, int level = 5);

public:
	void emitStatus(const char *message);

protected:
	void euclideanDistance(CvPoint2D32f *point1, CvPoint2D32f *point2, float *match, int nPts);
	void normCrossCorrelation(IplImage *imgI, IplImage *imgJ, CvPoint2D32f *points0, CvPoint2D32f *points1, int nPts, char *status, float *match,int winsize, int method);
};

#endif /* LK_ */
