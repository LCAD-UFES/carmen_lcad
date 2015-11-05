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

#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv/cv.h>
#include <algorithm>
#include <math.h>
#include <numeric>

static const double pi = 3.14159265358979323846264338327950288419716939937510;
static const double eu = 2.71828182845904523536028747135266249775724709369995;
#define nan std::numeric_limits<double>::quiet_NaN()
#define inf std::numeric_limits<double>::infinity()

#define LOG_FILE "debug.txt"
// Macros
#define isnan(x) !((x < 0) || (x > 0) || (x == 0))

//#define DEBUG_LOG
#ifdef DEBUG_LOG
#define LOG(msg) \
	FILE *pLogFile = NULL; \
	pLogFile = fopen (LOG_FILE, "a"); \
	if (pLogFile) { \
	fprintf (pLogFile, "%s\n", msg); \
	fclose (pLogFile); }
#else
#define LOG(msg)
#endif

using namespace cv;

bool loadMatrix(cv::Mat_<double>& m, std::string filename);
bool loadMatrix(cv::Mat_<uchar>& m, std::string filename);
bool loadMatrix(cv::Mat_<int>& m, std::string filename);
double fix(double d);
void loadImage(const cv::Mat& mat, IplImage *image);

template <class T>
void printMatrix(cv::Mat& m, std::string name = "")
{
	FILE *pLogFile = NULL;
	pLogFile = fopen (LOG_FILE, "a");
	if (!pLogFile) return;

	fprintf (pLogFile, "Start: %s %d x %d\n", name, m.rows, m.cols);
	for (int row = 0; row < m.rows; ++row)
	{
		for (int col = 0; col < m.cols; ++col)
			fprintf (pLogFile, "%f", m.at<T>(row, col));

		fprintf (pLogFile, "\n");
	}
	fprintf (pLogFile, "End:\n");
	fclose (pLogFile);
}

template<class T>
void printMatrix(T& m, std::string name = "")
{
	FILE *pLogFile = NULL;
	pLogFile = fopen (LOG_FILE, "a");
	if (!pLogFile) return;

	fprintf (pLogFile, "Start: %s %d x %d\n", name, m.rows, m.cols);
	for (int row = 0; row < m.rows; ++row)
	{
		for (int col = 0; col < m.cols; ++col)
			fprintf (pLogFile, "%f", m(row, col));

		fprintf (pLogFile, "\n");
	}
	fprintf (pLogFile, "End:\n");
	fclose (pLogFile);
}

template<class T>
bool colon(T j, T d, T k, std::vector<T>& v)
{
	if ((d == 0) || ((d > 0) && (j > k)) || ((d < 0) && (j < k))) return false;
	//T m = (T) fix((k-j)/d);
	for (T r=j; r <= k; r=r+d)
		v.push_back(r);
	return true;
}

template<class T>
bool colon(T j, T d, T k, cv::Mat_<T>& m)
{
	if ((d == 0) || ((d > 0) && (j > k)) || ((d < 0) && (j < k))) return false;
	//T m = (T) fix((k-j)/d);
	std::vector<T> v;
	T l = k+0.00000000001;
	for (T r=j; r <= l; r=r+d)
		v.push_back(r);
	m.create(1, v.size());
	for (unsigned i=0; i < v.size(); ++i)
		m(0, i) = v.at(i);
	return true;
}

template<class T>
void ntuples(T& inMat1, T& inMat2, T& outMat)
{
	int col = 0;
	outMat.create(2, inMat1.cols * inMat2.cols);
	for (int i=0; i < inMat1.cols; ++i)
	{
		for (int j=0; j < inMat2.cols; ++j)
		{
			outMat(0, col) = inMat1(0, i);
			outMat(1, col) = inMat2(0, j);
			col++;
		}
	}
}

template<class T>
void repmat(T& a, int m, T& outMat, int n = -1)
{
	//%REPMAT Replicate and tile an array.
	//%   B = repmat(A,M,N) creates a large matrix B consisting of an M-by-N
	//%   tiling of copies of A. The size of B is [size(A,1)*M, size(A,2)*N].
	//%   The statement repmat(A,N) creates an N-by-N tiling.
	//%   
	//%   B = REPMAT(A,[M N]) accomplishes the same result as repmat(A,M,N).
	//%
	if (n == -1) n = m;
	outMat.create(m*a.rows, n*a.cols);
	for (int tr=0; tr < m; ++tr)
		for (int tc=0; tc < n; ++tc)
			for (int r=0; r < a.rows; ++r)
				for (int c=0; c < a.cols; ++c)
					outMat(r+(tr*a.rows), c+(tc*a.cols)) = a(r,c);
}

template <class T>
void maxMat(const cv::Mat_<T>& mat, int& index, T& maxValue)
{
	index = 0;
	maxValue = mat(0,0);
	for (int c=0; c < mat.cols; ++c)
	{
		if (mat(0, c) > mat(0, index))
		{
			index = c;
			maxValue = mat(0, index);
		}
	}
}

template <class T>
void col2row(const cv::Mat_<T>& inMat, cv::Mat_<T>& outMat)
{
	T* data = (T*) inMat.data;
	outMat.create(inMat.rows, inMat.cols);
	for (int r=0; r < inMat.rows; ++r)
		for (int c=0; c < inMat.cols; ++c)
			outMat(r, c) = *data++;
}

template <class T>
void mean(const cv::Mat_<T>& inMat, int dim, cv::Mat_<T>& outMat)
{
	if (!((dim == 1) || (dim == 2)))
		return;

	if (dim == 1)
	{
		outMat.create(1, inMat.cols);
		for (int c=0; c < inMat.cols; ++c)
		{
			cv::Scalar m = cv::mean(inMat.col(c));
			outMat(0, c) = m[0];
		}
		return;
	}

	if (dim == 2)
	{
		outMat.create(inMat.rows, 1);
		for (int r=0; r < inMat.rows; ++r)
		{
			cv::Scalar m = cv::mean(inMat.row(r));
			outMat(r, 0) = m[0];
		}
	}
}

template<class T>
void gaussMatrix(unsigned csize, double sigma, cv::Mat_<T>& gaussMat)
{
	T k = 1.986*pi*sigma*sigma;
	T e = -1.0 / (2.0*sigma*sigma);
	gaussMat.create(csize, csize);
	for (T i=0; i < csize; i = i + 1.0)
	{
		T x = i - (csize/2.0) + 0.5;
		for (T j=0; j < csize; j = j + 1.0)
		{
			T y = j - (csize/2.0) + 0.5;
			gaussMat(i,j) = exp((T) e*((x*x) + (y*y))) / k;
		}
	}
}

template<class T>
cv::Mat_<T> reshape(cv::Mat_<T> inMat, int rows, int cols)
{
	cv::Mat_<T> tmp(rows, cols);
	if ((rows*cols) == (inMat.rows*inMat.cols))
	{
		int i=0;
		int j=0;
		int k=0;
		T* data = (T*) tmp.data;
		for (int c=0; c < inMat.cols; ++c)
		{
			for (int r=0; r < inMat.rows; ++r)
			{
				data[i + k*cols] = inMat(r,c);
				k++;
				if (k == rows)
				{
					i = ++j;
					k = 0;
				}
			}
		}
	}
	return tmp;
}

template<class T>
void printMemoryBlock(T* block, size_t size, std::string name = "")
{
	FILE *pLogFile = NULL;
	pLogFile = fopen (LOG_FILE, "a");
	if (!pLogFile) return;

	fprintf (pLogFile, "Start: %s: %d\n", name, size);
	T* ptr = block;
	for (size_t i=0; i < size; ++i)
		fprintf (pLogFile, "%f", *ptr++);

	fprintf (pLogFile, "End:\n");
	fclose (pLogFile);
}

#endif /* UTILS_HPP */
