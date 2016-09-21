/*
 * Project:  vanishingPoint
 *
 * File:     main.cpp
 *
 * Contents: Creation, initialisation and usage of MSAC object
 *           for vanishing point estimation in images or videos
 *
 * Author:   Marcos Nieto <marcos.nieto.doncel@gmail.com>
 *
 * Homepage: www.marcosnieto.net/vanishingPoint
 */


#ifdef WIN32
	#include <windows.h>
#endif
#include <iostream>
#ifdef linux
	#include <stdio.h>
#endif

#define USE_PPHT
#define MAX_NUM_LINES	200

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "MSAC.h"

using namespace std;
using namespace cv;

void help()
{
	 cout << "/*\n"
         << " **************************************************************************************************\n"
		 << " * Vanishing point detection using Hough and MSAC \n"
         << " * ----------------------------------------------------\n"		 
		 << " * \n"
		 << " * Author:Marcos Nieto\n"
		 << " * www.marcosnieto.net\n"
		 << " * marcos.nieto.doncel@gmail.com\n"
		 << " * \n"
		 << " * Date:01/12/2011\n"
		 << " **************************************************************************************************\n"
		 << " * \n"
		 << " * Usage: \n"		 		 
		 << " *		-numVps		# Number of vanishing points to detect (at maximum) \n"
		 << " *		-mode		# Estimation mode (default is NIETO): LS (Least Squares), NIETO\n"
		 << " *		-video		# Specifies video file as input (if not specified, camera is used) \n"
		 << " *		-image		# Specifies image file as input (if not specified, camera is used) \n"
		 << " *		-verbose	# Actives verbose: ON, OFF (default)\n"
		 << " *		-play		# ON: the video runs until the end; OFF: frame by frame (key press event)\n"
		 << " *		-resizedWidth	# Specifies the desired width of the image (the height is computed to keep aspect ratio)\n"
		 << " * Example:\n"
		 << " *		vanishingPoint.exe -numVps 2 -video myVideo.avi -verbose ON\n"
		 << " *		vanishingPoint.exe -numVps 2 -image myImage.jpg\n"
		 << " *		vanishingPoint.exe -numVps 1 -play OFF -resizedWidth 640\n"		 
		 << " * \n"
		 << " * Keys:\n"
		 << " *		Esc: Quit\n"
         << " */\n" << endl;
}

/** This function contains the actions performed for each image*/
void processImage(MSAC &msac, int numVps, cv::Mat &imgGRAY, cv::Mat &outputImg)
{
	cv::Mat imgCanny;

	// Canny
	cv::Canny(imgGRAY, imgCanny, 180, 120, 3);

	// Hough
	vector<vector<cv::Point> > lineSegments;
	vector<cv::Point> aux;
#ifndef USE_PPHT
	vector<Vec2f> lines;
	cv::HoughLines( imgCanny, lines, 1, CV_PI/180, 200);

	for(size_t i=0; i< lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines[i][1];

		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;

		Point pt1, pt2;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		aux.clear();
		aux.push_back(pt1);
		aux.push_back(pt2);
		lineSegments.push_back(aux);

		line(outputImg, pt1, pt2, CV_RGB(0, 0, 0), 1, 8);
	
	}
#else
	vector<Vec4i> lines;	
	int houghThreshold = 70;
	if(imgGRAY.cols*imgGRAY.rows < 400*400)
		houghThreshold = 100;		
	
	cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10,10);

	while(lines.size() > MAX_NUM_LINES)
	{
		lines.clear();
		houghThreshold += 10;
		cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10, 10);
	}
	for(size_t i=0; i<lines.size(); i++)
	{		
		Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];
		line(outputImg, pt1, pt2, CV_RGB(0,0,0), 2);
		/*circle(outputImg, pt1, 2, CV_RGB(255,255,255), CV_FILLED);
		circle(outputImg, pt1, 3, CV_RGB(0,0,0),1);
		circle(outputImg, pt2, 2, CV_RGB(255,255,255), CV_FILLED);
		circle(outputImg, pt2, 3, CV_RGB(0,0,0),1);*/

		// Store into vector of pairs of Points for msac
		aux.clear();
		aux.push_back(pt1);
		aux.push_back(pt2);
		lineSegments.push_back(aux);
	}
	
#endif

	// Multiple vanishing points
	std::vector<cv::Mat> vps;			// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
	std::vector<std::vector<int> > CS;	// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
	std::vector<int> numInliers;

	std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
	
	// Call msac function for multiple vanishing point estimation
	msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, numVps); 
	for(int v=0; v<vps.size(); v++)
	{
		printf("VP %d (%.3f, %.3f, %.3f)", v, vps[v].at<float>(0,0), vps[v].at<float>(1,0), vps[v].at<float>(2,0));
		fflush(stdout);
		double vpNorm = cv::norm(vps[v]);
		if(fabs(vpNorm - 1) < 0.001)
		{
			printf("(INFINITE)");
			fflush(stdout);
		}
		printf("\n");
	}		
		
	// Draw line segments according to their cluster
	msac.drawCS(outputImg, lineSegmentsClusters, vps);
}

/** Main function*/
int main(int argc, char** argv)
{	
	// Images
	cv::Mat inputImg, imgGRAY;	
	cv::Mat outputImg;

	// Other variables
	char *videoFileName = 0;
	char *imageFileName = 0;
	cv::VideoCapture video;
	bool useCamera = true;
	int mode = MODE_NIETO;
	int numVps = 1;
	bool playMode = true;
	bool stillImage = false;
	bool verbose = false;

	int procWidth = -1;
	int procHeight = -1;
	cv::Size procSize;

	// Start showing help
	help();

	// Parse arguments
	if(argc < 2)
		return -1;	
	for(int i=1; i<argc; i++)
	{
		const char* s = argv[i];

		if(strcmp(s, "-video" ) == 0)
		{
			// Input video is a video file
			videoFileName = argv[++i];
			useCamera = false;
		}
		else if(strcmp(s,"-image") == 0)
		{
			// Input is a image file
			imageFileName = argv[++i];
			stillImage = true;
			useCamera = false;
		}
		else if(strcmp(s, "-resizedWidth") == 0)
		{
			procWidth = atoi(argv[++i]);
		}
		else if(strcmp(s, "-verbose" ) == 0)
		{
			const char* ss = argv[++i];
			if(strcmp(ss, "ON") == 0 || strcmp(ss, "on") == 0 
				|| strcmp(ss, "TRUE") == 0 || strcmp(ss, "true") == 0 
				|| strcmp(ss, "YES") == 0 || strcmp(ss, "yes") == 0 )
				verbose = true;			
		}
		else if(strcmp(s, "-play" ) == 0)
		{
			const char* ss = argv[++i];
			if(strcmp(ss, "OFF") == 0 || strcmp(ss, "off") == 0 
				|| strcmp(ss, "FALSE") == 0 || strcmp(ss, "false") == 0 
				|| strcmp(ss, "NO") == 0 || strcmp(ss, "no") == 0 
				|| strcmp(ss, "STEP") == 0 || strcmp(ss, "step") == 0)
				playMode = false;			
		}
		else if(strcmp(s, "-mode" ) == 0)
		{
			const char* ss = argv[++i];
			if(strcmp(ss, "LS") == 0)
				mode = MODE_LS;
			else if(strcmp(ss, "NIETO") == 0)
				mode = MODE_NIETO;
			else
			{
				perror("ERROR: Only LS or NIETO modes are supported\n");
			}
		}
		else if(strcmp(s,"-numVps") == 0)
		{
			numVps = atoi(argv[++i]);
		}
	}

	// Open video input
	if( useCamera )
		video.open(0);
	else
	{
		if(!stillImage)
			video.open(videoFileName);
	}

	// Check video input
	int width = 0, height = 0, fps = 0, fourcc = 0;
	if(!stillImage)
	{
		if( !video.isOpened() )
		{
			printf("ERROR: can not open camera or video file\n");
			return -1;
		}
		else
		{
			// Show video information
			width = (int) video.get(CV_CAP_PROP_FRAME_WIDTH);
			height = (int) video.get(CV_CAP_PROP_FRAME_HEIGHT);
			fps = (int) video.get(CV_CAP_PROP_FPS);
			fourcc = (int) video.get(CV_CAP_PROP_FOURCC);

			if(!useCamera)
				printf("Input video: (%d x %d) at %d fps, fourcc = %d\n", width, height, fps, fourcc);
			else
				printf("Input camera: (%d x %d) at %d fps\n", width, height, fps);
		}
	}
	else
	{
		inputImg = cv::imread(imageFileName);
		if(inputImg.empty())
			return -1;

		width = inputImg.cols;
		height = inputImg.rows;

		printf("Input image: (%d x %d)\n", width, height);

		playMode = false;
	}

	// Resize	
	if(procWidth != -1)
	{
	
		procHeight = height*((double)procWidth/width);
		procSize = cv::Size(procWidth, procHeight);

		printf("Resize to: (%d x %d)\n", procWidth, procHeight);	
	}
	else
		procSize = cv::Size(width, height);

	// Create and init MSAC
	MSAC msac;
	msac.init(mode, procSize, verbose);
	
	
	int frameNum=0;
	for( ;; )
	{
		if(!stillImage)
		{
			printf("\n-------------------------\nFRAME #%6d\n", frameNum);
			frameNum++;

			// Get current image		
			video >> inputImg;
		}	

		if( inputImg.empty() )
			break;
		
		// Resize to processing size
		cv::resize(inputImg, inputImg, procSize);		

		// Color Conversion
		if(inputImg.channels() == 3)
		{
			cv::cvtColor(inputImg, imgGRAY, CV_BGR2GRAY);	
			inputImg.copyTo(outputImg);
		}
		else
		{
			inputImg.copyTo(imgGRAY);
			cv::cvtColor(inputImg, outputImg, CV_GRAY2BGR);
		}

		// ++++++++++++++++++++++++++++++++++++++++
		// Process		
		// ++++++++++++++++++++++++++++++++++++++++
		processImage(msac, numVps, imgGRAY, outputImg);


		// View
		imshow("Output", outputImg);
		if(playMode)
			cv::waitKey(1);
		else
			cv::waitKey(0);

		char q = (char)waitKey(1);

		if( q == 27 )
		{
			printf("\nStopped by user request\n");
			break;
		}	

		if(stillImage)
			break;
	}

	if(!stillImage)
		video.release();
	
	return 0;	
	
}
