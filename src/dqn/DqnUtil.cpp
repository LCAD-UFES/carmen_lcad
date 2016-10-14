

#include "DqnUtil.h"
#include <sys/time.h>
#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace cv;


double
get_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	double time_in_sec = (tv.tv_sec) + ((double) tv.tv_usec * (double) 10e-7);
	return time_in_sec;
}


void
show_control(int *button_state, int command)
{
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;

	Mat m = Mat(200, 400, CV_8UC3);
	memset(m.data, 0, m.rows * m.cols * m.channels() * sizeof(uchar));

	line(m, Point(0, 0), Point(m.cols, 0), Scalar::all(255), 2);
	line(m, Point(0, m.rows - 1), Point(m.cols - 1, m.rows - 1), Scalar::all(255), 2);
	line(m, Point(0, m.rows / 2), Point(m.cols - 1, m.rows / 2), Scalar::all(255), 2);

	for (int i = 0; i < 5; i++)
		line(m, Point((m.cols / 5) * i, 0), Point((m.cols / 5) * i, m.rows - 1), Scalar::all(255), 2);
	line(m, Point(m.cols - 1, 0), Point(m.cols - 1, m.rows - 1), Scalar::all(255), 2);

	putText(m, "L", Point((m.cols / 5) * 0 + (m.cols / 10), (m.rows / 2) * 0 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "U", Point((m.cols / 5) * 1 + (m.cols / 10), (m.rows / 2) * 0 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "R", Point((m.cols / 5) * 2 + (m.cols / 10), (m.rows / 2) * 0 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "Y", Point((m.cols / 5) * 3 + (m.cols / 10), (m.rows / 2) * 0 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "X", Point((m.cols / 5) * 4 + (m.cols / 10), (m.rows / 2) * 0 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);

	putText(m, "<<", Point((m.cols / 5) * 0 + (m.cols / 10), (m.rows / 2) * 1 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "D", Point((m.cols / 5) * 1 + (m.cols / 10), (m.rows / 2) * 1 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, ">>", Point((m.cols / 5) * 2 + (m.cols / 10), (m.rows / 2) * 1 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "B", Point((m.cols / 5) * 3 + (m.cols / 10), (m.rows / 2) * 1 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);
	putText(m, "A", Point((m.cols / 5) * 4 + (m.cols / 10), (m.rows / 2) * 1 + (m.rows / 10)), fontFace, fontScale, Scalar::all(255), thickness, 8);

	// up
	if (button_state[0]) rectangle(m, Point((m.cols / 5), 0), Point((m.cols / 5) * 2, m.rows / 2), Scalar(0, 0, 255), -1);
	// down
	if (button_state[2]) rectangle(m, Point((m.cols / 5), m.rows / 2), Point((m.cols / 5) * 2, m.rows - 1), Scalar(0, 0, 255), -1);
	// left
	if (button_state[3]) rectangle(m, Point(0, m.rows / 2), Point((m.cols / 5), m.rows - 1), Scalar(0, 0, 255), -1);
	// right
	if (button_state[1]) rectangle(m, Point((m.cols / 5) * 2, m.rows / 2), Point((m.cols / 5) * 3, m.rows - 1), Scalar(0, 0, 255), -1);

	// l
	if (command == 9) rectangle(m, Point(0, 0), Point((m.cols / 5), m.rows / 2), Scalar(255, 0, 0), -1);
	// r
	if (command == 10) rectangle(m, Point((m.cols / 5) * 2, 0), Point((m.cols / 5) * 3, m.rows / 2), Scalar(255, 0, 0), -1);
	// y
	if (command == 8) rectangle(m, Point((m.cols / 5) * 3, 0), Point((m.cols / 5) * 4, m.rows / 2), Scalar(255, 0, 0), -1);
	// x
	if (command == 7) rectangle(m, Point((m.cols / 5) * 4, 0), Point((m.cols / 5) * 5 - 1, m.rows / 2), Scalar(255, 0, 0), -1);
	// b
	if (command == 6) rectangle(m, Point((m.cols / 5) * 3, m.rows / 2), Point((m.cols / 5) * 4, m.rows - 1), Scalar(255, 0, 0), -1);
	// A
	if (command == 5) rectangle(m, Point((m.cols / 5) * 4, m.rows / 2), Point((m.cols / 5) * 5 - 1, m.rows - 1), Scalar(255, 0, 0), -1);


	imshow("control", m);
	waitKey(1);
}
