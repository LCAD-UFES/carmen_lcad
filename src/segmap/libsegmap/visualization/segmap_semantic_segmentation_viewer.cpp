
#include <vector>
#include <opencv/cv.hpp>
#include <carmen/segmap_colormaps.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>

using namespace std;
using namespace cv;


Mat
segmented_image_view(Mat &m)
{
	CityScapesColorMap color_map;
	Mat view(m.rows, m.cols, CV_8UC3);

	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			int cl = m.data[3 * (i * m.cols + j)];
			Scalar color;

			color = color_map.color(cl);

			view.data[3 * (i * view.cols + j)] = color[2];
			view.data[3 * (i * view.cols + j) + 1] = color[1];
			view.data[3 * (i * view.cols + j) + 2] = color[0];
		}
	}

	return view;
}
