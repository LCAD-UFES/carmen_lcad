
#include "semantic_segmentation.h"
#include <cstdio>

int
main()
{
	SegNet seg_net("segnet_model_driving_webdemo.prototxt", "segnet_iter_30000_timo.caffemodel", "camvid12.png");

	printf("Running...\n");
	cv::Mat img = cv::imread("example.png");
	cv::Mat prediction = seg_net.Predict(img);
	cv::Mat colors = seg_net.ClassesToColors(prediction);
	printf("Done...\n");

	printf("Output image shape: %d %d %d\n", prediction.rows, prediction.cols, prediction.channels());
	printf("Showing a subregion : \n");

	for (int ch = 0; ch < 3; ch++)
	{
		printf("Channel %d: \n", ch);

		for (int i = 0; i < 40; i++)
		{
			for (int j = 0; j < 60; j++)
				printf("%d ", prediction.data[3 * (i * prediction.cols + j) + ch]);

			printf("\n");
		}

		printf("\n");
	}

	cv::imshow("input", img);
	cv::imshow("output", colors);
	cv::waitKey(-1);

	return 0;
}











