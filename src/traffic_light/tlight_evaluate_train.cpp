
#include "tlight_vgram.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>


using namespace std;
using namespace cv;


// debug
struct timeval start_time, end_time;


void
start_count_time()
{
	gettimeofday(&start_time, NULL);
}


double
get_ellapsed_time()
{
	gettimeofday(&end_time, NULL);
	double ellapsed = (end_time.tv_sec - start_time.tv_sec) * 1000000 + (end_time.tv_usec - start_time.tv_usec);
	return ellapsed / 1000000.0;
}


int
main(int argc, char **argv)
{
	if (argc < 3)
		exit(printf("Error:: Use %s <trained net> <file with list of image>\n", argv[0]));

	int label;
	double confidence;

	TLightVgRam net(argv[1]);
	FILE *f = fopen(argv[2], "r");

	double accuracy = 0.0;
	int n = 1;

	while (!feof(f))
	{
		int true_label;
		char image_name[1024];

		fscanf(f, "%s %d", image_name, &true_label);

		check_image_exists(image_name);
		Mat img = imread(image_name);
		Mat *m = preproc_image(img, Rect(0,0,img.cols, img.rows), 20, 40);

		start_count_time();
		net.Forward(m, &label, &confidence);
		double t = get_ellapsed_time();

		if (label == true_label)
			accuracy += 1;

		printf("Prediction: %s Groundtruth: %s Confidence: %lf Forward time: %lf Acc: %lf %s\n",
			label_to_string(label),
			label_to_string(true_label),
			confidence, t,
			100 * (accuracy / (double) n),
			(label == true_label)?(" "):("*"));

		imshow("img", img);
		imshow("proc", *m);

		if (label == true_label)
			waitKey(1);
		else
			waitKey(0);

		n++;
	}

	fclose(f);
	return 0;
}
