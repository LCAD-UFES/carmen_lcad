
#include "tlight_vgram.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>
#include <cstdlib>

using namespace std;
using namespace cv;


class Data
{
public:
	char image_path[1024];
	Rect roi;
};

vector<Data*> reds;
vector<Data*> greens;

const int NUM_TRAINING_SESSIONS = 100;
const int TLIGHT_COLS = 20;
const int TLIGHT_ROWS = 40;
const int NUM_NEURONS = 25;
const int NUM_SYNAPSIS = 256;
const int MEM_SIZE = 1000;
const int RED = 0;
const int GREEN = 1;


void
loadData(char *filename, vector<Data*> &v)
{
	FILE *f = fopen(filename, "r");

	if (f == NULL)
		exit(printf("Error::Unable to open file '%s'\n", filename));

	int n, lx, ly, rx, ry;

	while (!feof(f))
	{
		Data *d = new Data();

		n = fscanf(f, "%s %d %d %d %d",
			d->image_path, &lx, &ly, &rx, &ry);

		d->roi.x = lx;
		d->roi.y = ly;
		d->roi.width = rx - lx;
		d->roi.height = ry - ly;

		if (n != 5)
		{
			delete(d);
			break;
		}

		v.push_back(d);
	}

	fclose(f);
}


void
check_image_exists(char *filename)
{
	FILE *f = fopen(filename, "r");

	if (f == NULL)
		exit(printf("Error: Unable to open image '%s'\n", filename));

	fclose(f);
}


Mat*
preproc(Data *d)
{
	static Mat *m = NULL;

	if (m == NULL)
		m = new Mat(Size(TLIGHT_COLS, TLIGHT_ROWS), CV_8UC1);

	check_image_exists(d->image_path);

	// load the image and select the roi area
	Mat img = imread(d->image_path)(d->roi);
	// blur
	GaussianBlur(img, img, Size(3, 3), 1);
	// convert to grayscale
	Mat gray;
	cvtColor(img, gray, CV_BGR2GRAY);
	// resize
	resize(gray, *m, m->size());

	return m;
}


int
main(int argc, char **argv)
{
	if (argc < 3)
		exit(printf("Error:: Use %s <reds file> <greens file>\n", argv[0]));

	loadData(argv[1], reds);
	loadData(argv[2], greens);

	VgRamNeuronConfig *config = new VgRamNeuronConfig();

	config->replacement = ReplacementStrategyRandom;
	config->memory_size = MEM_SIZE;

	TLightVgRam net(config, NUM_NEURONS, NUM_SYNAPSIS);

	Data *d;
	int status, prediction;
	double confidence;

	for (int i = 0; i < NUM_TRAINING_SESSIONS; i++)
	{
		if (i % 2 == 0)
		{
			d = reds[rand() % reds.size()];
			status = RED;
		}
		else
		{
			d = greens[rand() % greens.size()];
			status = GREEN;
		}

		Mat *tlight = preproc(d);

		// evaluate the sample to check if the net is learning
		if (i > 1)
		{
			net.Forward(tlight, &prediction, &confidence);
			printf("Net pred: %d Expected: %d Confidence: %lf %s\n",
					prediction, status, confidence, (status == prediction) ? (" ") : ("*"));
			imshow("tligtht", *tlight);
			waitKey(0);
		}
		else
			net.Train(tlight, status);
	}

	return 0;
}
