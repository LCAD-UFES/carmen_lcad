
#include "tlight_vgram.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>


using namespace std;
using namespace cv;


vector<Data*> reds;
vector<Data*> greens;
vector<Data*> yellows;
vector<Data*> offs;


const int NUM_TRAINING_SESSIONS = 100;
const int TLIGHT_COLS = 40;
const int TLIGHT_ROWS = 80;
const int NUM_NEURONS = 25;
//const int NUM_SYNAPSIS = 2048;
const int NUM_SYNAPSIS = 1024;
const int MEM_SIZE = 5000;
const int NUM_CLASSES = 4;
const int VIEW_ACTIVE = 0;
const int ZOOM = 5;

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
mouse_callback(int event, int x, int y, int flags, void *userdata)
{
	(void) flags;
	Mat *m = (Mat *) userdata;

	if (event == EVENT_LBUTTONDOWN)
	{
		int r, g, b;

		b = m->data[3 * (y * m->cols + x) + 0];
		g = m->data[3 * (y * m->cols + x) + 1];
		r = m->data[3 * (y * m->cols + x) + 2];

		printf("x: %d y: %d r: %d g: %d b: %d\n", x, y, r, g, b);
	}
}


// function for debugging purposes
void
view_yellow_images()
{
	Mat zoom(Size(ZOOM * TLIGHT_COLS, ZOOM * TLIGHT_ROWS), CV_8UC1);

	namedWindow("img");
	setMouseCallback("img", mouse_callback, &zoom);

	for (size_t i = 0; i < yellows.size(); i++)
	{
		if (i % 3 == 0)
		{
			printf("red\n");
			resize(*preproc(reds[i], TLIGHT_COLS, TLIGHT_ROWS), zoom, zoom.size());
		}
		else if (i % 3 == 1)
		{
			printf("greens\n");
			resize(*preproc(greens[i], TLIGHT_COLS, TLIGHT_ROWS), zoom, zoom.size());
		}
		else if (i % 3 == 2)
		{
			printf("yellows\n");
			resize(*preproc(yellows[i], TLIGHT_COLS, TLIGHT_ROWS), zoom, zoom.size());
		}

		imshow("img", zoom);

		char c = ' ';

		while (c != 'q')
			c = waitKey(1);

		if (c == 's')
		{
			char cmd[2048];
			sprintf(cmd, "cp %s problem_%ld.png", yellows[i]->image_path, i);
			printf("cmd: %s\n", cmd);
			system(cmd);
		}
	}
	exit(0);
}


void
init_confusion_matrix(double ***confusion_matrix, int n)
{
	*confusion_matrix = (double**) calloc (n, sizeof(double*));

	for (int i = 0; i < n; i++)
	{
		(*confusion_matrix)[i] = (double*) calloc (n, sizeof(double));

		for (int j = 0; j < n; j++)
			(*confusion_matrix)[i][j] = 0.0;
	}
}


void
print_confusion_mat(double **confusion_matrix, int n)
{
	printf("Confusion matrix (frequency): \n");

	for (int i = 0; i < n; i++)
	{
		printf("%s:\t", label_to_string(i));
		double sum = 0.0;

		for (int j = 0; j < n; j++)
			sum += confusion_matrix[i][j];

		if (sum == 0.0)
			sum = 1.0;

		for (int j = 0; j < n; j++)
			printf("%.2lf\t", confusion_matrix[i][j]);

		printf("\n");
	}

	printf("Confusion matrix (%%): \n");
	for (int i = 0; i < n; i++)
	{
		printf("%s:\t", label_to_string(i));
		double sum = 0.0;

		for (int j = 0; j < n; j++)
			sum += confusion_matrix[i][j];

		if (sum == 0.0)
			sum = 1.0;

		for (int j = 0; j < n; j++)
			printf("%.2lf\t", confusion_matrix[i][j] / sum);

		printf("\n");
	}
}


void
train_sample(TLightVgRam &net, Data *d, int label)
{
	Mat *tlight = preproc(d, TLIGHT_COLS, TLIGHT_ROWS);
	net.Train(tlight, label);

//	static int ts = 0;
//	char nome[1024];
//	sprintf(nome, "mlp/img/%s_%04d.png", label_to_string(label), ts++);
//	imwrite(nome, *tlight);
//	printf("TRAIN %s %d\n", nome, label);
}


size_t
num_max_images()
{
	size_t max = reds.size();

	if (greens.size() > max)
		max = greens.size();

	if (yellows.size() > max)
		max = yellows.size();

	if (offs.size() > max)
		max = offs.size();

	return max;
}


void
train_net(TLightVgRam &net, char *output_filename)
{
	// OBS: if the neuron memory is not enough for storing all the patterns from the training set,
	// the training order matter (the last patterns will replace the first)!

	size_t i;
	size_t max = num_max_images();

	printf("Training...\n");

	for (i = 0; i < max / 2; i++)
	//for (i = 0; i < max; i++)
	{
		if (i < reds.size() / 2)
		//if (i < reds.size())
			train_sample(net, reds[i], 0);
		else
			train_sample(net, reds[rand() % (reds.size() / 2)], 0);
			//train_sample(net, reds[rand() % (reds.size())], 0);

		if (i < greens.size() / 2)
		//if (i < greens.size())
			train_sample(net, greens[i], 1);
		else
			train_sample(net, greens[rand() % (greens.size() / 2)], 1);
			//train_sample(net, greens[rand() % (greens.size())], 1);

		if (i < yellows.size() / 2)
		//if (i < yellows.size())
			train_sample(net, yellows[i], 2);
		else
			train_sample(net, yellows[rand() % (yellows.size() / 2)], 2);
			//train_sample(net, yellows[rand() % (yellows.size())], 2);

		if (i < offs.size() / 2)
		//if (i < offs.size())
			train_sample(net, offs[i], 3);
		else
			train_sample(net, offs[rand() % (offs.size() / 2)], 3);
			//train_sample(net, offs[rand() % (offs.size())], 3);
	}

	printf("Done\n");
	printf("Memory occupation: %d\n", net.MemOccupation());

	printf("Saving training data...\n");
	net.Save(output_filename);
	printf("Done\n");
}


void
test_sample_and_update_report(TLightVgRam &net, Data *d, int label, double *avg_forward_time, double **confusion_matrix, int *num_forwards)
{
	int prediction;
	double confidence;

	Mat *tlight = preproc(d, TLIGHT_COLS, TLIGHT_ROWS);

//	static int ts = 0;
//	char nome[1024];
//	sprintf(nome, "mlp/img/%s_%04d.png", label_to_string(label), ts++);
//	imwrite(nome, *tlight);
//	printf("TEST %s %d\n", nome, label);

	start_count_time();
	net.Forward(tlight, &prediction, &confidence);
	(*avg_forward_time) += get_ellapsed_time();
	(*num_forwards)++;

	confusion_matrix[label][prediction]++;

	if (label != prediction)
	{
		static int error_count = 1;
		static char name[2048];
		sprintf(name, "problems/err_%03d_%s_%s.png", error_count, label_to_string(label), label_to_string(prediction));

		static Mat *error_zoom = NULL;

		if (error_zoom == NULL)
			error_zoom = new Mat(Size(ZOOM * TLIGHT_COLS, ZOOM * TLIGHT_ROWS), CV_8UC3);

		resize(*tlight, *error_zoom, error_zoom->size());
		fprintf(stderr, "Miss %d! exp: %s pred: %s image: %s saving: %s\n", error_count, label_to_string(label), label_to_string(prediction), d->image_path, name);
		imwrite(name, *error_zoom);

		//imshow("zoom", *error_zoom);
		//waitKey(0);

		error_count++;
	}

	if (VIEW_ACTIVE)
	{
		static Mat *zoom = NULL;

		if (zoom == NULL)
			zoom = new Mat(Size(ZOOM * TLIGHT_COLS, ZOOM * TLIGHT_ROWS), CV_8UC3);

		printf("--------------------------------------------\n\n");
		printf("Image: %s\n\n", d->image_path);
		printf("Net pred: %s Expected: %s Confidence: %lf %s\n\n",
				label_to_string(prediction),
				label_to_string(label), confidence,
				(label == prediction) ? (" ") : ("*"));
		resize(*tlight, *zoom, zoom->size());
		imshow("tligtht", *zoom);
		waitKey(1);
	}
}


void
test_net(TLightVgRam &net)
{
	size_t i;
	double **confusion_matrix;
	int num_forwards = 0;
	double avg_forward_time = 0.0;

	init_confusion_matrix(&confusion_matrix, NUM_CLASSES);

	printf("Testing reds...\n");

	for (i = reds.size() / 2; i < reds.size(); i++)
	//for (i = 0; i < reds.size(); i++)
		test_sample_and_update_report(net, reds[i], 0, &avg_forward_time, confusion_matrix, &num_forwards);

	printf("Done\n");
	print_confusion_mat(confusion_matrix, NUM_CLASSES);
	printf("\nAvg forward time: %lf s\n", avg_forward_time / (double) num_forwards);
	printf("Testing greens...\n");

	for (i = greens.size() / 2; i < greens.size(); i++)
	//for (i = 0; i < greens.size(); i++)
		test_sample_and_update_report(net, greens[i], 1, &avg_forward_time, confusion_matrix, &num_forwards);

	printf("Done\n");
	print_confusion_mat(confusion_matrix, NUM_CLASSES);
	printf("\nAvg forward time: %lf s\n", avg_forward_time / (double) num_forwards);
	printf("Testing yellows...\n");

	for (i = yellows.size() / 2; i < yellows.size(); i++)
	//for (i = 0; i < yellows.size(); i++)
		test_sample_and_update_report(net, yellows[i], 2, &avg_forward_time, confusion_matrix, &num_forwards);

	printf("Done\n");
	print_confusion_mat(confusion_matrix, NUM_CLASSES);
	printf("\nAvg forward time: %lf s\n", avg_forward_time / (double) num_forwards);
	printf("Testing offs...\n");

	for (i = offs.size() / 2; i < offs.size(); i++)
	//for (i = 0; i < offs.size(); i++)
		test_sample_and_update_report(net, offs[i], 3, &avg_forward_time, confusion_matrix, &num_forwards);

	printf("Done\n\n");

	print_confusion_mat(confusion_matrix, NUM_CLASSES);
	printf("\nAvg forward time: %lf s\n", avg_forward_time / (double) num_forwards);
}


void
print_initial_data_report()
{
	printf("Num red samples: %ld\n", reds.size());
	printf("Num green samples: %ld\n", greens.size());
	printf("Num yellow samples: %ld\n", yellows.size());
	printf("Num offs samples: %ld\n", offs.size());
}


int
main(int argc, char **argv)
{
	srand(0);

	char *output_filename;

	if (argc < 6)
		exit(printf("Error:: Use %s <reds file> <greens file> <yellows file> <off file> <train-output-file>\n", argv[0]));

	loadData(argv[1], reds);
	loadData(argv[2], greens);
	loadData(argv[3], yellows);
	loadData(argv[4], offs);
	output_filename = argv[5];

	print_initial_data_report();
	//view_yellow_images();

	VgRamNeuronConfig *config = new VgRamNeuronConfig();

	config->replacement = ReplacementStrategyRandom;
	config->memory_size = MEM_SIZE;

	TLightVgRam net(config, NUM_NEURONS, NUM_SYNAPSIS, TLightVgRam::ColorOption::UseGreenAsHighAndDiscardBlue);

	train_net(net, output_filename);
	test_net(net);

	return 0;
}
