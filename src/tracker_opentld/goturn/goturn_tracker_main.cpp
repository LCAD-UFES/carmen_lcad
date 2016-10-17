// This file was mostly taken from the example given here:
// http://www.votchallenge.net/howto/integration.html

// Uncomment line below if you want to use rectangles
//#define VOT_RECTANGLE
//#include "native/vot.h"

#include "tracker/tracker.h"
#include "regressor/regressor_train.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

//using std::std::string;

double
color_based(cv::Mat *m, cv::Mat *n, BoundingBox *a, BoundingBox *b)
{
	const static double DIFF_R = 10;
	const static double DIFF_G = 10;
	const static double DIFF_B = 10;

	double num_similar_pixels = 0;
	double num_total_pixels = 0;

	int dax = a->x2_ - a->x1_;
	int dbx = b->x2_ - b->x1_;
	int dx = (dax < dbx) ? dax : dbx;

	int day = a->y2_ - a->y1_;
	int dby = b->y2_ - b->y1_;
	int dy = (day < dby) ? day : dby;

	double dist = 0;

//	printf("%d %d %d %d %d %d\n", dax, dbx, dx, day, dby, dy);
//	printf("%lf %lf %lf %lf\n", a->x1_, a->y1_, b->x1_, b->y1_);

	for (int i = 0; i < dy; i++)
	{
		for (int j = 0; j < dx; j++)
		{
			int my = a->y1_ + i;
			int mx = a->x1_ + j;

			int ny = b->y1_ + i;
			int nx = b->x1_ + j;

			unsigned char mr = m->data[3 * (my * m->cols + mx) + 2];
			unsigned char mg = m->data[3 * (my * m->cols + mx) + 1];
			unsigned char mb = m->data[3 * (my * m->cols + mx) + 0];

			unsigned char nr = n->data[3 * (ny * n->cols + nx) + 2];
			unsigned char ng = n->data[3 * (ny * n->cols + nx) + 1];
			unsigned char nb = n->data[3 * (ny * n->cols + nx) + 0];

//			if (i < 3 && j < 3)
//				printf("PIXELS: %d %d %d %d | Rs: %d %d COLOR: %d %d %d | %d %d %d | %d %d %d\n", mx, my, nx, ny, 3 * (ny + n->cols + nx) + 2, 3 * (my + m->cols + mx) + 2, mr, mg, mb, nr, ng, nb, mr - nr, mg - ng, mb - nb);

			if (abs(mr - nr) < DIFF_R && abs(mg - ng) < DIFF_G && abs(mb - nb) < DIFF_B)
				num_similar_pixels++;

			dist += sqrt(pow(mr - nr, 2) + pow(mg - ng, 2) + pow(mb - nb, 2));

			num_total_pixels++;
		}
	}

	//return num_similar_pixels / num_total_pixels;
	return dist / (double) num_total_pixels;
}


double
gaussian_random(double mean, double std)
{
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
	return mean + std * z;
}


std::vector<std::pair<int, int> > generate_distro(double distro_radius, int num_points)
{
	std::vector<std::pair<int, int> > distribution;

	for (int i = 0; i < num_points; i++)
		distribution.push_back(
			std::pair<int, int>(
				(int) gaussian_random(0, distro_radius),
				(int) gaussian_random(0, distro_radius)
			)
		);

	return distribution;
}


void
build_bit_pattern(cv::Mat *m, int mx, int my, std::vector<std::pair<int, int> > *distribution, int *bp)
{
	int i;
	int bp_size = distribution->size() / (sizeof(int) * 8);

	for (i = 0; i < bp_size; i++)
		bp[i] = 0;

	unsigned char cr = m->data[3 * (my * m->cols + mx) + 2];
	unsigned char cg = m->data[3 * (my * m->cols + mx) + 1];
	unsigned char cb = m->data[3 * (my * m->cols + mx) + 0];
	unsigned char cgray = (cr + cb + cg) / 3;

	for (i = 0; i < distribution->size(); i++)
	{
		int current_slot = i / (sizeof(int) * 8);

		int px = mx + distribution->at(i).first;
		int py = my + distribution->at(i).second;

		if (px < 0) px = 0;
		if (py < 0) py = 0;
		if (px >= m->cols) px = m->cols - 1;
		if (py >= m->rows) py = m->rows - 1;

		unsigned char pr = m->data[3 * (py * m->cols + px) + 2];
		unsigned char pg = m->data[3 * (py * m->cols + px) + 1];
		unsigned char pb = m->data[3 * (py * m->cols + px) + 0];
		unsigned char pgray = (pr + pb + pg) / 3;

		bp[current_slot] = bp[current_slot] << 1;

		if (pgray > cgray)
			bp[current_slot] = bp[current_slot] | 1;
	}
}


int hamming(int *bpa, int *bpb, int num_points)
{
	int num_slots = num_points / (sizeof(int) * 8) + 1;
	int dist = 0;

	for (int i = 0; i < num_slots; i++)
	{
		for (int j = 0; j < sizeof(int) * 8; j++)
		{
			if ((bpa[i] & 1) != (bpb[i] & 1))
				dist++;

			bpa[i] >>= 1;
			bpb[i] >>= 1;
		}
	}

	return dist;
}


double
structure_based(cv::Mat *m, cv::Mat *n, BoundingBox *a, BoundingBox *b)
{
	const static double DIFF_PATTERN = 64;
	const static double DISTRO_RADIUS = 5;
	const static int NUM_POINTS = 1024;

	static int *bpa = (int*) calloc ((NUM_POINTS / (sizeof(int) * 8) + 1), sizeof(int));
	static int *bpb = (int*) calloc ((NUM_POINTS / (sizeof(int) * 8) + 1), sizeof(int));

	static std::vector<std::pair<int, int> > distribution = generate_distro(DISTRO_RADIUS, NUM_POINTS);

	double num_similar_pixels = 0;
	double num_total_pixels = 0;

	int dax = a->x2_ - a->x1_;
	int dbx = b->x2_ - b->x1_;
	int dx = (dax < dbx) ? dax : dbx;

	int day = a->y2_ - a->y1_;
	int dby = b->y2_ - b->y1_;
	int dy = (day < dby) ? day : dby;

	double dist_total = 0;

	for (int i = 0; i < dy; i++)
	{
		for (int j = 0; j < dx; j++)
		{
			int my = a->y1_ + i;
			int mx = a->x1_ + j;

			int ny = b->y1_ + i;
			int nx = b->x1_ + j;

			build_bit_pattern(m, mx, my, &distribution, bpa);
			build_bit_pattern(n, nx, ny, &distribution, bpb);

			int dist = hamming(bpa, bpb, NUM_POINTS);

			dist_total += dist;

			if (dist < DIFF_PATTERN)
				num_similar_pixels++;

			num_total_pixels++;
		}
	}

	//return num_similar_pixels / num_total_pixels;
	return dist_total / (double) num_total_pixels;
}


int
main(int argc, char *argv[])
{
	std::string model_file = "tracker.prototxt";
	std::string trained_file = "tracker.caffemodel";

	int gpu_id = 0;

	Regressor regressor(model_file, trained_file, gpu_id, false);

	// Ensuring randomness for fairness.
	srandom (time(NULL));

	// Create a tracker object.
	const bool show_intermediate_output = false;
	Tracker tracker(show_intermediate_output);

	//VOT vot; // Initialize the communcation

	// Get region and first frame
	//VOTRegion region = vot.region();
	std::string path; // TODO // = vot.frame();

	// Load the first frame and use the initialization region to initialize the tracker.
	cv::Mat m = cv::imread("lena.jpg");
	cv::Mat mv = cv::imread("lena.jpg");

	BoundingBox b;
	b.x1_ = 246;
	b.y1_ = 250;
	b.x2_ = 294;
	b.y2_ = 285;
	tracker.Init(m, b, &regressor); // TODO

	b.DrawBoundingBox(&mv);

	// Track and estimate the bounding box location.
	BoundingBox bbox_estimate = b;
	double theta = 0;
	int illumination_m = 1;

	for (;;)
	{
		cv::Mat n = m.clone();

		cv::Point center = cv::Point(n.cols / 2, n.rows / 2);
		cv::Mat rot = cv::getRotationMatrix2D(center, theta, 1.0);
		cv::warpAffine(m, n, rot, n.size());

		if (illumination_m > 0)
			n *= illumination_m;
		else if (illumination_m < 0)
			n /= (-illumination_m);

		bbox_estimate.DrawBoundingBox(&n);

//		cv::Mat mb(m, cv::Rect(b.x1_, b.y1_, b.x2_ - b.x1_, b.y2_ - b.y1_));
//		cv::Mat nb(n, cv::Rect(bbox_estimate.x1_, bbox_estimate.y1_, bbox_estimate.x2_ - bbox_estimate.x1_, bbox_estimate.y2_ - bbox_estimate.y1_));

//		double dcolor = color_based(&m, &n, &b, &bbox_estimate);
//		double dstructure = structure_based(&m, &n, &b, &bbox_estimate);

//		cv::imshow("mb", mb);
//		cv::imshow("nb", nb);

//		printf("COLOR: %lf STRUCTURE: %lf\n", dcolor, dstructure);

		cv::imshow("m", mv);
		cv::imshow("n", n);
		char c = cv::waitKey(-1);

		switch (c)
		{
			case 'w':
				bbox_estimate.y1_ -= 2;
				bbox_estimate.y2_ -= 2;
				break;
			case 's':
				bbox_estimate.y1_ += 2;
				bbox_estimate.y2_ += 2;
				break;
			case 'a':
				bbox_estimate.x1_ -= 2;
				bbox_estimate.x2_ -= 2;
				break;
			case 'd':
				bbox_estimate.x1_ += 2;
				bbox_estimate.x2_ += 2;
				break;
			case 't':
				n = m.clone();
				center = cv::Point(n.cols / 2, n.rows / 2);
				rot = cv::getRotationMatrix2D(center, theta, 1.0);
				cv::warpAffine(m, n, rot, n.size());
				if (illumination_m > 0)
					n *= illumination_m;
				else if (illumination_m < 0)
					n /= (-illumination_m);
				tracker.Track(n, &regressor, &bbox_estimate);
				break;
			case 'f':
				theta += 2;
				break;
			case 'r':
				theta -= 2;
				break;
			case 'y':
				illumination_m += 2;
				break;
			case 'h':
				illumination_m -= 2;
				break;
			case 'x':
				bbox_estimate = b;
				break;
			case 'q':
				return 0;
		}
	}

	//vot.report(region); // Report the position of the tracker

	// Finishing the communication is completed automatically with the destruction
	// of the communication object (if you are using pointers you have to explicitly
	// delete the object).

	return 0;
}
