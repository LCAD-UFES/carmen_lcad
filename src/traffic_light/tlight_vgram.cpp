
#include "tlight_vgram.h"
#include <opencv/highgui.h>


void
TLightVgRam::SaveSynapticDistribution(FILE *f)
{
	for (int i = 0; i < max_neurons_; i++)
	{
		for (int j = 0; j < num_synapsis_; j++)
			fprintf(f, "%lf %lf ", synaptic_distributions_[i][j].first, synaptic_distributions_[i][j].second);
		fprintf(f, "\n");
	}
}


void
TLightVgRam::LoadSynapticDistribution(FILE *f)
{
	double x, y;
	synaptic_distributions_.clear();

	for (int i = 0; i < max_neurons_; i++)
	{
		vector<pair<double, double>> v;

		for (int j = 0; j < num_synapsis_; j++)
		{
			fscanf(f, "%lf %lf ", &x, &y);
			v.push_back(pair<double, double>(x, y));
		}

		synaptic_distributions_.push_back(v);
	}
}


void
TLightVgRam::InitSynapticDistribution()
{
	double x, y;

	for (int i = 0; i < max_neurons_; i++)
	{
		vector<pair<double, double>> v;

		for (int j = 0; j < num_synapsis_; j++)
		{
			x = (double) rand() / (double) RAND_MAX;
			y = (double) rand() / (double) RAND_MAX;

			v.push_back(pair<double, double>(x, y));
		}

		synaptic_distributions_.push_back(v);
	}
}


unsigned int
TLightVgRam::GetPixelIntensity(Mat *m, int y, int x)
{
	unsigned int intensity = 0;

	if (m->channels() == 3)
	{
		unsigned int b = (unsigned int) m->data[3 * (y * m->cols + x) + 0];
		unsigned int g = (unsigned int) m->data[3 * (y * m->cols + x) + 1];
		unsigned int r = (unsigned int) m->data[3 * (y * m->cols + x) + 2];

		if (color_option_ == ColorOption::UseBlueAsHighOrder)
			intensity = (((r & 0x000000ff) << 0) | ((g & 0x000000ff) << 8) | ((b & 0x000000ff) << 16));
		else if (color_option_ == ColorOption::UseRedAsHighOrder)
			intensity = (((b & 0x000000ff) << 0) | ((g & 0x000000ff) << 8) | ((r & 0x000000ff) << 16));
		else if (color_option_ == ColorOption::UseRedAsHighAndDiscardBlue)
			intensity = (((g & 0x000000ff) << 0) | ((r & 0x000000ff) << 8));
		else if (color_option_ == UseGreenAsHighAndDiscardBlue)
			intensity = (((r & 0x000000ff) << 0) | ((g & 0x000000ff) << 8));
		else if (color_option_ == ColorOption::UseGray)
			intensity = (b + g + r) / 3;
		else if (color_option_ == ColorOption::UseGrayWithoutBlue)
			intensity = (g + r) / 3;
	}
	else
		intensity = (unsigned int) (m->data[y * m->cols + x]);

	return intensity;
}


BitPattern*
TLightVgRam::TLightToBitPattern(Mat *tlight, int neuron_id)
{
	BitPattern *b = new BitPattern(num_synapsis_);

	int x, y;
	int nx, ny;
	int p, np;
	unsigned char c, nc;

	//int z = 5;
	//Mat view(Size(tlight->cols * z, tlight->rows * z), tlight->type());
	//resize(*tlight, view, view.size());

	for (int j = 0; j < num_synapsis_; j++)
	{
		if (j < num_synapsis_ - 1)
		{
			p = j;
			np = j + 1;
		}
		else
		{
			p = j;
			np = 0;
		}

		x = synaptic_distributions_[neuron_id][p].first * tlight->cols;
		y = synaptic_distributions_[neuron_id][p].second * tlight->rows;

		nx = synaptic_distributions_[neuron_id][np].first * tlight->cols;
		ny = synaptic_distributions_[neuron_id][np].second * tlight->rows;

		c = GetPixelIntensity(tlight, y, x); //tlight->data[y * tlight->cols + x];
		nc = GetPixelIntensity(tlight, ny, nx); //tlight->data[ny * tlight->cols + nx];

		//cv::circle(view, cv::Point(x * z, y * z), 2, Scalar(255,0,0), 1);

		if (c > (nc + 10))
			b->set(j);
	}

	//cv::imshow("syns", view);
	//cv::waitKey(-1);

	return b;
}


void
TLightVgRam::InitNeurons()
{
	for (int i = 0; i < max_neurons_; i++)
		ensemble_.push_back(new VgRamNeuron(*config_));
}

//FILE *f = fopen(filename, "w");
//
//if (f == NULL)
//	exit(printf("Error::Unable to open file '%s'\n", filename));
//
//config_->save(f);
//
//fprintf(f, "%d %d %d ", max_neurons_, num_synapsis_, color_option_);
//fprintf(f, "%d ", ensemble_.size());
//
//SaveSynapticDistribution(f);
//
//for (size_t i = 0; i < ensemble_.size(); i++)
//	ensemble_[i]->save(f);
//
//fclose(f);

TLightVgRam::TLightVgRam(const char *filename)
{
	FILE *f = fopen(filename, "r");

	if (f == NULL)
		exit(printf("Error::Unable to open file '%s'\n", filename));

	config_ = new VgRamNeuronConfig();
	config_->load(f);

	int ensemble_size;

	fscanf(f, "%d %d %d", &max_neurons_, &num_synapsis_, (int*) &color_option_);
	fscanf(f, "%d", &ensemble_size);

	LoadSynapticDistribution(f);

	for (int i = 0; i < ensemble_size; i++)
	{
		ensemble_.push_back(new VgRamNeuron());
		ensemble_[i]->load<TLightStatus>(f);
	}

	fclose(f);
}


TLightVgRam::TLightVgRam(VgRamNeuronConfig *config, int num_neurons, int num_synapsis, ColorOption color_option)
{
	config_ = config;
	max_neurons_ = num_neurons;
	num_synapsis_ = num_synapsis;
	color_option_ = color_option;

	InitSynapticDistribution();
	InitNeurons();
}


void
TLightVgRam::Train(Mat *tlight, int status)
{
	for (size_t i = 0; i < ensemble_.size(); i++)
		ensemble_[i]->train(TLightToBitPattern(tlight, i), new TLightStatus(status));
}


void
TLightVgRam::Forward(Mat *tlight, int *status, double *confidence)
{
	map<int, int> votes;

	for (size_t i = 0; i < ensemble_.size(); i++)
	{
		BitPattern *b = TLightToBitPattern(tlight, i);
		VgRamNeuronResult result = ensemble_[i]->nearests(b);
		TLightStatus *output = (TLightStatus*) result.values[rand() % result.values.size()];
		int st = output->status;

		if (votes.find(st) != votes.end())
			votes[st]++;
		else
			votes[st] = 1;

		delete(b);
	}

	vector<int> winners;
	map<int, int>::iterator it;

	winners.push_back(votes.begin()->first);

	for (it = votes.begin(); it != votes.end(); it++)
	{
		if (it->second > votes[winners[0]])
		{
			winners.clear();
			winners.push_back(it->first);
		}
		else if (it->second > votes[winners[0]])
			winners.push_back(it->first);
	}

	*status = winners[rand() % winners.size()];
	*confidence = 1.0;
}


void
TLightVgRam::Save(const char *filename)
{
	FILE *f = fopen(filename, "w");

	if (f == NULL)
		exit(printf("Error::Unable to open file '%s'\n", filename));

	config_->save(f);

	fprintf(f, "%d %d %d ", max_neurons_, num_synapsis_, color_option_);
	fprintf(f, "%d ", (int) ensemble_.size());

	SaveSynapticDistribution(f);

	for (size_t i = 0; i < ensemble_.size(); i++)
		ensemble_[i]->save(f);

	fclose(f);
}


const char*
label_to_string(int label)
{
	if (label == 0)
		return "red";
	else if (label == 1)
		return "green";
	else if (label == 2)
		return "yellow";
	else
		return "off";
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
preproc_image(Mat &img, Rect img_roi, int TLIGHT_COLS, int TLIGHT_ROWS)
{
	static Mat *m = NULL;

	if (m == NULL)
		m = new Mat(Size(TLIGHT_COLS, TLIGHT_ROWS), CV_8UC3);

#if 0

	Mat roi = img(roi).clone();
	resize(roi, *m, m->size());

#elif 1

	Mat roi = img(img_roi).clone();

	for (int i = 0; i < roi.rows; i++)
	{
		for (int j = 0; j < roi.cols; j++)
		{
			unsigned char r, g, b;

			b = (unsigned char) (roi.data[3 * (i * roi.cols + j) + 0]);
			g = (unsigned char) (roi.data[3 * (i * roi.cols + j) + 1]);
			r = (unsigned char) (roi.data[3 * (i * roi.cols + j) + 2]);

			//if ((r < 100 && g < 100) || (b > 100 && g < 150))
			if ((r < 100 && g < 100) || (b > 150 && g < 150))
			{
				roi.data[3 * (i * roi.cols + j)+0]=0;
				roi.data[3 * (i * roi.cols + j)+1]=0;
				roi.data[3 * (i * roi.cols + j)+2]=0;
			}
//			else
//			{
//				roi.data[3 * (i * roi.cols + j)+0]=255;
//				roi.data[3 * (i * roi.cols + j)+1]=255;
//				roi.data[3 * (i * roi.cols + j)+2]=255;
//			}
		}
	}

	//imshow("roi", roi);
	//waitKey(1);

	resize(roi, *m, m->size());
	GaussianBlur(*m, *m, Size(3, 3), 1);

#elif 0

	Mat eq;
	vector<Mat> channels;

	cvtColor(img, eq, CV_BGR2YCrCb);
	split(eq, channels);
	equalizeHist(channels[0], channels[0]);
	merge(channels, eq);
	cvtColor(eq, eq, CV_YCrCb2BGR);

	Mat roi = eq(img_roi).clone();
	resize(roi, *m, m->size());
	GaussianBlur(*m, *m, Size(3, 3), 1);

#endif

//	resize(roi, *m, m->size());
//	imshow("roi", *m);


//	imshow("eq", *m);
//	waitKey(0);

	return m;
}


Mat*
preproc(Data *d, int TLIGHT_COLS, int TLIGHT_ROWS)
{
	check_image_exists(d->image_path);
	Mat img = imread(d->image_path);
	return preproc_image(img, d->roi, TLIGHT_COLS, TLIGHT_ROWS);
}
