
#ifndef _TLIGHT_VGRAM_H_
#define _TLIGHT_VGRAM_H_

#include <vector>
#include <map>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wnn/VgRamNeuron.h>
#include <carmen/tlight_state_recog.h>

using namespace std;
using namespace cv;


class Data
{
public:
	char image_path[1024];
	Rect roi;
};


class TLightStatus : public VgRamNeuronOutput
{
public:
	int status;

	TLightStatus(int s = -1) { status = s; }
	virtual ~TLightStatus() {}
	virtual int save(FILE *f) { return fprintf(f, "%d ", status); }
	virtual int load(FILE *f) { return fscanf(f, "%d ", &status); }
};

const char* label_to_string(int label);
void check_image_exists(char *filename);
Mat* preproc_image(Mat &img, Rect img_roi, int TLIGHT_COLS, int TLIGHT_ROWS);
Mat* preproc(Data *d, int TLIGHT_COLS, int TLIGHT_ROWS);

class TLightVgRam : public TLightRecogInterface
{
public:

	// defines how the net will deal with color
	enum ColorOption
	{
		UseGray = 0,
		UseGrayWithoutBlue,
		UseRedAsHighOrder,
		UseBlueAsHighOrder,
		UseRedAsHighAndDiscardBlue,
		UseGreenAsHighAndDiscardBlue
	};

	TLightVgRam();
	TLightVgRam(VgRamNeuronConfig *config, int num_neurons, int num_synapsis, ColorOption color_option = UseGreenAsHighAndDiscardBlue);

	void Train(Mat *tlight, int status);
	void Forward(Mat *tlight, int *status, double *confidence);

	virtual int run(cv::Mat &m, cv::Rect &r)
	{
		int label;

		Mat* p = preproc_image(m, r, TRAFFIC_SIGN_WIDTH, TRAFFIC_SIGN_HEIGHT);
		Forward(p, &label, NULL);

		return label;
	}

	void Save(const char *filename);
	int MemOccupation() { return ensemble_[0]->NumTrainedPatterns(); }

protected:

	VgRamNeuronConfig *config_;
	std::vector<VgRamNeuron*> ensemble_;
	std::vector<std::vector<pair<double, double>>> synaptic_distributions_; // pairs x, y in the interval [0.0, 1.0]

	int max_neurons_;
	int num_synapsis_;
	ColorOption color_option_;

	void SaveSynapticDistribution(FILE *f);
	void LoadSynapticDistribution(FILE *f);

	void InitSynapticDistribution();
	void InitNeurons();

	unsigned int GetPixelIntensity(Mat *m, int y, int x);
	BitPattern* TLightToBitPattern(Mat *tlight, int neuron_id);
};



#endif
