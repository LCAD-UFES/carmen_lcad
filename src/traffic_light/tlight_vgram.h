
#ifndef _TLIGHT_VGRAM_H_
#define _TLIGHT_VGRAM_H_


#include <vector>
#include <map>
#include <opencv/cv.h>
#include <wnn/VgRamNeuron.h>


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


class TLightVgRam
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

	TLightVgRam(const char *net_filename);
	TLightVgRam(VgRamNeuronConfig *config, int num_neurons, int num_synapsis, ColorOption color_option = UseGreenAsHighAndDiscardBlue);

	void Train(Mat *tlight, int status);
	void Forward(Mat *tlight, int *status, double *confidence);

	void Save(const char *filename);
	int MemOccupation() { return ensemble_[0]->NumTrainedPatterns(); }

protected:

	VgRamNeuronConfig *config_;
	vector<VgRamNeuron*> ensemble_;
	vector<vector<pair<double, double>>> synaptic_distributions_; // pairs x, y in the interval [0.0, 1.0]

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

const char* label_to_string(int label);
void check_image_exists(char *filename);
Mat* preproc_image(Mat &img, Rect img_roi, int TLIGHT_COLS, int TLIGHT_ROWS);
Mat* preproc(Data *d, int TLIGHT_COLS, int TLIGHT_ROWS);


#endif
