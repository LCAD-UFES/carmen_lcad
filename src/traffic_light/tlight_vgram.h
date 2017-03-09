
#ifndef _TLIGHT_VGRAM_H_
#define _TLIGHT_VGRAM_H_


#include <vector>
#include <map>
#include <opencv/cv.h>
#include <wnn/VgRamNeuron.h>


using namespace std;
using namespace cv;


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
	VgRamNeuronConfig *config_;
	// pairs x, y in the interval [0.0, 1.0]
	vector<vector<pair<double, double>>> synaptic_distributions_;
	vector<VgRamNeuron*> ensemble_;
	int max_neurons_;
	int num_synapsis_;

	void SaveSynapticDistribution(FILE *f)
	{
		for (int i = 0; i < max_neurons_; i++)
		{
			for (size_t j = 0; j < synaptic_distributions_.size(); j++)
				fprintf(f, "%lf %lf ", synaptic_distributions_[i][j].first, synaptic_distributions_[i][j].second);
		}
	}

	void LoadSynapticDistribution(FILE *f)
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

	void InitSynapticDistribution()
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

	BitPattern* TLightToBitPattern(Mat *tlight, int neuron_id)
	{
		BitPattern *b = new BitPattern(num_synapsis_);

		int x, y;
		int nx, ny;
		int p, np;
		unsigned char c, nc;

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

			c = tlight->data[y * tlight->cols + x];
			nc = tlight->data[ny * tlight->cols + nx];

			if (c > nc)
				b->set(j);
		}

		return b;
	}

	void InitNeurons()
	{
		for (int i = 0; i < max_neurons_; i++)
			ensemble_.push_back(new VgRamNeuron(*config_));
	}

public:

	TLightVgRam(VgRamNeuronConfig *config, int num_neurons, int num_synapsis)
	{
		config_ = config;
		max_neurons_ = num_neurons;
		num_synapsis_ = num_synapsis;
		InitSynapticDistribution();
		InitNeurons();
	}

	void Train(Mat *tlight, int status)
	{
		for (size_t i = 0; i < ensemble_.size(); i++)
			ensemble_[i]->train(TLightToBitPattern(tlight, i), new TLightStatus(status));
	}

	void Forward(Mat *tlight, int *status, double *confidence)
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

	void Save(const char *filename)
	{
		FILE *f = fopen(filename, "w");

		if (f == NULL)
			exit(printf("Error::Unable to open file '%s'\n", filename));

		fprintf(f, "%ld", ensemble_.size());

		SaveSynapticDistribution(f);

		for (size_t i = 0; i < ensemble_.size(); i++)
			ensemble_[i]->save(f);

		fclose(f);
	}

	void Load(const char *filename)
	{
		FILE *f = fopen(filename, "r");

		if (f == NULL)
			exit(printf("Error::Unable to open file '%s'\n", filename));

		fscanf(f, "%d", &max_neurons_);

		LoadSynapticDistribution(f);

		for (int i = 0; i < max_neurons_; i++)
			ensemble_[i]->load<TLightStatus>(f);

		fclose(f);
	}
};


#endif
