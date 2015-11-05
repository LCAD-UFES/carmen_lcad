/*
 * neuron.h
 *
 *  Created on: 19/10/2012
 *      Author: _filipe
 */

#ifndef _NN_LIB_NEURON_H_
#define _NN_LIB_NEURON_H_

#include <vector>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

namespace nn_lib
{
	namespace neuron
	{
		template<class OutputClass>
		class Neuron
		{
			public:
				Neuron(){}
				~Neuron(){}
				virtual void train(vector<float>, OutputClass) = 0;
				virtual OutputClass test(vector<float>) = 0;
		};

		template<class OutputClass>
		class NeuronVGRAM : Neuron<OutputClass>
		{
			vector<pair<vector<float>, OutputClass> > neuron_memory;

			double
			calculate_hamming_distance(vector<float> a, vector<float> b)
			{
				int i;
				double hamming_dist = 0;

				if (a.size() != b.size())
					exit(printf("Error: Trying to calculate hamming distance between two vectors with different size\n"));

				for(i = 0; i < (int) a.size(); i++)
					hamming_dist += fabs(a[i] - b[i]);

				return hamming_dist;
			}

			public:

				NeuronVGRAM()
				{
				}


				~NeuronVGRAM()
				{
				}


				void
				train(vector<float> input_vector, OutputClass expected_output)
				{
					pair<vector<float>, OutputClass> neuron_memory_item;

					neuron_memory_item.first = input_vector;
					neuron_memory_item.second = expected_output;

					neuron_memory.push_back(neuron_memory_item);
				}


				OutputClass
				test(vector<float> input_vector)
				{
					int i, is_first = 1, min_hamming_dist_position = 0;
					double hamming_dist = 0, min_hamming_dist = 0;

					if (neuron_memory.size() == 0)
						exit(printf("Error: Trying to test an empty neuron\n"));

					for(i = 0; i < (int) neuron_memory.size(); i++)
					{
						hamming_dist = calculate_hamming_distance(input_vector, neuron_memory[i].first);

						if (is_first)
						{
							min_hamming_dist = hamming_dist;
							is_first = 0;
						}
						else
						{
							// TODO: save vectors with equals hamming dist to choose randomly between them
							if (hamming_dist < min_hamming_dist)
							{
								min_hamming_dist = hamming_dist;
								min_hamming_dist_position = i;
							}
						}
					}

					return neuron_memory[min_hamming_dist_position].second;
				}
		};

	}

}

#endif // _NN_LIB_NEURON_H_

// @
