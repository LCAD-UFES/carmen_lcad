/*
 * simple_neuron_vgram_classification.cpp
 *
 *  Created on: 18/10/2012
 *      Author: _filipe
 */

#include "neuron.h"
#include <stdio.h>
#include <string>

using namespace std;

int
main(void)
{
	string output;
	nn_lib::neuron::NeuronVGRAM<string> neuron;

	// initialize input_vector_1 with 4 elements 0
	vector<float> input_vector_1(4, 0);
	// initialize input_vector_2 with 4 elements 1
	vector<float> input_vector_2(4, 1);
	// initialize input_vector_3 with 4 elements 0
	vector<float> input_vector_3(4, 0);

	// add some noise to input_vector_3
	input_vector_3[0] = 1.0;

	neuron.train(input_vector_1, "class 1");
	neuron.train(input_vector_2, "class 2");

	output = neuron.test(input_vector_3);
	printf("neuron answer: %s\n", output.c_str());

	printf("Terminou!\n");
	return 0;
}
