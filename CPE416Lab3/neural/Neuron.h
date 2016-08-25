#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#define ALPHA 0.01
class Neuron
{
	public:
		Neuron(int number_of_inputs);
		float *weights;
		float *old_weights;
		int number_of_inputs;
		float compute(float* inputs);
		float train_as_output(float* inputs, float actual_output, float desired_output);
		float train_as_hidden(float* inputs, float actual_output, float dirac_weight_sum);
		void debug();
};