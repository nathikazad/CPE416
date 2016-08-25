#include "Neuron.h"


class Layer
{
	public:
		Layer(int number_of_inputs, int number_of_neurons);
		int number_of_inputs;
		int number_of_neurons;
		Neuron **Neurons;
		float *output_vector;
		float *dirac_vector;
		float* compute(float* inputs);
		void train_as_output(float* inputs, float* outputs);
		void train_as_hidden(float* inputs, Layer* after);
		void debug();
};