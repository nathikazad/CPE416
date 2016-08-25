#include <stdio.h>
#include <stdlib.h>
#include "Layer.h"

class NeuralNetwork {
public:
	NeuralNetwork(int number_of_inputs, int* layers, int number_of_layers);
	Layer **Layers;
	int number_of_layers;
	int number_of_inputs;
	float* compute(float* inputs);
	void train(float* inputs, float* outputs);
	void debug();
};
