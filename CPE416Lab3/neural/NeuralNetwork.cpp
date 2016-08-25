#include "NeuralNetwork.h"

void * operator new(size_t size) {
	return malloc(size);
}

void operator delete(void * ptr) {
	free(ptr);
}

void * operator new[](size_t size) {
	return malloc(size);
}

void operator delete[](void * ptr) {
	free(ptr);
}

NeuralNetwork::NeuralNetwork(int number_of_inputs, int* neurons_in_layer, int number_of_layers) {
	Layers = new Layer*[number_of_layers];
	this->number_of_inputs = number_of_inputs;
	this->number_of_layers = number_of_layers;
	for (int i = 0; i < number_of_layers; i++) {
		int number_of_neurons = neurons_in_layer[i];
		Layers[i] = new Layer(number_of_inputs, number_of_neurons);
		number_of_inputs = neurons_in_layer[i];
	}
}
float* NeuralNetwork::compute(float* inputs) {
	for (int i = 0; i < number_of_layers; i++) {
		inputs = Layers[i]->compute(inputs);
	}
	return inputs;

}
void NeuralNetwork::train(float* inputs, float* outputs) {
	// Layers[number_of_layers-1]->train_as_output(Layers[number_of_layers-2]->output_vector, outputs);
	Layers[1]->train_as_output(Layers[0]->output_vector, outputs);
	Layers[0]->train_as_hidden(inputs, Layers[1]);
}

void NeuralNetwork::debug() {
	// printf("Number of Layers:%d\n",number_of_layers);
	// printf("Number of Inputs:%d\n",number_of_inputs);
	for (int i = 0; i < number_of_layers; i++) {
		printf("Layer:%d\n", i);
		Layers[i]->debug();
	}
}
