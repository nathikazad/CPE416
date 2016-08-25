#include "Layer.h"
Layer::Layer(int number_of_inputs, int number_of_neurons)
{
	Neurons=new Neuron*[number_of_neurons];
	this->number_of_inputs=number_of_inputs;
	this->number_of_neurons=number_of_neurons;
	this->output_vector=new float[number_of_neurons];
	this->dirac_vector=new float[number_of_neurons];
	for(int i=0;i<number_of_neurons;i++)
	{
		Neurons[i]=new Neuron(number_of_inputs);
	}
}
float* Layer::compute(float* inputs)
{
	for(int i=0;i<number_of_neurons;i++)
		output_vector[i]=Neurons[i]->compute(inputs);
	return output_vector;
}
void Layer::train_as_output(float* inputs, float* desired_outputs)
{
	for(int i=0;i<number_of_neurons;i++)
	{
		// printf("Neuron:%d \n",i);
		dirac_vector[i]=Neurons[i]->train_as_output(inputs,output_vector[i],desired_outputs[i]);
	}
}
void Layer::train_as_hidden(float* inputs, Layer* after)
{
	for(int i=0;i<number_of_neurons;i++)
	{
		// printf("Neuron:%d \n",i);
		float dirac_sum=0;
		for(int j=0;j<after->number_of_neurons;j++)
		{
			dirac_sum+=after->dirac_vector[j]*after->Neurons[j]->old_weights[i]; //not sure, check again
		}
		dirac_vector[i]=Neurons[i]->train_as_hidden(inputs, output_vector[i],dirac_sum);
	}
}
void Layer::debug()
{
	// printf("\tNumber of Neurons:%d\n",number_of_neurons);
	// printf("\tNumber of Inputs:%d\n",number_of_inputs);
	for(int j=0;j<number_of_neurons;j++)
	{
		printf("\tNeuron %d",j);
		printf("\t Output:%f", output_vector[j]);
		printf("\tDirac:%f\n", dirac_vector[j]);
		Neurons[j]->debug();
	}

}