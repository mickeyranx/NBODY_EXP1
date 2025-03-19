#include "NbodyIntegrator.h"
#include <iostream>
#include <fstream>




NbodyIntegrator::NbodyIntegrator(TimeStep timeStepFunction)
{
	tsf = timeStepFunction;

}


//this function will be overriden by the child classes and will never be used
void NbodyIntegrator::startIntegration(std::vector<Body> bodies, double timeStep, int Iterations, std::string outputPath)
{
	
	std::cout << "test" << std::endl;

}

Customvectors::Vector NbodyIntegrator::calculateAcceleration(std::vector<Body> bodies, Body reference_body, int N, int j) {
	Customvectors::Vector a = Customvectors::Vector(0,0,0);

	//iterate over all bodies
	for (int i = 0; i < N; i++)
	{
		//no self-interaction 
		if (i == j) {
			continue;
		}
		else {
			Body current_body = bodies[i];
			Customvectors::Vector r_ij = current_body.getPosition() - reference_body.getPosition();
			//calculate acceleration on body j
			a = a + r_ij * (1 / r_ij.getLength()) * current_body.getMass();

		}



	}
	return a;
}



Customvectors::Vector NbodyIntegrator::calculateJerk(std::vector<Body> bodies, Body reference_body, int N, int j) {
	Customvectors::Vector a_dot = Customvectors::Vector(0, 0, 0);
	//iterate over all bodies
	for (int i = 0; i < N; i++)
	{
		//no self-interaction 
		if (i == j) {
			continue;
		}
		else {
			Body current_body = bodies[i];
			Customvectors::Vector r_ij = current_body.getPosition() - reference_body.getPosition();
			Customvectors::Vector v_ij = current_body.getVelocity() - reference_body.getVelocity();
			double r_len = r_ij.getLength();

			//calculate acceleration on body j
			a_dot = a_dot + (v_ij * pow((1 / r_len), 2)) + (r_ij * (1 / pow(r_len, 5)) * (r_ij * v_ij) * 3);
		}



	}
	return a_dot;
}





