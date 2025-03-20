#include "NbodyIntegrator.h"
#include <iostream>
#include <fstream>

using namespace Customvectors;


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


double NbodyIntegrator::calculateEnergy(std::vector<Body> &bodies, int N) {
	
	double E = 0;
	for (int i = 0; i < N; i++)
	{
		Body b1 = bodies[i];
		double m1 = b1.getMass();
		Vector v1 = b1.getVelocity();
		
		E += 0.5 * m1 * (v1 * v1); //kinetic part
		for (int j = i + 1; j < N; j++) 
		{
			Body b2 = bodies[j];
			E += -1 * m1 * b2.getMass() / (b1.getPosition() - b2.getPosition()).getLength(); //potential part, G is set to 1
			
		}



	}

	return E;

}


Customvectors::Vector NbodyIntegrator::calculateAngularMomentum(std::vector<Body>& bodies) {
	Body b1 = bodies[0];
	Body b2 = bodies[1];
	double m1 = b1.getMass();
	double m2 = b2.getMass();
	Vector r1 = b1.getPosition();
	Vector r2 = b2.getPosition();
	Vector v1 = b1.getVelocity();
	Vector v2 = b2.getVelocity();
	//reduced mass
	double mu = m1 * m2 / (m1 + m2);
	//relative position and velocity
	Vector r = r2 - r1;
	Vector v = v2 - v1;
	// % is vector product
	return (r % v) * mu;

}


Customvectors::Vector NbodyIntegrator::calulateRungeLenz(std::vector<Body>& bodies, Customvectors::Vector j) {
	Body b1 = bodies[0];
	Body b2 = bodies[1];
	double m1 = b1.getMass();
	double m2 = b2.getMass();
	Vector r1 = b1.getPosition();
	Vector r2 = b2.getPosition();
	Vector v1 = b1.getVelocity();
	Vector v2 = b2.getVelocity();
	Vector r = r2 - r1;
	//Runge Lenz vector
	Vector e = ((v2 - v1) % j) - r * (1/ r.getLength());
	return e;
}

double NbodyIntegrator::calculateMajorSemiAxis(Customvectors::Vector j, Customvectors::Vector e) {

	return (j * j) / (1 - e * e);

}
