#include "EulerChromer.h"
#include <iostream>

std::vector<Body> EulerChromer::integrate(std::vector<Body> bodies, double timeStep)
{

	int N = bodies.size();

	std::vector<Body> newImage;
	for (int i = 0; i < N; i++)
	{
		Customvectors::Vector a = EulerChromer::calculateAcceleration(bodies, bodies[i], N, i);
		std::cout << "a = " << a.getLength() << std::endl;
		Body current_body = bodies[i];
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();
		Customvectors::Vector v_new = v_n + a * timeStep;
		Customvectors::Vector r_new = r_n + v_new * timeStep;
		//add Body to newImage list with new position and velocity vectors
		newImage.push_back(Body(current_body.getMass(), r_new, v_new));
		//deconstruct old body?
	}

	return newImage;
}

void EulerChromer::startIntegration(std::vector<Body> initalImage, double timeStep, int iterations, std::string outputPath)
{


	std::vector<Body> previousImage = initalImage;
	for (int i = 0; i < iterations; i++)
	{
		std::cout << "next Iteration:" << std::endl;

		std::vector<Body> newImage = integrate(previousImage, timeStep);


		for (Body b : newImage) {
			std::cout << b.getPosition().getX() << std::endl;
			std::cout << b.getPosition().getY() << std::endl;
			std::cout << b.getPosition().getZ() << std::endl;
		}

		previousImage = newImage;


	}


}
