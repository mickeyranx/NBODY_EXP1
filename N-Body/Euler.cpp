#include "Euler.h"
#include <iostream>
//only works in C++ version >= 17
#include <variant>





std::vector<Body> Euler::integrate(std::vector<Body> bodies, double timeStep)
{



	int N = bodies.size();
	
	std::vector<Body> newImage; 
	for (int i = 0; i < N; i++)
	{
		Customvectors::Vector a = Euler::calculateAcceleration(bodies, bodies[i], N, i);
		std::cout << "a = " << a.getLength() << std::endl;
		Body current_body = bodies[i];
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();
		Customvectors::Vector v_new = v_n + a * timeStep;
		Customvectors::Vector r_new = r_n + v_n * timeStep;
		//add Body to newImage list with new position and velocity vectors
		newImage.push_back(Body(current_body.getMass(), r_new, v_new));
		//deconstruct old body?
	}

	return newImage;
}

void Euler::startIntegration(std::vector<Body> initalImage, double timeStep, int iterations,std::string output_file)
{
	std::ofstream File(output_file);
	File << "t" << "\t";
	for (int i = 0; i < initalImage.size(); i++) //output file setup
	{
		File << "x_" + std::to_string(i) << "\t" << "y_" + std::to_string(i) << "\t" << "z_" + std::to_string(i) << "\t";
	}
	File << "\n";
	File << std::fixed << std::setprecision(2);
	std::vector<Body> previousImage = initalImage;
	for (int i = 0; i < iterations; i++)
	{
		
		
		std::vector<Body> newImage = integrate(previousImage, timeStep);

		File << timeStep * i << "\t";
		for (Body b : newImage) {
			File << b.getPosition().getX() << "\t" << b.getPosition().getY() << "\t" << b.getPosition().getZ() << "\t";
			
		}
		File << "\n";
		previousImage = newImage;

		
	}
	File.close();

}
