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
	int N = initalImage.size();
	//-------------------------------------
	//        file setup
	//-------------------------------------
	std::ofstream File(output_file);
	File << "t" << "\t";
	for (int i = 0; i < initalImage.size(); i++) //output file setup
	{
		File << "x_" + std::to_string(i) << "\t" << "y_" + std::to_string(i) << "\t" << "z_" + std::to_string(i) << "\t";
	}
	File << "E";
	if (N == 2) {
		File << "\t" << "|j|" << "\t" << "|e|" << "\t" << "a_maj";
	}
	File << "\n";
	File << std::fixed << std::setprecision(2);
	//-------------------------------------
	//            integration
	//-------------------------------------
	std::vector<Body> previousImage = initalImage;
	for (int i = 0; i < iterations; i++)
	{
		
		
		std::vector<Body> newImage = integrate(previousImage, timeStep);

		File << timeStep * i << "\t";
		for (Body b : newImage) {
			File << b.getPosition().getX() << "\t" << b.getPosition().getY() << "\t" << b.getPosition().getZ() << "\t";
			
		}

		//-------------------------------------
		//        conserved quantities
		//-------------------------------------
		double E = calculateEnergy(newImage, N);
		File << E << "\t";
		if (N == 2) {
			Vector j = calculateAngularMomentum(newImage);
			Vector e = calulateRungeLenz(newImage, j);
			double a = calculateMajorSemiAxis(j, e);
			File << j.getLength() << "\t" << e.getLength() << "\t" << a;
		}

		 
		


		File << "\n";
		previousImage = newImage; //prepare for next iteration

		
	}
	File.close();

}
