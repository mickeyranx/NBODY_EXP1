#include "VelocityVerlet.h"
#include <iostream>
//only works in C++ version >= 17
#include <variant>





std::vector<Body> VelocityVerlet::integrate(std::vector<Body> bodies, double time_step)
{
	int N = bodies.size();

	std::vector<Body> new_temp_image = {};
	std::vector<Customvectors::Vector> a_ns = {};

	for (int i = 0; i < N; i++)
	{
		Body current_body = bodies[i];
		Customvectors::Vector a_n = calculateAcceleration(bodies, current_body, N, i);
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();
		Customvectors::Vector r_nn = r_n + v_n * time_step + a_n * 0.5 * time_step * time_step;
		current_body.setPosition(r_nn);
		a_ns.push_back(a_n);
		new_temp_image.push_back(current_body);
		
	}
	std::vector<Body> newImage = {};
	for (int i = 0; i < N; i++)
	{
		Body current_body = new_temp_image[i];
		Customvectors::Vector a_nn = calculateAcceleration(new_temp_image, current_body, N, i);
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector v_nn = v_n + (a_ns[i] + a_nn) * time_step;
		current_body.setVelocity(v_nn);
		newImage.push_back(current_body);
	}

	return newImage;
}

void VelocityVerlet::startIntegration(std::vector<Body> initalImage, double eta, int iterations, std::string output_file)
{
	int N = initalImage.size();
	double time_step = eta;
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


		std::vector<Body> newImage = integrate(previousImage, time_step);

		File << time_step * i << "\t";
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
		//--------------------------------------
		//     calculate next time step
		//--------------------------------------

		switch (getTimeStep()) {
		case TimeStep::LINEAR:
			break;
		case TimeStep::QUADRATIC:
			break;
		case TimeStep::CURVATURE:
			time_step = timeStepCurvature(newImage, N, time_step);
			break;
		}



		previousImage = newImage; //prepare for next iteration


	}
	File.close();

}
