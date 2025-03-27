#include "RK4.h"
#include <iostream>


std::vector<Body> RK4::integrate(std::vector<Body> bodies, double dt)
{
	int N = bodies.size();

	std::vector<Body> new_image = bodies;
	std::vector<Body> new_temp_image_1 = {};
	std::vector<Body> new_temp_image_2 = {};

	for (int i = 0; i < N; i++)
	{
		Body current_body = bodies[i];
		Customvectors::Vector a_n = calculateAcceleration(bodies, current_body, N, i);
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();

		Customvectors::Vector v_1 = a_n * dt;
		Customvectors::Vector r_1 = v_n * dt;
		new_image[i].alterPosition(r_1*(1.0 / 6.0)); //add first order term
		new_image[i].alterVelocity(v_1*(1.0 / 6.0)); //add first order term
		current_body.setPosition(r_n + r_1 * 0.5);
		new_temp_image_1.push_back(current_body); //for calculation of v_2
		Customvectors::Vector r_2 = (v_n + v_1 * 0.5) * dt;
		new_image[i].alterPosition(r_2 * (1.0 / 3.0)); //add second order term
		current_body.setPosition(r_n + r_2 * 0.5);
		new_temp_image_2.push_back(current_body); //for calculation of v_3
		
		

	}
	std::vector<Body> new_temp_image_3 = {};
	for (int i = 0; i < N; i++)
	{
		Body current_body_1 = new_temp_image_1[i];
		Body old_body = bodies[i];
		Customvectors::Vector v_n = old_body.getVelocity();
		Customvectors::Vector r_n = old_body.getPosition();
		Customvectors::Vector v_2 = calculateAcceleration(new_temp_image_1, current_body_1, N, i) * dt;
		new_image[i].alterVelocity(v_2 * (1.0 / 3.0)); //add second order term
		Customvectors::Vector r_3 = (v_n + v_2 * 0.5) * dt;
		new_image[i].alterPosition(r_3 * (1.0 / 3.0)); //add third order term
		Body current_body_2 = new_temp_image_2[i];
		Customvectors::Vector v_3 = calculateAcceleration(new_temp_image_2, current_body_2, N, i) * dt;
		new_image[i].alterVelocity(v_3 * (1.0 / 3.0)); //add third order term
		Customvectors::Vector r_4 = (v_n + v_3) * dt;
		new_image[i].alterPosition(r_4 * (1.0 / 6.0)); //add fourth order term
		old_body.setPosition(r_n + r_3);
		new_temp_image_3.push_back(old_body);
		
	}

	for (int i = 0; i < N; i++)
	{
		Customvectors::Vector v_4 = calculateAcceleration(new_temp_image_3, new_temp_image_3[i], N, i) * dt;
		new_image[i].alterVelocity(v_4 * (1.0 / 6.0)); //add fourth order term
	}

	return new_image;
}

void RK4::startIntegration(std::vector<Body> initalImage, double eta, int iterations, std::string output_file)
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
