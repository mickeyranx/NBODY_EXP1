#include "IteratedHermit.h"
#include <iostream>
//only works in C++ version >= 17
#include <variant>





std::vector<Body> IteratedHermit::integrate(std::vector<Body> bodies, double dt)
{
	int N = bodies.size();

	std::vector<Body> new_temp_image = {};
	std::vector<Customvectors::Vector> a_ns = {};
	std::vector<Customvectors::Vector> a_n_dots = {};
	
	for (int i = 0; i < N; i++)
	{
		Body current_body = bodies[i];
		Customvectors::Vector a_n = calculateAcceleration(bodies, current_body, N, i);
		Customvectors::Vector a_n_dot = calculateJerk(bodies, current_body, N, i);
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();
		Customvectors::Vector r_nn_p = r_n + v_n * dt + a_n * 0.5 * dt * dt + a_n_dot * (1.0 / 6.0) * pow(dt, 3);
		Customvectors::Vector v_nn_p = v_n + a_n * dt + a_n_dot * 0.5 * dt * dt;
		current_body.setPosition(r_nn_p);
		current_body.setVelocity(v_nn_p);
		a_ns.push_back(a_n);
		a_n_dots.push_back(a_n_dot);
		new_temp_image.push_back(current_body);

	}
	std::vector<Body> newImage = {};
	for (int i = 0; i < N; i++)
	{
		Body current_body = new_temp_image[i];
		Body old_body = bodies[i];
		Customvectors::Vector a_nn_p = calculateAcceleration(new_temp_image, current_body, N, i);
		Customvectors::Vector a_nn_p_dot = calculateJerk(new_temp_image, current_body, N, i);
		Customvectors::Vector a_n = a_ns[i];
		Customvectors::Vector a_n_dot = a_n_dots[i];

		Customvectors::Vector r_nn_p = current_body.getPosition();
		Customvectors::Vector v_nn_p = current_body.getVelocity();
		Customvectors::Vector v_n = old_body.getVelocity();
		Customvectors::Vector r_n = old_body.getPosition();
		/*
		//Hermit-Interpolation
		Customvectors::Vector a_2 = (a_n - a_nn_p) * (-3.0 / (dt * dt)) - (a_n_dot * 2.0 + a_nn_p_dot) * (1 / dt);
		Customvectors::Vector a_3 = (a_n - a_nn_p) * (2.0 / (dt * dt * dt)) + (a_n_dot + a_nn_p_dot) * (1 / (dt * dt));
		*/
		//correction
		Customvectors::Vector v_nn_c = v_n + (a_nn_p + a_n) * 0.5 * dt + (a_nn_p_dot - a_n_dot) * (dt * dt) * (1.0/12.0);
		Customvectors::Vector r_nn_c = r_n + (v_nn_c + v_n) * 0.5 * dt + (a_nn_p - a_n) * (dt * dt) * (1.0 / 12.0);
		current_body.setVelocity(v_nn_c);
		current_body.setPosition(r_nn_c);
		newImage.push_back(current_body);
	}

	return newImage;
}

void IteratedHermit::startIntegration(std::vector<Body> initalImage, double eta, int iterations, std::string output_file)
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
