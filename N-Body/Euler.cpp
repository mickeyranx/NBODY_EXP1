#include "Euler.h"


std::tuple<std::vector<Body>, double> Euler::integrate(std::vector<Body> bodies, double dt, int N)
{
	//------------------------------------------------
	//          calculatate next image
	//------------------------------------------------
	std::vector<Body> new_image; 
	for (int i = 0; i < N; i++)
	{
		Customvectors::Vector a = Euler::calculateAcceleration(bodies, bodies[i], N, i);
		Body current_body = bodies[i];
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();
		Customvectors::Vector v_new = v_n + a * dt;
		Customvectors::Vector r_new = r_n + v_n * dt;
		//add Body to newImage list with new position and velocity vectors
		new_image.push_back(Body(current_body.getMass(), r_new, v_new));
		
	}
	//------------------------------------------------
	//          determine next time step
	//------------------------------------------------
	switch (getTimeStep())
	{
	case TimeStep::LINEAR:
		return std::make_tuple(new_image, dt);
		break;
	case TimeStep::QUADRATIC:
		return std::make_tuple(new_image, dt * dt);
		break;
	case TimeStep::DYNAMIC:
		double new_time_step = timeStepCurvature(new_image, N, dt);
		double max_step = getMaxTimeStep();
		dt = (new_time_step > max_step || new_time_step < 0) ? max_step : new_time_step;
		return std::make_tuple(new_image, dt);
		break;
	
	}

	return std::make_tuple(new_image, 0);

}

void Euler::startIntegration(std::vector<Body> inital_image, double eta, int iterations,std::string output_file)
{
	int N = inital_image.size();
	double time_step = eta;
	//-------------------------------------
	//        file setup
	//-------------------------------------
	std::ofstream File(output_file);
	File << "t" << "\t";
	for (int i = 0; i < inital_image.size(); i++) //output file setup
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
	std::vector<Body> previous_image = inital_image;
	for (int i = 0; i < iterations; i++)
	{
		std::vector<Body> new_image;


		std::tie(new_image, time_step) = integrate(previous_image, time_step, N);
		
	
		File << time_step * i << "\t";
		for (Body b : new_image) {
			File << b.getPosition().getX() << "\t" << b.getPosition().getY() << "\t" << b.getPosition().getZ() << "\t";
			
		}

		//-------------------------------------
		//        conserved quantities
		//-------------------------------------
		double E = calculateEnergy(new_image, N);
		File << E << "\t";
		if (N == 2) {
			Vector j = calculateAngularMomentum(new_image);
			Vector e = calulateRungeLenz(new_image, j);
			double a = calculateMajorSemiAxis(j, e);
			File << j.getLength() << "\t" << e.getLength() << "\t" << a;
		}
		File << "\n";
		

		
		previous_image = new_image; //prepare for next iteration

		
	}
	File.close();

}
