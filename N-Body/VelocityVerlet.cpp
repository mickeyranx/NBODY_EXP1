#include "VelocityVerlet.h"




std::tuple<std::vector<Body>, double> VelocityVerlet::integrate(std::vector<Body> bodies, double dt, int N)
{
	//------------------------------------------------
	//          determine next image
	//------------------------------------------------
	std::vector<Body> new_temp_image = {};
	std::vector<Customvectors::Vector> a_ns = {};

	for (int i = 0; i < N; i++)
	{
		Body current_body = bodies[i];
		Customvectors::Vector a_n = calculateAcceleration(bodies, current_body, N, i);
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();

		Customvectors::Vector r_nn = r_n + v_n * dt + a_n * 0.5 * dt * dt;
		current_body.setPosition(r_nn);
		a_ns.push_back(a_n);
		new_temp_image.push_back(current_body);
		
	}
	std::vector<Body> new_image = {};
	for (int i = 0; i < N; i++)
	{
		Body current_body = new_temp_image[i];
		Customvectors::Vector a_nn = calculateAcceleration(new_temp_image, current_body, N, i);
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector v_nn; 
		/*
		if (getTimeStep() == TimeStep::DYNAMIC) { //kick-drift 
			v_nn = v_n + 

		}
		else {
			v_nn =  v_n + (a_ns[i] + a_nn) * dt;
		}
		*/
		current_body.setVelocity(v_nn);
		new_image.push_back(current_body);
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

void VelocityVerlet::startIntegration(std::vector<Body> initial_image, double eta, int iterations, std::string output_file)
{
	int N = initial_image.size();
	double time_step = eta;
	//-------------------------------------
	//        file setup
	//-------------------------------------
	std::ofstream File(output_file);
	File << "t" << "\t";
	for (int i = 0; i < initial_image.size(); i++) //output file setup
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
	std::vector<Body> previous_image = initial_image;
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
