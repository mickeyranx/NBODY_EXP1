#include "Heun.h"


std::tuple<std::vector<Body>, double> Heun::integrate(std::vector<Body> &image,  double eta, int N)
{
	std::vector<Customvectors::Vector> a_ns; //accelerations
	//------------------------------------------------
	//         calculate accelerations
	//------------------------------------------------
	for (int i = 0; i < N; i++)
	{
		a_ns.push_back(calculateAcceleration(image, image[i], N, i));
	}

	//------------------------------------------------
	//          determine next time step
	//------------------------------------------------
	double dt = 0;
	switch (getTimeStep())
	{
	case TimeStep::LINEAR:
		dt = eta;
		break;
	case TimeStep::QUADRATIC:
		dt = eta * eta;
		break;
	case TimeStep::DYNAMIC:
		double new_time_step = timeStepCurvature(image, a_ns, N, eta);
		double max_step = getMaxTimeStep();
		dt = (new_time_step > max_step || new_time_step < 0) ? max_step : new_time_step;
		break;

	}

	//------------------------------------------------
	//                 integration
	//------------------------------------------------
	std::vector<Body> new_image = image;
	std::vector<Body> new_temp_image = {};
	
	for (int i = 0; i < N; i++)
	{
		Body current_body = image[i];
		
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();

		Customvectors::Vector v_1 = a_ns[i] * dt;
		Customvectors::Vector r_1 = v_n * dt;
		new_image[i].alterPosition(r_1 * 0.5); //add first order term
		new_image[i].alterVelocity(v_1 * 0.5); //add first order term
		Customvectors::Vector r_2 = (v_n + v_1) * dt;
		new_image[i].alterPosition(r_2 * 0.5); //add second order term
		current_body.setPosition(r_n + r_1);
		new_temp_image.push_back(current_body); //for calculation of v_2

	}

	for (int i = 0; i < N; i++)
	{
		Customvectors::Vector v_2 = calculateAcceleration(new_temp_image, new_temp_image[i], N, i) * dt;
		new_image[i].alterVelocity(v_2 * 0.5); //add second order term
	}

	return std::make_tuple(new_image, dt);
}

void Heun::startIntegration(std::vector<Body> initial_image, double eta, double max_integration_time, std::string output_file)
{
	int N = initial_image.size();
	double time_passed = 0;
	double time_step;

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
	//          inital image
	// ------------------------------------
	File << time_passed << "\t";
	for (Body b : initial_image) {
		File << b.getPosition().getX() << "\t" << b.getPosition().getY() << "\t" << b.getPosition().getZ() << "\t";

	}
	double E = calculateEnergy(initial_image, N);
	File << E << "\t";
	if (N == 2) {
		Vector j = calculateAngularMomentum(initial_image);
		Vector e = calulateRungeLenz(initial_image, j);
		double a = calculateMajorSemiAxis(j, e);
		File << j.getLength() << "\t" << e.getLength() << "\t" << a;
	}
	File << "\n";
	//-------------------------------------
	//            integration
	//-------------------------------------
	std::vector<Body> previous_image = initial_image;
	while (time_passed < max_integration_time) {
		std::vector<Body> new_image;

		std::tie(new_image, time_step) = integrate(previous_image, eta, N);

		File << time_passed << "\t";
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

		time_passed += time_step;
		previous_image = new_image; //prepare for next iteration


	}
	
	File.close();

}
