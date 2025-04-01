#include "Euler.h"


std::tuple<std::vector<Body>, double> Euler::integrate(std::vector<Body> &image,  double eta , int N)
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
	//        calculate time step
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
		double new_time_step = timeStepCurvature(image, a_ns ,N, eta);
		double max_step = getMaxTimeStep();
		dt = (new_time_step > max_step || new_time_step < 0) ? max_step : new_time_step;
		break;
	}

	//------------------------------------------------
	//          calculatate next image
	//------------------------------------------------
	std::vector<Body> new_image; 
	for (int i = 0; i < N; i++)
	{
		
		Body current_body = image[i];
		Customvectors::Vector v_n = current_body.getVelocity();
		Customvectors::Vector r_n = current_body.getPosition();
		Customvectors::Vector v_new = v_n + a_ns[i] * dt;
		Customvectors::Vector r_new = r_n + v_n * dt;
		//add Body to newImage list with new position and velocity vectors
		new_image.push_back(Body(current_body.getMass(), r_new, v_new));
		
	}
	
	return std::make_tuple(new_image, dt); //return new constellation and time step

}

void Euler::startIntegration(std::vector<Body> inital_image, double eta, double max_integration_time, std::string output_file)
{
	int N = inital_image.size();
	double time_passed = 0;
	double time_step;
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
	File << std::fixed << std::setprecision(4);
	//-------------------------------------
	//          inital image
	// ------------------------------------
	File << time_passed << "\t";
	for (Body b : inital_image) {
		File << b.getPosition().getX() << "\t" << b.getPosition().getY() << "\t" << b.getPosition().getZ() << "\t";

	}
	double E = calculateEnergy(inital_image, N);
	File << E << "\t";
	if (N == 2) {
		Vector j = calculateAngularMomentum(inital_image);
		Vector e = calulateRungeLenz(inital_image, j);
		double a = calculateMajorSemiAxis(j, e);
		File << j.getLength() << "\t" << e.getLength() << "\t" << a;
	}
	File << "\n";
	//-------------------------------------
	//            integration
	//-------------------------------------
	std::vector<Body> previous_image = inital_image;
	int i = 0;
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

		i++;
		time_passed += time_step;
		previous_image = new_image; //prepare for next iteration
	}
	
	File.close();

}
