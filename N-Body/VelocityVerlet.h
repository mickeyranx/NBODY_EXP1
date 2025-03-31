#pragma once
#include <iostream>
#include <fstream>
#include <tuple>

#include "NbodyIntegrator.h"


class VelocityVerlet : public NbodyIntegrator {
public:

	VelocityVerlet(TimeStep tsf) : NbodyIntegrator(tsf) {};
	VelocityVerlet(TimeStep tsf, double max_step) : NbodyIntegrator(tsf, max_step) {};

	std::tuple<std::vector<Body>, double> integrate(std::vector<Body> bodies, double dt, int N);

	void startIntegration(std::vector<Body> inital_image, double eta, int iterations, std::string output_file) override;

};


