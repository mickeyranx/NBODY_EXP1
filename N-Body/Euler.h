#pragma once
#include <iostream>
#include <fstream>
#include <tuple>

#include "NbodyIntegrator.h"


class Euler : public NbodyIntegrator {
public:

	Euler(TimeStep tsf) : NbodyIntegrator(tsf) { };
	Euler(TimeStep tsf, double max_step) : NbodyIntegrator(tsf, max_step) {};

	std::tuple<std::vector<Body>, double> integrate(std::vector<Body> bodies, double time_step, int N);

	void startIntegration(std::vector<Body> inital_image, double time_step, int Iterations, std::string output_path) override;

};

