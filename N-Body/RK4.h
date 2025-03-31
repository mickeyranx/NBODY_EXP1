#pragma once
#include <iostream>
#include <fstream>
#include "NbodyIntegrator.h"
#include <tuple>

class RK4 : public NbodyIntegrator {
public:

	RK4(TimeStep tsf) : NbodyIntegrator(tsf) {};
	RK4(TimeStep tsf, double max_step) : NbodyIntegrator(tsf, max_step) {};


	std::tuple<std::vector<Body>, double> integrate(std::vector<Body> bodies, double dt, int N);

	void startIntegration(std::vector<Body> inital_image, double eta, int iterations, std::string output_file) override;


};