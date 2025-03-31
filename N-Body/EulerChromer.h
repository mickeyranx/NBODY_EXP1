#pragma once
#include "NbodyIntegrator.h"
#include <iostream>
#include <tuple>
#include <fstream>

class EulerChromer : public NbodyIntegrator {
public:
	
	EulerChromer(TimeStep tsf) : NbodyIntegrator(tsf) {};
	EulerChromer(TimeStep tsf, double max_step) : NbodyIntegrator(tsf, max_step) {};
	std::tuple<std::vector<Body>, double> integrate(std::vector<Body> bodies, double dt, int N);
	void startIntegration(std::vector<Body> inital_image, double eta, int iterations, std::string output_file);


};


