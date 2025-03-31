#pragma once
#include <iostream>
#include <fstream>
#include <tuple>

#include "NbodyIntegrator.h"


class Hermit : public NbodyIntegrator {
public:

	Hermit(TimeStep tsf) : NbodyIntegrator(tsf) {};
	Hermit(TimeStep tsf, double min_step) : NbodyIntegrator(tsf, min_step) {};

	std::tuple<std::vector<Body>, double> integrate(std::vector<Body> bodies, double dt, int N);

	void startIntegration(std::vector<Body> inital_image, double eta, int iterations, std::string output_file) override;

	double calculateTimeStepHermit(
		std::vector<Customvectors::Vector> &as, 
		std::vector<Customvectors::Vector> &a_dots, 
		std::vector<Customvectors::Vector> &a_2s,
		std::vector<Customvectors::Vector> &a_3s, int N, double dt);


};