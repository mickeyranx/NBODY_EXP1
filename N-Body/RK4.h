#pragma once
#pragma once
#include <iostream>
#include <fstream>

#include "NbodyIntegrator.h"


class RK4 : public NbodyIntegrator {
public:

	RK4(TimeStep tsf) : NbodyIntegrator(tsf) {};

	std::vector<Body> integrate(std::vector<Body> bodies, double timeStep);

	void startIntegration(std::vector<Body> initalImage, double timeStep, int Iterations, std::string outputPath) override;

};