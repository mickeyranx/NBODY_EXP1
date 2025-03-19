#pragma once
#include "NbodyIntegrator.h"

class EulerChromer : public NbodyIntegrator {
public:
	

	std::vector<Body> integrate(std::vector<Body> bodies, double timeStep);
	void startIntegration(std::vector<Body> initalImage, double timeStep, int iterations, std::string outputPath);


};


