#pragma once
#include "vector.h"
#include "TimeStep.h"
#include "Body.h"
#include <vector>
#include <any>
#include <string>

class NbodyIntegrator{


private:
	
	//type of parameter
	TimeStep tsf;

public:
	NbodyIntegrator(TimeStep timeStepFunction);
	//calculates the next picture

	
	//starts the Integration with 
	virtual void startIntegration(std::vector<Body> bodies, double timeStep, int iterations, std::string outputPath);
	

	//virtual void simulate(int Iterations, int N);
	//List of bodies, N = number of bodies, j ist the current body
	static Customvectors::Vector calculateAcceleration(std::vector<Body> bodies, Body reference_body, int N, int j);
	//List of bodies, N = number of bodies, j ist the current body
	static Customvectors::Vector calculateJerk(std::vector<Body> bodies, Body reference_body, int N, int j);
			
	static double timeStepLinear(double t) { return t; };

	static double timeStepQuadratic(double t) { return pow(t, 2); };

	//static double timeStepCurvature(double t);

	//start integration 


};

