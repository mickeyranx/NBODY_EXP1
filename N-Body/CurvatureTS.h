#pragma once
#include <vector>
#include "TimeStepFunction.h"
#include "NbodyIntegrator.h"
class CurvatureTS : TimeStepFunction<double, std::vector<double>, std::vector<double>> {
	double timeStep(double t, std::vector<double> listOfAccelerations, std::vector<double>);

};

