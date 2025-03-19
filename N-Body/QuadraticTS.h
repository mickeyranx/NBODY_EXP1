#pragma once
#include "TimeStepFunction.h"
#include <cmath>
class QuadraticTS : TimeStepFunction<double> {
public:
	double timeStep(double t) { return pow(t,2); };

};