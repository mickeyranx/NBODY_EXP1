#pragma once
#include "TimeStepFunction.h"
class LinearTS : public TimeStepFunction<double>{
public:
	double timeStep(double t) { return t;};

};