#include "CurvatureTS.h"

double CurvatureTS::timeStep(double t, std::vector<double> listOfAccelerations, std::vector<double> listOfJerks)
{
	int size = listOfAccelerations.size();
	std::vector<double> timeSteps(size);
	for (int i = 0; i < size; i++)
	{
		timeSteps[i] = t * sqrt(listOfAccelerations[i]/listOfAccelerations[i]);
	}

	//TODO: find the min value and return 
    return 0.0;
}
