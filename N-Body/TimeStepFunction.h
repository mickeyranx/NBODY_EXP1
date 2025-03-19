#pragma once
//
template <typename... Args>
//interface which has a function 
class TimeStepFunction
{
public:
	//this function should calculate the next time step, depending on the different ways to use a time step it can have different arguments and number of arguments
	virtual double calculateTimeStep(Args... args) = 0;
};

