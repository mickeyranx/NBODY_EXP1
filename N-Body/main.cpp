// N-Body.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//

#include <iostream>
#include "vector.h"
#include <fstream>
#include "Body.h"
#include <vector>
#include <string>
#include "NbodyIntegrator.h"
#include "TimeStepFunction.h"
#include "Euler.h"
#include "TimeStep.h"


using namespace std;
using namespace Customvectors;






//this function reads a file containing the initial values of the bodies in the x y z vx vy vz m format, and parses the data into a list of bodies
static std::vector<Body> setup(std::string fileName) {
	//open file with given file name
	ifstream file(fileName);
	
	//initialize list  
	std::vector<Body> bodies;
	
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	double m;
	//read doubles from file until there are no more values left
	for (int i = 0; (file >> x >> y >> z >> vx >> vy >> vz >> m); i++)
	{
		Vector pos = Vector(x, y, z);
		Vector vel = Vector(vx, vy, vz);
		Body body = Body(m, pos, vel);
		//add created body to the list
		bodies.push_back(body);

	}
	return bodies;

}


//main function 
int main() {
	clock_t start = clock();
	//project path

	//pointer to path string (path to write data) change to local path 
	//TODO: make this independet on environment
	string outputPath = "output.txt";

	//inputfile name
	string inputName = "2body.txt";

	//timestep dt
	double dt = 1;

	//number of integrations
	int n = 20;

	//
	std::vector<Body> startingImage = setup(inputName);

	//switch between integrators via commenting
	Euler integrator = Euler(TimeStep::LINEAR);
	//NbodyIntegrator integrator = new EulerChromer();
	//NbodyIntegrator integrator = new ();

	//Start the integration
	integrator.startIntegration(startingImage, dt, n, outputPath);

	//TODO: 

	clock_t end = clock();
	double elapsed = double(end - start) / CLOCKS_PER_SEC;
	printf("execution time: %.3f sec", elapsed);
}



