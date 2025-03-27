// N-Body.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//

#include <iostream>
#include "vector.h"
#include <fstream>
#include "Body.h"
#include <vector>
#include <string>
#include "NbodyIntegrator.h"
#include "Euler.h"
#include "TimeStep.h"
#include <tuple>


using namespace std;
using namespace Customvectors;





//this function reads a file containing the initial values of the bodies in the x y z vx vy vz m format
//             and parses the COM-transformed data into a list of bodies
static std::vector<Body> setup(std::string file_name) {
	//--------------------------------------------------
	//          parse starting data from file 
	//               into a list of bodies
	//--------------------------------------------------

	//open file with given file name
	ifstream file(file_name);
	
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
	

	
	//--------------------------------------------------
	//          calculate COM reference
	//--------------------------------------------------
	Vector r_s = Vector(0, 0, 0);
	Vector v_s = Vector(0, 0, 0);
	double M = 0;
	for (int i = 0; i < bodies.size(); i++)
	{
		Body b = bodies[i];
		double m = b.getMass();
		r_s = r_s + b.getPosition() * m;
		v_s = v_s + b.getVelocity() * m;
		M += m; //should add up to 1 in the end
	}
	cout << "total mass is " << M << endl;

	vector<Body> bodies_COM = {};
	for (int i = 0; i < bodies.size(); i++)
	{
		Body b = bodies[i];
		Vector vel_COM =  (b.getVelocity() - v_s) * (1 / M);
		Vector pos_COM = (b.getPosition() - r_s) * (1 / M);
		bodies_COM.push_back(Body(b.getMass(), pos_COM, vel_COM));
	}
	

	return bodies_COM;

}


//main function 
int main() {
	clock_t start = clock();
	/*
		TODO: 
		-implement reading N, nu and t_max
		-implement dynamic time step for Hermit and RK4
		-implement minimum time step for dynamic TS
		-kickdrift for Velocity-Verlet
	*/
	
	//------------------------------------------------
	//       setup and load data from files
	//------------------------------------------------
	string output_path = "output.txt";
	
	//inputfile-name for body data 
	string input_name = "2body.txt";
	//inputfile-name for integration data
	string input_name2 = "";
	
	//ifstream File(input_name2);

	
	//timestep dt
	double eta = 0.3;

	//number of integrations
	int n = 20;
	std::vector<Body> starting_image = setup("input_files/" + input_name);
	//------------------------------------------------
	//       select integrator and time step
	//				  calculation
	//------------------------------------------------
	/*
	   available Time steps:
			-LINEAR
			-QUADRATIC
			-CURVATURE
			(-HERMIT)
			(-RKFOUR)
	
	*/
	//switch between integrators via commenting
	Euler integrator = Euler(TimeStep::LINEAR);
	//NbodyIntegrator integrator = new EulerChromer();
	//NbodyIntegrator integrator = new ();

	//Start the integration
	integrator.startIntegration(starting_image, eta, n, output_path);


	clock_t end = clock();
	double elapsed = double(end - start) / CLOCKS_PER_SEC;
	printf("execution time: %.3f sec", elapsed);
}



