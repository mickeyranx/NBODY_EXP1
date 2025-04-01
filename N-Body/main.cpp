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
/*
		open questions:
				-kick drift for Velocity Verlet
				-dynamic time step for hermit
		TODO:

	*/

//main function 
int main() {
	clock_t start = clock();

	//------------------------------------------------
	//       select integrator and time step
	//				  calculation
	//------------------------------------------------
	/*
	   available Time steps:
			-LINEAR
			-QUADRATIC
			-CURVATURE
			-HERMIT (exclusive for hermit and iterated hermit)

		available Integrators:
			-Euler
			-EulerChromer
			-VelocityVerlet
			-Hermit
			-IteratedHermit
			-Heun
			-RK4
		examples:
		     RK4 integrator = RK4(TimeStep::LINEAR);
			Euler integrator = Euler(TimeStep::DYNAMIC, 0.04); (overloaded constructor with TimeStep and max time_step)

	*/


	Euler integrator = Euler(TimeStep::LINEAR);

	//------------------------------------------------
	//       setup and load data from files
	//------------------------------------------------
	string output_path = "output.txt"; //filename of output-file
	//inputfile-name for body data 
	string input_name = "2body.txt"; //filename of constellation input
	//string input_name2 = "pla3.txt";
	string input_name2 = "pl.1k.txt"; //filename of integration-info input
	//string input_name2 = "pl.100.txt";
	//string input_name2 = "in2.txt";
	
	ifstream File("input_files/" + input_name2);
	int N;
	double t_max;
	double eta;
	File >> N >> t_max >> eta;	
	File.close();
	std::vector<Body> starting_image = setup("input_files/" + input_name);

	/*-------------------------------------------
		starting_image = starting constellation
		eta = time step parameter
		t_max = maximum time of integration
	
	--------------------------------------------*/
	integrator.startIntegration(starting_image, eta, t_max, output_path); //Start the integration and writing to file 
	
	
	clock_t end = clock();
	double elapsed = double(end - start) / CLOCKS_PER_SEC;
	printf("execution time: %.3f sec", elapsed);
}



