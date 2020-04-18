/* --------------------------------------------------------------
 TSP Modeling in MILP and CP
 Author: Kyle E. C. Booth (kbooth@mie.utoronto.ca)
 ---------------------------------------------------------------- */

#include <iostream>
#include <string>
#include <vector>
#include <ilcplex/ilocplex.h>
#include <ilcp/cp.h>
#include "problem.h"

using namespace std;

int main(int argc, const char* argv[])
{
	IloEnv env;
	try
	{
		double timelimit = atof(argv[1]);	// Time limit
		std::string modelType = argv[2];	// Model type
		int seed = atoi(argv[3]); 		// Random seed 
		int size = atoi(argv[4]);		// Instance size
		int vehicles = atoi(argv[5]);		// Number of vehicles

		Problem *prob; 
	 	prob = new (env) Problem(env, timelimit);

		prob->generateInstance(seed, size);	// Generate random instance

		if (modelType == "MILP")
			prob->createModelMILP(timelimit, size, vehicles); 
		else if (modelType == "CP")
			prob->createModelCP(timelimit, size, vehicles);
		else if (modelType == "CP-SR")
			prob->createModelCPsingleResource(timelimit, size, vehicles);

	}
	catch(IloException& e)
	{
		env.out() << " ERROR: " << e << std::endl;
	}
	env.end();
	return 0;
}
