/* --------------------------------------------------------------
 TSP Modeling in MILP and CP
 Author: Kyle E. C. Booth (kbooth@mie.utoronto.ca)
 ---------------------------------------------------------------- */

#include <vector>
#include <string>
#include <ilcp/cp.h>
#include <ilcp/cp.h>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

class Problem {

	int n;
	vector < vector < int > > d;

protected:
	IloEnv env;
	IloModel model;
	IloInt timeLimit;
	
public:
  Problem(IloEnv e, IloInt timeLimit):
    env(e),
    timeLimit(timeLimit)

  {}
  virtual ~Problem() {}
	void generateInstance(int seed, int size);
	void createModelMILP(double timelimit, int size, int vehicles);
	void createModelCP(double timelimit, int size, int vehicles);
	void createModelCPsingleResource(double timelimit, int size, int vehicles);
};

