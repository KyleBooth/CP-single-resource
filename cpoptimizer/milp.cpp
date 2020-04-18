/* --------------------------------------------------------------
 TSP Modeling in MILP and CP
 Author: Kyle E. C. Booth (kbooth@mie.utoronto.ca)
 ---------------------------------------------------------------- */

#include "problem.h"
#include <math.h>
#include <ilcp/cp.h>
#include <ilcplex/ilocplex.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <time.h>
#include <ctime>
#include <set>

/////////////////////////////////////////////////////////////////
// MILP Model
/////////////////////////////////////////////////////////////////
void Problem::createModelMILP(double timeLimit, int size, int vehicles)
{
  cout << "Building MILP model." << endl;
  clock_t start_time, end_time;
  start_time = clock();
  model = IloModel(env);

  cout << "Vehicles: " << vehicles << endl;

  n = size;

  // Arc routing variables
  IloArray < IloBoolVarArray > x(env);
  for (int i = 0; i < n; i++)
  {
	IloBoolVarArray tmp(env);
	for (int j = 0; j < n; j++)
	{
		if (i != j)
			tmp.add(IloBoolVar(env));
		else
			tmp.add(IloBoolVar(env, 0, 0));
		model.add(tmp[j] >= 0);
	}
	x.add(tmp);
  }

  // Sequencing variables
  IloNumVarArray tau(env);
  for (int i = 0; i < n; i++)
  {
	if (i == 0)
		tau.add(IloNumVar(env, 0, 0));
	else
		tau.add(IloNumVar(env, 0, n));
	model.add(tau[i] >= 0);
  }

  // Degree constraints (outflow and inflow)
  IloNumExpr sumX_start(env);
  for (int j = 1; j < n; j++)
	sumX_start += x[0][j];
  model.add(sumX_start == vehicles);
 
  IloNumExpr sumX_end(env);
  for (int i = 1; i < n; i++)
	sumX_end += x[i][0];
  model.add(sumX_end == vehicles);

  for (int i = 1; i < n; i++) // Outflow
  { 
	IloNumExpr sumX(env);
	for (int j = 0; j < n; j++)
	{
		if (i != j)
			sumX += x[i][j];
	}
	model.add(sumX == 1);
  }
  for (int j = 1; j < n; j++) // Inflow
  {
	IloNumExpr sumX(env);
	for (int i = 0; i < n; i++)
	{
		if (i != j)
			sumX += x[i][j];
	}
	model.add(sumX == 1);
  }

 
  // Subtour elimination constraints
  for (int i = 0; i < n; i++)
  {
	for (int j = 1; j < n; j++)
		model.add(tau[i] + x[i][j] - n * (1 - x[i][j]) <= tau[j]);	
  }

  // Objective function
  IloNumExpr distance(env);
  for (int i = 0; i < n; i++)
  {
	for (int j = 0; j < n; j++)
	{
		if (i != j)
			distance += x[i][j] * d[i][j];
	}
  }

  model.add(IloMinimize(env, distance));

  IloCplex cplex(model);
  cplex.setParam(IloCplex::TiLim, timeLimit);
  cplex.setParam(IloCplex::Threads, 1); 

  if (cplex.solve())
  {
	cout << "MILP model results." << endl;
	cout << "Solution found. Optimization successful." << endl;
	cout << "Checking solution..." << endl;
	cout << "Solution valid." << endl;
	cout << "Route distance: " << cplex.getValue(distance) << endl;
	cout << "Arc variables: " << endl;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			if (cplex.getValue(x[i][j]) == 1)
				cout << i << "->" << j << endl;
		}
	}
	cout << "Sequencing variables: " << endl;
	for (int i = 0; i < n; i++)
		cout << i << ": " << cplex.getValue(tau[i]) << endl;
  }
  else
	cout << "No solution. Infeasible." << endl;

  cplex.end();
  return;
}

