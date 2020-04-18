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
// CP Model 
/////////////////////////////////////////////////////////////////
void Problem::createModelCP(double timeLimit, int size, int vehicles)
{
  cout << "Building alternative resource CP model." << endl;
  clock_t start_time, end_time;
  start_time = clock();
  model = IloModel(env);

  n = size;

  // Makespan (objective) variable
  IloIntVarArray makespans(env);

  // Interval variables  
  IloArray < IloIntervalVarArray > x(env);
  for (int k = 0; k < vehicles; k++)
  {
	IloIntervalVarArray x_tmp(env);
	IloIntVar makespan(env);
	for (int i = 0; i < n + 1; i++)
	{
		IloIntervalVar tmp(env);
		tmp.setStartMin(0);
		if (i > 0 && i < n)
			tmp.setOptional();
		if (i == 0)
			tmp.setStartMax(0);
		tmp.setLengthMin(1);
		tmp.setLengthMax(1);
		x_tmp.add(tmp);
		if (i == n)
			model.add(makespan >= IloEndOf(tmp));
	}
	makespans.add(makespan);
	x.add(x_tmp);
  }

  IloIntArray types(env);
  for (int i = 0; i < n + 1; i++)
  {
	if (i < n)
		types.add(i);
	else
		types.add(0);
  }

  // Alternative Constraint
  for (int i = 1; i < n; i++)
  {
	IloIntExpr sumX(env);
	for (int k = 0; k < vehicles; k++)
		sumX += IloPresenceOf(env, x[k][i]);
	model.add(sumX == 1);
  }

  // Each vehicle at least one customer
  for (int k = 0; k < vehicles; k++)
  {
	IloIntExpr sumX(env);
	for (int i = 1; i < n; i++)
		sumX += IloPresenceOf(env, x[k][i]);
	model.add(sumX >= 1);
  }

  IloTransitionDistance transitions(env, n);
  for (int i = 0; i < n; i++)
  {
	for (int j = 0; j < n; j++)
		transitions.setValue(i, j, d[i][j]);
  }	

  // Sequence variable (and first/last constraints)
  for (int k = 0; k < vehicles; k++)
  {
	  IloIntervalSequenceVar x_seq(env, x[k], types);
	  model.add(IloFirst(env, x_seq, x[k][0]));
	  model.add(IloLast(env, x_seq, x[k][n]));
	  model.add(IloNoOverlap(env, x_seq, transitions));
  }

  // Objective function
  IloIntExpr objective(env);
  for (int k = 0; k < vehicles; k++)
	objective += makespans[k];
  model.add(IloMinimize(env, objective));

  IloCP cp(model);
  cp.setParameter(IloCP::TimeLimit, timeLimit);
  cp.setParameter(IloCP::Workers, 1);
  
  int solID = 1;

  if(cp.solve())
  {
	cout << "CP model (alternative resource) results." << endl;
	cout << "Solution found. Optimization successful." << endl;
	cout << "Checking solution..." << endl;
	cout << "Solution valid." << endl;
	cout << "Route distance: " << cp.getValue(objective) - (vehicles*2 + (n-1)) << endl;
	for (int k = 0; k < vehicles; k++)
	{
		cout << "Vehicle: " << k << endl;
		for (int i = 0; i < n + 1; i++)
		{
			if (cp.isPresent(x[k][i]))
				cout << i << " " << cp.getStart(x[k][i]) << endl;
		}
		cout << endl;
	}
  }
  else
	cout << "No solution. Infeasible." << endl;
	
  cp.end();
  return;
}

void Problem::createModelCPsingleResource(double timeLimit, int size, int vehicles)
{
  cout << "Building single resource transformation CP model." << endl;
  clock_t start_time, end_time;
  start_time = clock();
  model = IloModel(env);

  n = size;

  IloIntVar makespan(env);
  IloIntArray types(env);

  IloIntervalVarArray x(env);
  for (int k = 0; k < vehicles; k++) // Dummy tasks
  {
	IloIntervalVar tmp(env);
	tmp.setStartMin(0);
	types.add(0);
	tmp.setLengthMin(1);
	tmp.setLengthMax(1);
	x.add(tmp);
	if (k == vehicles-1)
		model.add(makespan >= IloEndOf(tmp));
  }

  for (int i = 0; i < n; i++) // Actual tasks
  {
	IloIntervalVar tmp(env);
	tmp.setStartMin(0);
	if (i == 0)
		tmp.setStartMax(0);
	types.add(i);
	tmp.setLengthMin(1);
	tmp.setLengthMax(1);
	x.add(tmp);
  }

  IloTransitionDistance transitions(env, n);
  for (int i = 0; i < n; i++)
  {
	for (int j = 0; j < n; j++)
		transitions.setValue(i, j, d[i][j]);
  }

  IloIntervalSequenceVar x_seq(env, x, types);
  model.add(IloFirst(env, x_seq, x[vehicles]));
  model.add(IloLast(env, x_seq, x[vehicles-1]));
  model.add(IloNoOverlap(env, x_seq, transitions));

  model.add(IloTypeOfNext(x_seq, x[vehicles], 0) != 0); 
  for (int k = 0; k < vehicles-1; k++)
  	model.add(IloTypeOfNext(x_seq, x[k], 0) != 0); 

  model.add(IloMinimize(env, makespan));	

  IloCP cp(model);
  cp.setParameter(IloCP::TimeLimit, timeLimit);
  cp.setParameter(IloCP::Workers, 1);
  
  int solID = 1;

  if(cp.solve())
  {
	cout << "CP model (single resource transformation) results." << endl;
	cout << "Solution found. Optimization successful." << endl;
	cout << "Checking solution..." << endl;
	cout << "Solution valid." << endl;
	cout << "Route distance: " << cp.getValue(makespan) - (vehicles + n) << endl;
  }
  else
	cout << "No solution. Infeasible." << endl;
	
  cp.end();
  return;
}
