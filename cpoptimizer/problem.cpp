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

using namespace std;

void Problem::generateInstance(int seed, int size)
{
  srand (seed);
  vector < vector < int > > coords;
  int max_coord = 250;
  for (int i = 0; i < size; i++)
  {
	vector < int > tmp;
	int x = rand() % max_coord + 1;
	int y = rand() % max_coord + 1;
	tmp.push_back(x);
	tmp.push_back(y);
	coords.push_back(tmp);
  } 
  cout << endl << "Distance matrix: " << endl;
  for (int i = 0; i < size; i++)
  {
	vector < int > tmp;
	for (int j = 0; j < size; j++)
	{
		int dist = 0;
		if (i != j)
		{
			int dx = coords[i][0]-coords[j][0];
			int dy = coords[i][1]-coords[j][1];
			dist = std::sqrt(dx * dx + dy * dy);
		}
		if (size < 30)
			cout << dist << " ";
		tmp.push_back(dist);
	}
	if (size < 30)
		cout << endl;
	d.push_back(tmp);
  }
  if (size < 30)
  	cout << endl;

  return;
}





