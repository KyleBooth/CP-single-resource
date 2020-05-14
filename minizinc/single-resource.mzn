% m-TSP: Single resource CP model (MiniZinc implementation)
% Author: Kyle E. C. Booth (kbooth@mie.utoronto.ca)

include "globals.mzn";

% Parameters
int: m;
int: n; 
int: H;
set of int: M = 1..m;
set of int: J = 1..n;
array[J] of int: p;
array[J,J] of int: tt;

% Variables

% Constraints

% Objective function

% Solution output
