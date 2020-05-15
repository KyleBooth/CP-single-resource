% m-TSP variant: Single resource CP model (MiniZinc implementation)
% Author: Kyle E. C. Booth (kbooth@mie.utoronto.ca)

include "globals.mzn";

% Parameters
int: m;
int: n; 
int: H;
int: H_aug = m*H-1;
set of int: M = 1..m;
set of int: J = 1..n;
array[J] of int: p;
array[J,J] of int: tt;

% Variables
array[J] of var 0..H_aug: z; % intermediate objective variables
array[J] of var 0..H_aug: S; % start time variables

% Constraints
constraint forall(j in J)('mod'(S[j], H) <= H - p[j] - 1);
constraint disjunctive(S, p);
constraint forall(j, k in J where j < k)(S[j] >= S[k] + (p[k] + tt[k,j]) \/ S[k] >= S[j] + (p[j] + tt[j,k]));

% Objective function
constraint forall(j in J)(z[j] == 'mod'(S[j] + p[j],H));
solve minimize max(z);

% Solution output
output ["Objective: ", show(max(z)), ", Start times: ", show(S)] 
