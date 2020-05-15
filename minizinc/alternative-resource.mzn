% m-TSP variant: Alternative resource CP model (MiniZinc implementation)
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
var 0..H: Z; % objective variable
array[J] of var 0..H: S; % start time variables
array[M,J] of var opt 0..H: S_opt; % optional start time variables

% Constraints
constraint forall(j in J)(alternative(S[j], p[j], [S_opt[i,j] | i in M], [p[j] | i in M]));
constraint forall(i in M)(disjunctive([S_opt[i,j] | j in J], p));
constraint cumulative(S, p, [1 | j in J], m);

constraint forall(i in M)(forall(j, k in J where j < k)(if not absent(S_opt[i,j]) /\ not absent(S_opt[i,k]) then S_opt[i,j] >= (S_opt[i,k] + (p[k] + tt[k,j])) \/ S_opt[i,k] >= (S_opt[i,j] + (p[j] + tt[j,k])) else true endif));

% Objective function
constraint (Z == max(j in J)(S[j] + p[j]));
solve minimize Z;

% Solution output
output ["Objective: ", show(Z), ", Start times:", show(S), ", Opt start times: ", show(S_opt)]
