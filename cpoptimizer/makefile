CCC = g++ -O0

CONCERTDIR = ../../software/solvers/CPLEX_Studio128/concert
CPLEXDIR = ../../software/solvers/CPLEX_Studio128/cplex
CPDIR = ../../software/solvers/CPLEX_Studio128/cpoptimizer

CFLAGS = -DIL_STD -O -DNDEBUG -I$(CPLEXDIR)/include -I$(CPDIR)/include -I$(CONCERTDIR)/include -fPIC -fstrict-aliasing -pedantic -Wall -fexceptions -Wno-long-long -m64 -DILOUSEMT -D_REENTRANT -DILM_REENTRANT -std=c++0x

LDFLAGS = -L$(CPDIR)/lib/x86-64_linux/static_pic -lcp -L$(CPLEXDIR)/lib/x86-64_linux/static_pic -lcplex -lilocplex -L$(CONCERTDIR)/lib/x86-64_linux/static_pic -lconcert -lpthread -lm -ldl

CPCLASSPATH = $(CPDIR)/lib/ILOG.CP.jar
LIBRARYPATH = $(CPDIR)/bin/x86-64_linux:$(CPLEXDIR)/bin/x86-64_linux

clean:
	rm -f tsp *.class

FORCE:

main:
	$(CCC) -o tsp $(CFLAGS) main.cpp problem.cpp milp.cpp cp.cpp $(LDFLAGS)
main.run: tsp FORCE
	./tsp
