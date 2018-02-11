#ifndef __CPFSOLVER__
#define __CPFSOLVER__

#include "Instance.h"
#include "Parser.h"
#include "SimplifiedEncoder.h"
#include "simp/SimpSolver.h"

class CPFSolver {
private:
	Instance _instance;
	Glucose::SimpSolver _solver;
	SimplifiedEncoder _encoder;

	int _verbose;
	int _max_makespan;

public:
	CPFSolver(Instance instance, int verbose = 0)
	: _instance(instance), _solver(), _encoder(_instance, &_solver, verbose) {
		_verbose = verbose;
		_max_makespan = 64;
	}

	CPFSolver(Instance instance, int max_makespan, int verbose)
	: _instance(instance), _solver(), _encoder(_instance, &_solver, verbose) {
		_max_makespan = max_makespan;
		_verbose = verbose;
	}

	CPFSolver(std::string filename, int verbose = 0)
	: _solver(), _encoder(_instance, &_solver, verbose) {
		Parser p(filename);
		_instance = p.parse();
		_verbose = verbose;
	}

	bool solve();

	 int get_initial_makespan() { return 0; }

private:
	bool solve_for_makespan(int makespan);

};

#endif
