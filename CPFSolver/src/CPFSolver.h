#ifndef __CPFSOLVER__
#define __CPFSOLVER__

#include "Instance.h"
#include "Parser.h"
#include "Encoder.h"
#include "Search.h"
#include "simp/SimpSolver.h"

class CPFSolver {
private:
	Instance _instance;
	Solution _solution;
	Glucose::SimpSolver _solver;

	/* Strategies: */
	Encoder *_encoder;
	Search  *_search;

	int _verbose;
	int _max_makespan;

	int _n_sat_calls   = 0;
	int _n_unsat_calls = 0;

public:
	// Complete constructor:
	CPFSolver(Instance instance, std::string encoding, std::string search, int max_makespan, int verbose)
	: _instance(instance), _solution(instance), _solver() {
		_max_makespan = max_makespan;
		_verbose = verbose;
		_solver.setIncrementalMode();
		create_encoder(encoding);
		create_search(search);
	}

	// Constructor with default maximum makespan.
	CPFSolver(Instance instance, std::string encoding, std::string search, int verbose = 0)
	: _instance(instance), _solution(instance), _solver() {
		_verbose = verbose;
		_max_makespan = _instance.n_vertices() * _instance.n_agents();
		_solver.setIncrementalMode();
		create_encoder(encoding);
		create_search(search);
	}

	// Minimal constructor. Default encoding, search, and maximum makespan.
	CPFSolver(Instance instance, int verbose = 0)
	: _instance(instance), _solution(instance), _solver() {
		_max_makespan = _instance.n_vertices() * _instance.n_agents();
		_solver.setIncrementalMode();
		create_encoder("simplified");
		create_encoder("UNSAT-SAT");
	}

	~CPFSolver() { delete _encoder; delete _search; }

	// Main contructor functionality. Provides a solution for its instance.
	Solution solve();

	// Write SAT solver statistics to a file.
	void write_stats(std::ostream &os);

private:

	// Auxiliary function. Used on solve(). Tries to find a solution for
	//   the instance with a given makespan
	bool solve_for_makespan(int makespan);

	// Auxiliary function. Used on constructors.
	void create_encoder(std::string encoding);

	// Auxiliary function. Used on constructors.
	void create_search(std::string search);

};

#endif
