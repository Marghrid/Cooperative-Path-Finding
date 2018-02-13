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
	Glucose::SimpSolver _solver;

	/* Strategies: */
	Encoder *_encoder;
	Search  *_search;

	int _verbose;
	int _max_makespan;

public:
	CPFSolver(Instance instance, std::string encoding, std::string search, int max_makespan, int verbose)
	: _instance(instance), _solver() {
		_max_makespan = max_makespan;
		_verbose = verbose;
		create_encoder(encoding);
		create_search(search);
	}

	CPFSolver(Instance instance, std::string encoding, std::string search, int verbose = 0)
	: _instance(instance), _solver() {
		_verbose = verbose;
		_max_makespan = _instance.n_vertices();
		create_encoder(encoding);
		create_search(search);
	}

	CPFSolver(Instance instance, int verbose = 0)
	: _instance(instance), _solver() {
		_max_makespan = _instance.n_vertices();
		create_encoder("simplified");
		create_encoder("UNSAT-SAT");
	}

	CPFSolver(std::string filename, std::string encoding, std::string search, int verbose = 0)
	: _solver() {
		Parser p(filename);
		_instance = p.parse();
		_verbose = verbose;
		_max_makespan = _instance.n_vertices();
		create_encoder(encoding);
		create_search(search);
	}

	~CPFSolver() { delete _encoder; delete _search; }

	Solution solve();

private:
	bool solve_for_makespan(int makespan);

	void create_encoder(std::string encoding);

	void create_search(std::string search);

};

#endif
