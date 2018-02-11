#ifndef __CPFSOLVER__
#define __CPFSOLVER__

#include "Instance.h"
#include "Parser.h"
#include "Encoder.h"
#include "SimplifiedEncoder.h"
#include "Search.h"
#include "UNSAT_SATSearch.h"
#include "SAT_UNSATSearch.h"
#include "BinarySearch.h"
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
	CPFSolver(Instance instance, int verbose = 0)
	: _instance(instance), _solver() {
		_max_makespan = 64;
		create_encoder("simplified");
		create_encoder("UNSAT-SAT");
	}

	CPFSolver(Instance instance, std::string encoding, std::string search, int verbose = 0)
	: _instance(instance), _solver() {
		_verbose = verbose;
		_max_makespan = 64;
		create_encoder(encoding);
		create_search(search);

	}

	CPFSolver(Instance instance, std::string encoding, std::string search, int max_makespan, int verbose)
	: _instance(instance), _solver() {
		_max_makespan = max_makespan;
		_verbose = verbose;
		create_encoder(encoding);
		create_search(search);
	}

	CPFSolver(std::string filename, std::string encoding, std::string search, int verbose = 0)
	: _solver() {
		Parser p(filename);
		_instance = p.parse();
		_verbose = verbose;
		create_encoder(encoding);
		create_search(search);
	}

	~CPFSolver() { delete _encoder; delete _search; }

	bool solve();

	 int get_initial_makespan() { return 0; }

private:
	bool solve_for_makespan(int makespan);

	void create_encoder(std::string encoding) {
		for(auto &a: encoding) a = tolower(a);

		if(encoding == "simplified")
			_encoder = new SimplifiedEncoder(_instance, &_solver, _verbose);
		else
			std::cerr << "Unknown encoding: " << encoding << std::endl;
	}

	void create_search(std::string search) {
		for(auto &a: search) a = tolower(a);

		if(search == "unsat-sat")
			_search = new UNSAT_SATSearch(_max_makespan);

		else if(search == "sat-unsat")
			_search = new SAT_UNSATSearch(_max_makespan);

		//else if(search == "binary")
		//	_search = new BinarySearch(_max_makespan);

		else
			std::cerr << "Unknown search: " << search << std::endl;
	}

};

#endif
