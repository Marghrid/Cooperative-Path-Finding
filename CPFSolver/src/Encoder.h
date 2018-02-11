/* encoder.h */
#ifndef __ENCODER__
#define __ENCODER__

#include "Instance.h"
#include "simp/SimpSolver.h"

class Encoder {
protected:

	Instance _instance;
	Glucose::SimpSolver *_solver;
	int _verbose;

public:
	Encoder(Instance instance, Glucose::SimpSolver *solver, int verbose = 0)
	: _instance(instance) {
		_verbose = verbose;
		_solver = solver;
	}

	virtual void create_vars_for_makespan(int makespan) = 0;
	virtual void create_clauses_for_makespan(int makespan) = 0;
	virtual void create_goal_assumptions(Glucose::vec<Glucose:: Lit> &assumptions, int makespan) = 0;
	virtual void show_results(int makespan) = 0;
};

#endif