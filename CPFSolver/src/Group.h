#ifndef __GROUP__
#define __GROUP__

#include <simp/SimpSolver.h>
#include "Instance.h"
#include "Agent.h"
#include "Solution.h"

struct Group {
	std::shared_ptr<Glucose::SimpSolver> solver;
	Solution solution;
	std::vector<Agent *> agents;

	int created_vars_makespan = -1;
	int created_clauses_makespan = 0;

	/* The indices are vertex IDs */
	std::vector<bool> vertex_starts_empty;
	std::vector<bool> vertex_ends_empty;

	Group(Instance &instance) :
			solution(instance),
			vertex_starts_empty(instance.n_vertices(), true),
			vertex_ends_empty(instance.n_vertices(), true){
		solver = std::make_shared<Glucose::SimpSolver>();
		solver->setIncrementalMode();
		solver->verbEveryConflicts = 10;
		solver->verbosity = 0;
	}

	void add_agent(Agent *a) {
		agents.push_back(a);
		vertex_starts_empty[a->initial_position()] = false;
		vertex_ends_empty[a->goal_position()] = false;
	}
};

std::ostream &operator<<(std::ostream &os, const Group &group);


#endif //__GROUP__
