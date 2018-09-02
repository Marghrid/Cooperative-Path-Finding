/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Group.h                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __GROUP__
#define __GROUP__

#include <simp/SimpSolver.h>
#include "Instance.h"
#include "Agent.h"
#include "Solution.h"

struct Group {
	std::shared_ptr<Glucose::SimpSolver> solver;
	Solution solution;
	std::vector<std::shared_ptr<Agent>> agents;

	int created_vars_makespan = -1;
	int created_clauses_makespan = 0;
	int last_solved_makespan = -1;

	/* The indices are vertex IDs */
	std::vector<bool> vertex_starts_empty;
	std::vector<bool> vertex_ends_empty;

	explicit Group(Instance &instance) :
			solution(instance),
			vertex_starts_empty(instance.n_vertices(), true),
			vertex_ends_empty(instance.n_vertices(), true){
		solver = std::make_shared<Glucose::SimpSolver>();
		solver->setIncrementalMode();
		solver->verbEveryConflicts = 10;
		solver->verbosity = 0;
	}

	virtual ~Group() {
	}

	unsigned long n_agents() const {
		return agents.size();
	}
	Agent &agent(unsigned a_id) const {
		for (const std::shared_ptr<Agent> &a : agents) {
			if (a->id() == a_id) return *a;
		}

		throw std::runtime_error(std::string("Nonexistent agent #" + std::to_string(a_id)));
	}

	void add_agent(std::shared_ptr<Agent> &a) {
		agents.push_back(a);
		vertex_starts_empty[a->initial_position()] = false;
		vertex_ends_empty[a->goal_position()] = false;
	}
};

std::ostream &operator<<(std::ostream &os, const Group &group);


#endif //__GROUP__
