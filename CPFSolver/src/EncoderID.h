//
// Created by Rui Cruz Ferreira on 11/07/2018.
//

#include <simp/SimpSolver.h>
#include "Instance.h"
#include "Group.h"

#ifndef __ENCODER_ID__
#define __ENCODER_ID__

class EncoderID {
protected:
	Instance _instance;
	Glucose::SimpSolver *_solver;
	int _verbose;

public:
	EncoderID(Instance &instance, Glucose::SimpSolver *solver, int verbose = 0)
			: _instance(instance), _solver(solver), _verbose(verbose) {}

	virtual ~EncoderID() = default;

	virtual void create_vars_for_makespan(Group *group, int makespan) = 0;

	virtual void create_clauses_for_makespan(Group *group, int makespan) = 0;

	virtual void
	create_goal_assumptions(Group *group, Glucose::vec<Glucose::Lit> &assumptions,
	                              int makespan) = 0;

	virtual Solution get_solution(Group *group, int makespan) = 0;

	virtual const std::string name() const = 0;

	void set_solver(Glucose::SimpSolver *solver) { _solver = solver; }

	virtual void
	create_planned_groups_assumptions(std::shared_ptr<Group> group,
		                                  std::list<std::shared_ptr<Group>> planned_groups,
		                                  Glucose::vec<Glucose::Lit> &assumptions) = 0;
};

#endif //__ENCODER_ID__
