/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Encoder.h:                                            *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __ENCODER__
#define __ENCODER__

#include "Instance.h"
#include "Solution.h"
#include "simp/SimpSolver.h"
#include "Group.h"

class Encoder {
protected:
	Instance _instance;
	Glucose::SimpSolver *_solver;
	int _verbose;

public:
	Encoder(Instance &instance, Glucose::SimpSolver *solver, int verbose = 0)
			: _instance(instance), _solver(solver), _verbose(verbose) {}

	virtual ~Encoder() = default;

	virtual void create_vars_for_makespan(int makespan) = 0;

	virtual void create_vars_for_group_makespan(Group *group, int makespan) = 0;

	virtual void create_clauses_for_makespan(int makespan) = 0;

	virtual void create_clauses_for_group_makespan(Group *group, int makespan) = 0;

	virtual void
	create_group_goal_assumptions(Group *group, Glucose::vec<Glucose::Lit> &assumptions,
	                              int makespan) = 0;

	virtual void create_goal_assumptions(Glucose::vec<Glucose::Lit> &assumptions, int makespan) = 0;

	virtual Solution get_solution(int makespan) = 0;

	virtual Solution get_group_solution(Group *group, int makespan) = 0;

	virtual const std::string name() const = 0;

	void set_solver(Glucose::SimpSolver *solver) { _solver = solver; }


	virtual void
	create_planned_groups_assumptions(std::vector<std::shared_ptr<Group>> planned_groups,
	                                  Glucose::vec<Glucose::Lit> &assumptions, int makespan) = 0;
};

#endif