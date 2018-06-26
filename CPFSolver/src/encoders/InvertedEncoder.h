/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: InvertedEncoder.h:                                  *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __SIMPLIFIED_ENCODER__
#define __SIMPLIFIED_ENCODER__

#include "Encoder.h"
#include "simp/SimpSolver.h"
#include "mtl/Vec.h"
#include "core/SolverTypes.h"

class InvertedEncoder: public Encoder {
private:

public:

	InvertedEncoder(Instance instance, Glucose::SimpSolver *solver, int verbose = 0)
	: Encoder(instance, solver, verbose) {}

	void create_vars_for_makespan(int makespan) override;

	void create_clauses_for_group_makespan(Group *group, int makespan) override;

	void create_clauses_for_makespan(int makespan) override;

	void create_goal_assumptions(Glucose::vec<Glucose:: Lit> &assumptions, int makespan) override;

	Solution get_solution(int makespan) override;

public:
	const std::string name() const override { return "Inverted"; }
};

#endif