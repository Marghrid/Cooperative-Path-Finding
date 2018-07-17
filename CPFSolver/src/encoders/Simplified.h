/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Simplified.h:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __SIMPLIFIED_ENCODER__
#define __SIMPLIFIED_ENCODER__

#include "Encoder.h"
#include "simp/SimpSolver.h"
#include "mtl/Vec.h"
#include "core/SolverTypes.h"

class Simplified : public Encoder {
private:
	// Value is incremented before actually creating.
	int _created_vars_makespan = -1;
	// No need to create clauses for timestep 0, since all positions
	//  are completely defined by the instance.
	int _created_clauses_makespan = 0;
public:

	Simplified(Instance instance, Glucose::SimpSolver *solver, int verbose = 0)
			: Encoder(instance, solver, verbose) {}

	void create_vars_for_makespan(int makespan) override;

	void create_clauses_for_makespan(int makespan) override;

	void create_goal_assumptions(Glucose::vec<Glucose::Lit> &assumptions, int makespan) override;

	Solution get_solution(int makespan) override;

public:

	int get_agent_id_x(int var_id);

	int get_vertex_id_x(int var_id);

	int get_timestep_x(int var_id);

	const std::string name() const override { return "Simplified"; }

private:
	Glucose::Var make_xvar_id(int agent_id, int vertex_id, int timestep) const;

	Glucose::Var make_evar_id(int vertex_id, int timestep) const;

};

#endif