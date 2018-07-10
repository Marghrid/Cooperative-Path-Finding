/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: SimplifiedEncoder.h:                                  *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __SIMPLIFIED_ENCODER__
#define __SIMPLIFIED_ENCODER__

#include "Encoder.h"
#include "simp/SimpSolver.h"
#include "mtl/Vec.h"
#include "core/SolverTypes.h"

class SimplifiedEncoder : public Encoder {
private:
	// Value is incremented before actually creating.
	int _created_vars_makespan = -1;
	// No need to create clauses for timestep 0, since all positions
	//  are completely defined by the instance.
	int _created_clauses_makespan = 0;
public:

	SimplifiedEncoder(Instance instance, Glucose::SimpSolver *solver, int verbose = 0)
			: Encoder(instance, solver, verbose) {}

	void create_vars_for_makespan(int makespan) override;

	void create_vars_for_group_makespan(Group *group, int makespan) override;

	void create_clauses_for_makespan(int makespan) override;

	void create_clauses_for_group_makespan(Group *group, int makespan) override;

	void create_goal_assumptions(Glucose::vec<Glucose::Lit> &assumptions, int makespan) override;

	void create_group_goal_assumptions(Group *group, Glucose::vec<Glucose::Lit> &assumptions,
	                                   int makespan) override;

	Solution get_solution(int makespan) override;

	Solution get_group_solution(Group *group, int makespan) override;

	void create_planned_groups_assumptions(std::shared_ptr<Group> group,
		                                       std::list<std::shared_ptr<Group>> planned_groups,
		                                       Glucose::vec<Glucose::Lit> &assumptions,
		                                       int makespan) override;

public:

	Glucose::Var make_xvar_id(int agent_id, int vertex_id, int timestep) const;

	Glucose::Var make_evar_id(int vertex_id, int timestep) const;

	int get_agent_id_x(int var_id);

	int get_vertex_id_x(int var_id);

	int get_timestep_x(int var_id);

	const std::string name() const override { return "Simplified"; }

	Glucose::Var make_group_xvar_id(int agent_id, int vertex_id, int timestep, Group &group) const;

	Glucose::Var make_group_evar_id(int vertex_id, int timestep, Group &group) const;
};

#endif