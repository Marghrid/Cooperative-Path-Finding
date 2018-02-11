#ifndef __SIMPLIFIED_ENCODER__
#define __SIMPLIFIED_ENCODER__

#include "Encoder.h"
#include "simp/SimpSolver.h"
#include "mtl/Vec.h"
#include "core/SolverTypes.h"

class SimplifiedEncoder: public Encoder {
private:
    int _created_vars_makespan = -1;
    int _created_clauses_makespan = -1;
public:

	SimplifiedEncoder(Instance instance, Glucose::SimpSolver *solver, int verbose = 0)
	: Encoder(instance, solver, verbose) {}

	virtual void create_vars_for_makespan(int makespan) override;

	virtual void create_clauses_for_makespan(int makespan) override;

	virtual void create_goal_assumptions(Glucose::vec<Glucose:: Lit> &assumptions, int makespan) override;

	virtual void show_results(int makespan) override;

public:

    Glucose::Var make_xvar_id(int agent_id, int vertex_id, int timestep);

    Glucose::Var make_evar_id(int vertex_id, int timestep);

    int get_agent_id_x(int var_id);

    int get_vertex_id_x(int var_id);

    int get_timestep_x(int var_id);
};

#endif