/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: SimplifiedID.h: All operations related to the         *
 *   Encoding, operating with groups as to allow for an        *
 *   Independence Detection solution.                          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __SIMPLIFIED_ID__
#define __SIMPLIFIED_ID__


#include <Group.h>
#include <EncoderID.h>

class SimplifiedID : public EncoderID {
public:

	SimplifiedID(Instance instance, Glucose::SimpSolver *solver, int verbose = 0)
	: EncoderID(instance, solver, verbose) {}

	void create_vars_for_makespan(Group *group, int makespan) override;

	void create_clauses_for_makespan(Group *group, int makespan) override;

	void create_goal_assumptions(Group *group, Glucose::vec<Glucose::Lit> &assumptions,
	                                   int makespan) override;

	Solution get_solution(Group *group, int makespan) override;

	void create_planned_groups_assumptions(std::shared_ptr<Group> group,
		                                       std::list<std::shared_ptr<Group>> planned_groups,
		                                       Glucose::vec<Glucose::Lit> &assumptions) override;

	const std::string name() const override { return "SimplifiedID"; }

private:
	Glucose::Var make_xvar_id(int agent_id, int vertex_id, int timestep, const Group &group) const;

	Glucose::Var make_evar_id(int vertex_id, int timestep, const Group &group) const;

	void create_initial_arrangement_clauses(const Group *group, unsigned long mu, unsigned int n) const;
};


#endif //__SIMPLIFIED_ID__
