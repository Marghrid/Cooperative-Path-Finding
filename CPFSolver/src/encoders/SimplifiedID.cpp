/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: SimplifiedID.cpp:                                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "SimplifiedID.h"

inline Glucose::Var
SimplifiedID::make_xvar_id(int agent_id, int vertex_id, int timestep, const Group &group) const {
	return static_cast<Glucose::Var>(timestep * _instance.n_vertices() * (group.n_agents() + 1)
	                                 + vertex_id * (group.n_agents() + 1)
	                                 + agent_id);
}

inline Glucose::Var SimplifiedID::make_evar_id(int vertex_id, int timestep, const Group &group) const {
	return static_cast<Glucose::Var>(timestep * _instance.n_vertices() * (group.n_agents() + 1)
	                                 + vertex_id * (group.n_agents() + 1)
	                                 + group.n_agents());
}

void SimplifiedID::create_vars_for_makespan(Group *group, int makespan) {
	if (_verbose > 2)
		std::cout << "Creating vars for group " << *group << " for makespan " << makespan
		          << std::endl;

	while (group->created_vars_makespan < makespan) {
		++group->created_vars_makespan;

		for (unsigned j = 0; j < _instance.n_vertices(); ++j) {
			for (unsigned k = 0; k < group->agents.size(); ++k) {

				if (_verbose > 3)
					std::cout << "var x for agent " << k
					          << ", vertex " << j
					          << ", and timestep " << group->created_vars_makespan
					          << " has id "
					          << make_xvar_id(k, j, group->created_vars_makespan, *group);

				while (make_xvar_id(k, j, group->created_vars_makespan, *group) >= group->solver->nVars())
					group->solver->newVar();

				if (_verbose > 3) std::cout << std::endl;
			}
			if (_verbose > 3)
				std::cout << "var epsilon for vertex " << j
				          << ", and timestep " << group->created_vars_makespan
				          << " has id "
				          << make_evar_id(j, group->created_vars_makespan, *group)
				          << std::endl;

			while (make_evar_id(j, group->created_vars_makespan, *group) >= group->solver->nVars())
				group->solver->newVar();
		}
	}
}

void SimplifiedID::create_clauses_for_makespan(Group *group, int makespan) {
	unsigned long mu = group->agents.size();
	unsigned n = _instance.n_vertices();

	if (group->created_clauses_makespan == 0) {
		create_vars_for_makespan(group, 0);
		create_initial_arrangement_clauses(group, mu, n);
	}

	while (group->created_clauses_makespan < makespan) {
		int i = ++group->created_clauses_makespan;
		create_vars_for_makespan(group, i);

		// At most one agent is placed in each vertex at each time step
		// The target vertex is vacant before the move,
		// and the source vertex will be vacant after it
		if (_verbose > 3)
			std::cout << "The target vertex is vacant before the move, \n"
			          << "and the source vertex will be vacant after it..."
			          << std::endl;
		for (unsigned k = 0; k < mu; ++k) {
			for (auto &e: _instance.bidirectional_edges()) {
				Glucose::vec<Glucose::Lit> lit_vec1;
				Glucose::vec<Glucose::Lit> lit_vec2;

				Glucose::Lit x1 = Glucose::mkLit(make_xvar_id(k, e.start(), i - 1, *group), true);
				Glucose::Lit x2 = Glucose::mkLit(make_xvar_id(k, e.end(), i, *group), true);

				lit_vec1.push(x1);
				lit_vec1.push(x2);
				lit_vec1.push(Glucose::mkLit(make_evar_id(e.end(), i - 1, *group), false));

				lit_vec2.push(x1);
				lit_vec2.push(x2);
				lit_vec2.push(Glucose::mkLit(make_evar_id(e.start(), i, *group), false));

				if (_verbose > 3) {
					std::cout << "adding clause: ~x(" << k << ", " << e.start() << ", " << i - 1 << ", "
					          << make_xvar_id(k, e.start(), i - 1, *group) << ") V ~x("
					          << k << ", " << e.end() << ", " << i << ", "
					          << make_xvar_id(k, e.end(), i, *group) << ") V e("
					          << e.end() << ", " << i - 1 << ", "
					          << make_evar_id(e.end(), i - 1, *group) << ")" << std::endl;

					std::cout << "adding clause: ~x(" << k << ", " << e.start() << ", " << i - 1 << ", "
					          << make_xvar_id(k, e.start(), i - 1, *group) << ") V ~x("
					          << k << ", " << e.end() << ", " << i << ", "
					          << make_xvar_id(k, e.end(), i, *group) << ")V e("
					          << e.start() << ", " << i << ", "
					          << make_evar_id(e.start(), i, *group) << ")" << std::endl;
				}

				group->solver->addClause(lit_vec1);
				group->solver->addClause(lit_vec2);
			}
		}

		if (_verbose > 3)
			std::cout << "An agent moves along an edge, or not at all..."
			          << std::endl;
		if (_verbose > 3)
			std::cout << "At most one agent is placed in each vertex at each time step..."
			          << std::endl;
		if (_verbose > 3)
			std::cout << "Establishing relation between epsilon and X variables..."
			          << std::endl;

		for (unsigned j = 0; j < n; ++j) {
			Glucose::Lit l3 = Glucose::mkLit(make_evar_id(j, i, *group), true); // relation

			for (unsigned k = 0; k < mu; ++k) {
				// relation between variables
				Glucose::Lit l2 = Glucose::mkLit(make_xvar_id(k, j, i, *group), true); // at most and relation
				group->solver->addClause(l3, l2); //relation

				if (_verbose > 3) //relation
					std::cout << "adding clause: ~e(" << j << ", " << i << ", "
					          << make_evar_id(j, i, *group) << ") V ~x("
					          << k << ", " << j << ", " << i << ", "
					          << make_xvar_id(k, j, i, *group) << ")" << std::endl;


				// At most one agent is placed in each vertex at each time step
				for (unsigned h = 0; h < k; ++h) {
					Glucose::Lit l1 = Glucose::mkLit(make_xvar_id(h, j, i, *group), true); // at most

					if (_verbose > 3) //at most
						std::cout << "adding clause: ~x(" << h << ", "
						          << j << ", " << i << ", "
						          << make_xvar_id(h, j, i, *group) << ") V ~x("
						          << k << ", " << j << ", " << i << ", "
						          << make_xvar_id(k, j, i, *group) << ")" << std::endl;

					group->solver->addClause(l1, l2); //at most
				}

				Glucose::vec<Glucose::Lit> lit_vec; // along an edge
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, j, i - 1, *group), true)); // along an edge
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, j, i, *group), false)); // along an edge
				if (_verbose > 3)
					std::cout << "adding clause: ~x(" << k << ", "
					          << j << ", " << i - 1 << ", "
					          << make_xvar_id(k, j, i - 1, *group) << ") V x("
					          << k << ", " << j << ", " << i << ", "
					          << make_xvar_id(k, j, i, *group) << ")";

				for (auto &v: _instance.get_neighbours(j)) {
					lit_vec.push(Glucose::mkLit(make_xvar_id(k, v, i, *group)));
					if (_verbose > 3)
						std::cout << " V x(" << k << ", " << v << ", " << i << ", "
						          << make_xvar_id(k, v, i, *group) << ")";
				}
				if (_verbose > 3) std::cout << std::endl;
				group->solver->addClause(lit_vec);
			}
		}
	}
}

void SimplifiedID::create_initial_arrangement_clauses(const Group *group, unsigned long mu, unsigned int n) const {
	// Initial arrangement:
	if (_verbose > 3)
		std::cout << "Initial arrangement..." << std::endl;
	for (unsigned j = 0; j < n; ++j) {

		// epsilon vars:
		if (group->vertex_starts_empty[j]) {
			if (_verbose > 3)
				std::cout << "adding clause: e(" << j << ", "
				               << 0 << ", " << make_evar_id(j, 0, *group) << ")" << std::endl;
			group->solver->addClause(Glucose::mkLit(make_evar_id(j, 0, *group)));
		} else {
			if (_verbose > 3)
				std::cout << "adding clause: ~e(" << j << ", "
				               << 0 << ", " << make_evar_id(j, 0, *group) << ")" << std::endl;
			group->solver->addClause(Glucose::mkLit(make_evar_id(j, 0, *group), true));
		}

		// X vars (for each agent):
		for (unsigned k = 0; k < mu; ++k) {
			if (j == group->agents[k]->initial_position()) {
				if (_verbose > 3)
					std::cout << "adding clause: x(" << k << ", " << j << ", " << 0 << ", "
					               << make_xvar_id(k, j, 0, *group) << ")" << std::endl;
				group->solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0, *group)));
			} else {
				if (_verbose > 3)
					std::cout << "adding clause: ~x(" << k << ", " << j << ", " << 0 << ", "
					               << make_xvar_id(k, j, 0, *group) << ")" << std::endl;
				group->solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0, *group), true));
			}
		}
	}
}

void SimplifiedID::create_goal_assumptions(Group *group,
                                           Glucose::vec<Glucose::Lit> &assumptions,
                                           int makespan) {
	if (_verbose > 1)
		std::cout << "Goal arrangements for group " << *group << std::endl;

	for (int j = 0; j < _instance.n_vertices(); ++j) {
		for (unsigned k = 0; k < group->agents.size(); ++k) {
			if (j == group->agents[k]->goal_position()) {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan, *group), false));
				// This should make sense: If it's no one's goal position, it should end empty.
				// assumptions.push(Glucose::mkLit(make_evar_id(j, makespan, *group), true));

				if (_verbose > 3)
					std::cout << "Creating assumption: x(" << k << ", " << j << ", " << makespan << ", "
					          << make_xvar_id(k, j, makespan, *group) << ")" << std::endl;
			} else {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan, *group), true));

				if (_verbose > 3)
					std::cout << "Creating assumption: ~x(" << k << ", " << j << ", " << makespan << ", "
					          << make_xvar_id(k, j, makespan, *group) << ")" << std::endl;
			}
		}
		// This is the original solution
		if (group->vertex_ends_empty[j]) {
			if (_verbose > 3)
				std::cout << "Creating assumption: e(" << j << ", "
				          << makespan << ", " << make_evar_id(j, makespan, *group) << ")" << std::endl;
			assumptions.push(Glucose::mkLit(make_evar_id(j, makespan, *group)));
		} else {
			if (_verbose > 3)
				std::cout << "Creating assumption: ~e(" << j << ", "
				          << makespan << ", " << make_evar_id(j, makespan, *group) << ")" << std::endl;
			assumptions.push(Glucose::mkLit(make_evar_id(j, makespan, *group), true));
		}
	}
}

void SimplifiedID::create_planned_groups_assumptions(std::shared_ptr<Group> group,
                                                     std::vector<std::shared_ptr<Group>> planned_groups,
                                                     Glucose::vec<Glucose::Lit> &assumptions) {

	if (_verbose > 3)
		std::cout << "Planned groups' assumptions for group " << *group << std::endl;

	for (std::shared_ptr<Group> &planned_group : planned_groups) {
		// for each planned group add X and eps vars for each value in solution
		// an x var for each agent, vertex and timestep,
		// an eps var for each vertex and timestep

		for (std::shared_ptr<Agent> &planned_agent : planned_group->agents) {
			for (unsigned t = 0; t < planned_group->solution.n_timesteps(); ++t) {
				for (unsigned k = 0; k < group->n_agents(); ++k) {
					// All groups' agents are not where planned groups' agents are.
					if (_verbose > 3)
						std::cout << "Creating assumption: ~x(" << k << ", "
						          << planned_group->solution.get_position(planned_agent, t) << ", " << t << ", "
						          << make_xvar_id(k, planned_group->solution.get_position(planned_agent, t), t,
						                          *group) << ")" << std::endl;
					assumptions.push(Glucose::mkLit(
							make_xvar_id(k, planned_group->solution.get_position(planned_agent, t), t, *group),
							true));
				}

				/*if (_verbose > 3)
					std::cout << "Creating assumption: ~e("
					          << planned_group->solution.get_position(planned_agent, t) << ", " << t << ", "
					          << make_evar_id(planned_group->solution.get_position(planned_agent, t), t, *group)
					          << ")" << std::endl;

				assumptions.push(Glucose::mkLit(
						make_evar_id(planned_group->solution.get_position(planned_agent, t), t, *group), true));*/
			}
		}
	}
}


Solution SimplifiedID::get_solution(Group *group, int makespan) {
	for (unsigned t = 0; t < makespan + 1; ++t) {
		for (unsigned k = 0; k < group->agents.size(); ++k) {
			for (unsigned j = 0; j < _instance.n_vertices(); ++j) {
				if (group->solver->modelValue(make_xvar_id(k, j, t, *group)) == l_True) {
					if (_verbose > 3)
						std::cout << "var x(" << k << ", " << j << ", " << t << ", "
						          << make_xvar_id(k, j, t, *group) << ") is true" << std::endl;
					group->solution.add(group->agents[k]->id(), j, t);
				}
			}
		}
	}
	if (!group->solution.check())
		std::cerr << "Something went wrong with solution." << std::endl;

	return group->solution;
}
