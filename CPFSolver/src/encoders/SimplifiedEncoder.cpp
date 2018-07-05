/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: SimplifiedEncoder.cpp:                                *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include <CPFSolver.h>
#include "SimplifiedEncoder.h"

void SimplifiedEncoder::create_vars_for_makespan(int makespan) {
	if (_verbose > 1)
		std::cout << "creating vars for makespan " << makespan << std::endl;

	while (_created_vars_makespan < makespan) {
		++_created_vars_makespan;

		for (int j = 0; j < _instance.n_vertices(); ++j) {
			for (unsigned k = 0; k < _instance.n_agents(); ++k) {

				if (_verbose > 2)
					std::cout << "var x for agent " << k
					          << ", vertex " << j
					          << ", and timestep " << _created_vars_makespan
					          << " has id " << make_xvar_id(k, j, _created_vars_makespan);

				while (make_xvar_id(k, j, _created_vars_makespan) >= _solver->nVars())
					_solver->newVar();

				if (_verbose > 2) std::cout << std::endl;
			}
			if (_verbose > 2)
				std::cout << "var epsilon for vertex " << j
				          << ", and timestep " << _created_vars_makespan
				          << " has id " << make_evar_id(j, _created_vars_makespan) << std::endl;

			while (make_evar_id(j, _created_vars_makespan) >= _solver->nVars())
				_solver->newVar();
		}
	}
}

void SimplifiedEncoder::create_clauses_for_makespan(int makespan) {
	unsigned mu = _instance.n_agents();
	unsigned n = _instance.n_vertices();

	if (_created_clauses_makespan == 0) {

		create_vars_for_makespan(0);

		// Initial arrangement:
		if (_verbose > 1)
			std::cout << "Initial arrangement..." << std::endl;
		for (int j = 0; j < n; ++j) {

			// epsilon vars:
			if (_instance.starts_empty(j)) {
				if (_verbose > 2)
					std::cout << "adding clause: e(" << j << ", "
					          << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
				_solver->addClause(Glucose::mkLit(make_evar_id(j, 0)));
			} else {
				if (_verbose > 2)
					std::cout << "adding clause: -e(" << j << ", "
					          << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
				_solver->addClause(Glucose::mkLit(make_evar_id(j, 0), true));
			}

			// X vars (for each agent):
			for (int k = 0; k < mu; ++k) {
				if (j == _instance.agent(k).initial_position()) {
					if (_verbose > 2)
						std::cout << "adding clause: x(" << k << ", " << j << ", "
						          << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
					_solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0)));
				} else {
					if (_verbose > 2)
						std::cout << "adding clause: ~x(" << k << ", " << j << ", "
						          << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
					_solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0), true));
				}
			}
		}
	}

	while (_created_clauses_makespan < makespan) {
		int i = ++_created_clauses_makespan;
		// At most one agent is placed in each vertex at each time step

		create_vars_for_makespan(i);


		// The target vertex is vacant before the move,
		// and the source vertex will be vacant after it
		if (_verbose > 1)
			std::cout << "The target vertex is vacant before the move, \n"
			          << "and the source vertex will be vacant after it..."
			          << std::endl;
		for (int k = 0; k < mu; ++k) {
			for (auto &e: _instance.bidirectional_edges()) {
				Glucose::vec<Glucose::Lit> lit_vec;
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, e.start(), i - 1), true));
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, e.end(), i), true));
				lit_vec.push(Glucose::mkLit(make_evar_id(e.end(), i - 1), false));
				lit_vec.push(Glucose::mkLit(make_evar_id(e.start(), i), false));
				if (_verbose > 2)
					std::cout << "adding clause: ~x(" << k << ", "
					          << e.start() << ", " << i - 1 << ", "
					          << make_xvar_id(k, e.start(), i - 1) << ") V ~x("
					          << k << ", " << e.end() << ", " << i << ", "
					          << make_xvar_id(k, e.end(), i) << ") V e("
					          << e.end() << ", " << i - 1 << ", "
					          << make_evar_id(e.end(), i - 1) << ") V e("
					          << e.start() << ", " << i << ", "
					          << make_evar_id(e.start(), i) << ")" << std::endl;
				_solver->addClause(lit_vec);
			}
		}

		if (_verbose > 1)
			std::cout << "An agent moves along an edge, or not at all..."
			          << std::endl;
		if (_verbose > 1)
			std::cout << "At most one agent is placed in each vertex at each time step..."
			          << std::endl;
		if (_verbose > 1)
			std::cout << "Establishing relation between epsilon and X variables..."
			          << std::endl;

		for (int j = 0; j < n; ++j) {
			Glucose::Lit l3 = Glucose::mkLit(make_evar_id(j, i), true); // relation

			for (int k = 0; k < mu; ++k) {
				// relation between variables
				Glucose::Lit l2 = Glucose::mkLit(make_xvar_id(k, j, i),
				                                 true); // at most and relation
				_solver->addClause(l3, l2); //relation

				if (_verbose > 2) //relation
					std::cout << "adding clause: ~e(" << j << ", " << i << ", "
					          << make_evar_id(j, i) << ") V ~x("
					          << k << ", " << j << ", " << i << ", "
					          << make_xvar_id(k, j, i) << ")" << std::endl;


				// At most one agent is placed in each vertex at each time step
				for (int h = 0; h < k; ++h) {
					Glucose::Lit l1 = Glucose::mkLit(make_xvar_id(h, j, i), true); // at most

					if (_verbose > 2) //at most
						std::cout << "adding clause: ~x(" << h << ", "
						          << j << ", " << i << ", "
						          << make_xvar_id(h, j, i) << ") V ~x("
						          << k << ", " << j << ", " << i << ", "
						          << make_xvar_id(k, j, i) << ")" << std::endl;

					_solver->addClause(l1, l2); //at most
				}

				Glucose::vec<Glucose::Lit> lit_vec; // along an edge
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, j, i - 1), true)); // along an edge
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, j, i), false)); // along an edge
				if (_verbose > 2)
					std::cout << "adding clause: ~x(" << k << ", "
					          << j << ", " << i - 1 << ", "
					          << make_xvar_id(k, j, i - 1) << ") V x("
					          << k << ", " << j << ", " << i << ", "
					          << make_xvar_id(k, j, i) << ")";

				for (auto &v: _instance.get_neighbours(j)) {
					lit_vec.push(Glucose::mkLit(make_xvar_id(k, v, i)));
					if (_verbose > 2)
						std::cout << " V x(" << k << ", "
						          << v << ", " << i << ", "
						          << make_xvar_id(k, v, i) << ")";
				}
				if (_verbose > 2) std::cout << std::endl;
				_solver->addClause(lit_vec);
			}
		}
	}
}

void
SimplifiedEncoder::create_goal_assumptions(Glucose::vec<Glucose::Lit> &assumptions, int makespan) {
	if (_verbose > 1)
		std::cout << "Goal arrangements..." << std::endl;

	for (int j = 0; j < _instance.n_vertices(); ++j) {
		for (unsigned k = 0; k < _instance.n_agents(); ++k) {
			if (j == _instance.agent(k).goal_position()) {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), false));

				if (_verbose > 2)
					std::cout << "Creating assumption: x(" << k << ", " << j << ", "
					          << makespan << ", " << make_xvar_id(k, j, makespan) << ")"
					          << std::endl;
			} else {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), true));

				if (_verbose > 2)
					std::cout << "Creating assumption: ~x(" << k << ", " << j << ", "
					          << makespan << ", " << make_xvar_id(k, j, makespan) << ")"
					          << std::endl;
			}
		}
		if (_instance.ends_empty(j)) {
			if (_verbose > 2)
				std::cout << "adding clause: e(" << j << ", "
				          << 0 << ", " << make_evar_id(j, makespan) << ")" << std::endl;
			assumptions.push(Glucose::mkLit(make_evar_id(j, makespan)));
		} else {
			if (_verbose > 2)
				std::cout << "adding clause: -e(" << j << ", "
				          << 0 << ", " << make_evar_id(j, makespan) << ")" << std::endl;
			assumptions.push(Glucose::mkLit(make_evar_id(j, makespan), true));
		}
	}
}

Solution SimplifiedEncoder::get_solution(int makespan) {
	Solution sol(_instance);
	for (unsigned t = 0; t < makespan + 1; ++t) {
		for (unsigned k = 0; k < _instance.n_agents(); ++k) {
			for (unsigned j = 0; j < _instance.n_vertices(); ++j) {
				if (_solver->modelValue(make_xvar_id(k, j, t)) == l_True) {
					if (_verbose > 2)
						std::cout << "var x(" << t << ", " << j << ", " << k << ", "
						          << make_xvar_id(k, j, t) << ") is true" << std::endl;
					sol.add(k, j, t);
				}
			}
		}
	}
	if (!sol.check())
		std::cerr << "Something went wrong with solution." << std::endl;

	return sol;
}

Solution SimplifiedEncoder::get_group_solution(Group *group, int makespan) {
	for (unsigned t = 0; t < makespan + 1; ++t) {
		for (unsigned k = 0; k < group->agents.size(); ++k) {
			for (unsigned j = 0; j < _instance.n_vertices(); ++j) {
				if (group->solver->modelValue(make_xvar_id(k, j, t)) == l_True) {
					if (_verbose > 2)
						std::cout << "var x(" << t << ", " << j << ", " << k << ", "
						          << make_xvar_id(k, j, t) << ") is true" << std::endl;
					group->solution.add(k, j, t);
				}
			}
		}
	}
	if (!group->solution.check())
		std::cerr << "Something went wrong with solution." << std::endl;

	return group->solution;
}

inline Glucose::Var
SimplifiedEncoder::make_xvar_id(int agent_id, int vertex_id, int timestep) const {
	return static_cast<Glucose::Var>(timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
	                                 + vertex_id * (_instance.n_agents() + 1)
	                                 + agent_id);
}

inline Glucose::Var SimplifiedEncoder::make_evar_id(int vertex_id, int timestep) const {
	return static_cast<Glucose::Var>(timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
	                                 + vertex_id * (_instance.n_agents() + 1)
	                                 + _instance.n_agents());
}

inline int SimplifiedEncoder::get_agent_id_x(int var_id) {
	return static_cast<int>(var_id % (_instance.n_vertices() * (_instance.n_agents() + 1)));
}

inline int SimplifiedEncoder::get_vertex_id_x(int var_id) {
	int int_div = static_cast<int>(std::floor(var_id / (_instance.n_agents() + 1)));
	return int_div % _instance.n_vertices();
}

inline int SimplifiedEncoder::get_timestep_x(int var_id) {
	return static_cast<int>(std::floor(
			var_id / (_instance.n_vertices() * (_instance.n_agents() + 1))));
}

void SimplifiedEncoder::create_clauses_for_group_makespan(Group *group, int makespan) {
	unsigned long mu = group->agents.size();
	unsigned n = _instance.n_vertices();

	if (group->created_clauses_makespan == 0) {

		create_vars_for_group_makespan(group, 0);

		// Initial arrangement:
		if (_verbose > 1)
			std::cout << "Initial arrangement..." << std::endl;
		for (unsigned j = 0; j < n; ++j) {

			// epsilon vars:
			if (group->vertex_starts_empty[j]) {
				if (_verbose > 2)
					std::cout << "adding clause: e(" << j << ", "
					          << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
				group->solver->addClause(Glucose::mkLit(make_evar_id(j, 0)));
			} else {
				if (_verbose > 2)
					std::cout << "adding clause: -e(" << j << ", "
					          << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
				group->solver->addClause(Glucose::mkLit(make_evar_id(j, 0), true));
			}

			// X vars (for each agent):
			for (unsigned k = 0; k < mu; ++k) {
				if (j == group->agents[k]->initial_position()) {
					if (_verbose > 2)
						std::cout << "adding clause: x(" << k << ", " << j << ", "
						          << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
					group->solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0)));
				} else {
					if (_verbose > 2)
						std::cout << "adding clause: ~x(" << k << ", " << j << ", "
						          << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
					group->solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0), true));
				}
			}
		}
	}

	while (group->created_clauses_makespan < makespan) {
		int i = ++group->created_clauses_makespan;

		create_vars_for_group_makespan(group, i);

		// At most one agent is placed in each vertex at each time step
		// The target vertex is vacant before the move,
		// and the source vertex will be vacant after it
		if (_verbose > 1)
			std::cout << "The target vertex is vacant before the move, \n"
			          << "and the source vertex will be vacant after it..."
			          << std::endl;
		for (int k = 0; k < mu; ++k) {
			for (auto &e: _instance.bidirectional_edges()) {
				Glucose::vec<Glucose::Lit> lit_vec;
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, e.start(), i - 1), true));  //FIXME
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, e.end(), i), true));  //FIXME
				lit_vec.push(Glucose::mkLit(make_evar_id(e.end(), i - 1), false));  //FIXME
				lit_vec.push(Glucose::mkLit(make_evar_id(e.start(), i), false));  //FIXME
				if (_verbose > 2)
					std::cout << "adding clause: ~x(" << k << ", "
					          << e.start() << ", " << i - 1 << ", "
					          << make_xvar_id(k, e.start(), i - 1) << ") V ~x("
					          << k << ", " << e.end() << ", " << i << ", "
					          << make_xvar_id(k, e.end(), i) << ") V e("
					          << e.end() << ", " << i - 1 << ", "
					          << make_evar_id(e.end(), i - 1) << ") V e("
					          << e.start() << ", " << i << ", "
					          << make_evar_id(e.start(), i) << ")" << std::endl;
				group->solver->addClause(lit_vec);
			}
		}

		if (_verbose > 1)
			std::cout << "An agent moves along an edge, or not at all..."
			          << std::endl;
		if (_verbose > 1)
			std::cout << "At most one agent is placed in each vertex at each time step..."
			          << std::endl;
		if (_verbose > 1)
			std::cout << "Establishing relation between epsilon and X variables..."
			          << std::endl;

		for (unsigned j = 0; j < n; ++j) {
			Glucose::Lit l3 = Glucose::mkLit(make_evar_id(j, i), true); // relation

			for (unsigned k = 0; k < mu; ++k) {
				// relation between variables
				Glucose::Lit l2 = Glucose::mkLit(make_xvar_id(k, j, i),
				                                 true); // at most and relation
				group->solver->addClause(l3, l2); //relation

				if (_verbose > 2) //relation
					std::cout << "adding clause: ~e(" << j << ", " << i << ", "
					          << make_evar_id(j, i) << ") V ~x("
					          << k << ", " << j << ", " << i << ", "
					          << make_xvar_id(k, j, i) << ")" << std::endl;


				// At most one agent is placed in each vertex at each time step
				for (unsigned h = 0; h < k; ++h) {
					Glucose::Lit l1 = Glucose::mkLit(make_xvar_id(h, j, i), true); // at most

					if (_verbose > 2) //at most
						std::cout << "adding clause: ~x(" << h << ", "
						          << j << ", " << i << ", "
						          << make_xvar_id(h, j, i) << ") V ~x("
						          << k << ", " << j << ", " << i << ", "
						          << make_xvar_id(k, j, i) << ")" << std::endl;

					group->solver->addClause(l1, l2); //at most
				}

				Glucose::vec<Glucose::Lit> lit_vec; // along an edge
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, j, i - 1), true)); // along an edge
				lit_vec.push(Glucose::mkLit(make_xvar_id(k, j, i), false)); // along an edge
				if (_verbose > 2)
					std::cout << "adding clause: ~x(" << k << ", "
					          << j << ", " << i - 1 << ", "
					          << make_xvar_id(k, j, i - 1) << ") V x("
					          << k << ", " << j << ", " << i << ", "
					          << make_xvar_id(k, j, i) << ")";

				for (auto &v: _instance.get_neighbours(j)) {
					lit_vec.push(Glucose::mkLit(make_xvar_id(k, v, i)));
					if (_verbose > 2)
						std::cout << " V x(" << k << ", "
						          << v << ", " << i << ", "
						          << make_xvar_id(k, v, i) << ")";
				}
				if (_verbose > 2) std::cout << std::endl;
				group->solver->addClause(lit_vec);
			}
		}
	}
}

void SimplifiedEncoder::create_vars_for_group_makespan(Group *group, int makespan) {
	if (_verbose > 1)
		std::cout << "creating vars for group " << *group << " for makespan " << makespan
		          << std::endl;

	while (group->created_vars_makespan < makespan) {
		++group->created_vars_makespan;

		for (int j = 0; j < _instance.n_vertices(); ++j) {
			for (unsigned k = 0; k < group->agents.size(); ++k) {

				if (_verbose > 2)
					std::cout << "var x for agent " << k
					          << ", vertex " << j
					          << ", and timestep " << group->created_vars_makespan
					          << " has id " << make_xvar_id(k, j, group->created_vars_makespan);

				while (make_xvar_id(k, j, group->created_vars_makespan) >= group->solver->nVars())
					group->solver->newVar();

				if (_verbose > 2) std::cout << std::endl;
			}
			if (_verbose > 2)
				std::cout << "var epsilon for vertex " << j
				          << ", and timestep " << group->created_vars_makespan
				          << " has id " << make_evar_id(j, group->created_vars_makespan)
				          << std::endl;

			while (make_evar_id(j, group->created_vars_makespan) >= group->solver->nVars())
				group->solver->newVar();
		}
	}
}

void SimplifiedEncoder::create_group_goal_assumptions(Group *group,
                                                      Glucose::vec<Glucose::Lit> &assumptions,
                                                      int makespan) {
	if (_verbose > 1)
		std::cout << "Goal arrangements for group " << *group << std::endl;

	for (int j = 0; j < _instance.n_vertices(); ++j) {
		for (unsigned k = 0; k < group->agents.size(); ++k) {
			if (j == group->agents[k]->goal_position()) {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), false));

				if (_verbose > 2)
					std::cout << "Creating assumption: x(" << k << ", " << j << ", "
					          << makespan << ", " << make_xvar_id(k, j, makespan) << ")"
					          << std::endl;
			} else {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), true));

				if (_verbose > 2)
					std::cout << "Creating assumption: ~x(" << k << ", " << j << ", "
					          << makespan << ", " << make_xvar_id(k, j, makespan) << ")"
					          << std::endl;
			}
		}
		if (group->vertex_ends_empty[j]) {
			if (_verbose > 2)
				std::cout << "adding clause: e(" << j << ", "
				          << 0 << ", " << make_evar_id(j, makespan) << ")" << std::endl;
			assumptions.push(Glucose::mkLit(make_evar_id(j, makespan)));
		} else {
			if (_verbose > 2)
				std::cout << "adding clause: ~e(" << j << ", "
				          << 0 << ", " << make_evar_id(j, makespan) << ")" << std::endl;
			assumptions.push(Glucose::mkLit(make_evar_id(j, makespan), true));
		}
	}
}

void SimplifiedEncoder::create_planned_groups_assumptions(
		std::vector<std::shared_ptr<Group>> planned_groups, Glucose::vec<Glucose::Lit> &assumptions,
		int makespan) {

}
