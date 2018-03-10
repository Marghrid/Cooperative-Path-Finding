/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: SimplifiedEncoder.cpp:                                *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "SimplifiedEncoder.h"

void SimplifiedEncoder::create_vars_for_makespan(int makespan) {
    if (_verbose > 0)
        std::cout << "creating vars for makespan " << makespan << std::endl;

    while (_created_vars_makespan < makespan) {
        ++_created_vars_makespan;

        for (int j = 0; j < _instance.n_vertices(); ++j) {
            for (unsigned k = 0; k < _instance.n_agents(); ++k) {

                if (_verbose > 1)
                    std::cout << "var x for agent " << k
                              << ", vertex " << j
                              << ", and timestep " << _created_vars_makespan
                              << " has id " << make_xvar_id(k, j, _created_vars_makespan);

                while (make_xvar_id(k, j, _created_vars_makespan) >= _solver->nVars())
                    _solver->newVar();

                if (_verbose > 1) std::cout << std::endl;
            }
            if (_verbose > 1)
                std::cout << "var epsilon for vertex " << j
                          << ", and timestep " << _created_vars_makespan
                          << " has id " << make_evar_id(j, _created_vars_makespan) << std::endl;

            while (make_evar_id(j, _created_vars_makespan) >= _solver->nVars())
                _solver->newVar();
        }
    }
}

void SimplifiedEncoder::create_clauses_for_makespan(int makespan) {
    int mu = _instance.n_agents();
    int n = _instance.n_vertices();

    if (_created_clauses_makespan == 0) {

        create_vars_for_makespan(0);

        // Initial arragement:
        if (_verbose > 0)
            std::cout << "Initial arragement..." << std::endl;
        for (int j = 0; j < n; ++j) {

            // epsilon vars:
            if (_instance.starts_empty(j)) {
                if (_verbose > 1)
                    std::cout << "adding clause: e(" << j << ", "
                              << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
                _solver->addClause(Glucose::mkLit(make_evar_id(j, 0)));
            } else {
                if (_verbose > 1)
                    std::cout << "adding clause: -e(" << j << ", "
                              << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
                _solver->addClause(Glucose::mkLit(make_evar_id(j, 0), true));
            }

            // X vars (for each agent):
            for (int k = 0; k < mu; ++k) {
                if (j == _instance.agent(k).initial_position()) {
                    if (_verbose > 1)
                        std::cout << "adding clause: x(" << k << ", " << j << ", "
                                  << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
                    _solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0)));
                } else {
                    if (_verbose > 1)
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
        if (_verbose > 0)
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
                if (_verbose > 1)
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

        if (_verbose > 0)
            std::cout << "An agent moves along an edge, or not at all..."
                      << std::endl;
        if (_verbose > 0)
            std::cout << "At most one agent is placed in each vertex at each time step..."
                      << std::endl;
        if (_verbose > 0)
            std::cout << "Establishing relation between epsilon and X variables..."
                      << std::endl;

        for (int j = 0; j < n; ++j) {
            Glucose::Lit l3 = Glucose::mkLit(make_evar_id(j, i), true); // relation

            for (int k = 0; k < mu; ++k) {
                // relation between variables
                Glucose::Lit l2 = Glucose::mkLit(make_xvar_id(k, j, i), true); // at most and relation
                _solver->addClause(l3, l2); //relation

                if (_verbose > 1) //relation
                    std::cout << "adding clause: ~e(" << j << ", " << i << ", "
                              << make_evar_id(j, i) << ") V ~x("
                              << k << ", " << j << ", " << i << ", "
                              << make_xvar_id(k, j, i) << ")" << std::endl;


                // At most one agent is placed in each vertex at each time step
                for (int h = 0; h < k; ++h) {
                    Glucose::Lit l1 = Glucose::mkLit(make_xvar_id(h, j, i), true); // at most

                    if (_verbose > 1) //at most
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
                if (_verbose > 1)
                    std::cout << "adding clause: ~x(" << k << ", "
                              << j << ", " << i - 1 << ", "
                              << make_xvar_id(k, j, i - 1) << ") V x("
                              << k << ", " << j << ", " << i << ", "
                              << make_xvar_id(k, j, i) << ")";

                for (auto &v: _instance.get_neighbours(j)) {
                    lit_vec.push(Glucose::mkLit(make_xvar_id(k, v, i)));
                    if (_verbose > 1)
                        std::cout << " V x(" << k << ", "
                                  << v << ", " << i << ", "
                                  << make_xvar_id(k, v, i) << ")";
                }
                if (_verbose > 1) std::cout << std::endl;
                _solver->addClause(lit_vec);
            }
        }
    }
}

void SimplifiedEncoder::create_goal_assumptions(Glucose::vec<Glucose::Lit> &assumptions, int makespan) {
    if (_verbose > 0)
        std::cout << "Goal arragements..." << std::endl;

    for (int j = 0; j < _instance.n_vertices(); ++j) {
        for (unsigned k = 0; k < _instance.n_agents(); ++k) {
            if (j == _instance.agent(k).goal_position()) {
                assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), false));

                if (_verbose > 1)
                    std::cout << "Creating assumption: x(" << k << ", " << j << ", "
                              << makespan << ", " << make_xvar_id(k, j, makespan) << ")" << std::endl;
            } else {
                assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), true));

                if (_verbose > 1)
                    std::cout << "Creating assumption: ~x(" << k << ", " << j << ", "
                              << makespan << ", " << make_xvar_id(k, j, makespan) << ")" << std::endl;
            }
        }
        if (_instance.ends_empty(j)) {
            if (_verbose > 1)
                std::cout << "adding clause: e(" << j << ", "
                          << 0 << ", " << make_evar_id(j, makespan) << ")" << std::endl;
            assumptions.push(Glucose::mkLit(make_evar_id(j, makespan)));
        } else {
            if (_verbose > 1)
                std::cout << "adding clause: -e(" << j << ", "
                          << 0 << ", " << make_evar_id(j, makespan) << ")" << std::endl;
            assumptions.push(Glucose::mkLit(make_evar_id(j, makespan), true));
        }
    }
}

Solution SimplifiedEncoder::get_solution(int makespan) {
    Solution sol(_instance);
    for (int i = 0; i < makespan + 1; ++i) {
        sol.increment_timestep();
        for (unsigned k = 0; k < _instance.n_agents(); ++k) {
            for (int j = 0; j < _instance.n_vertices(); ++j) {
                if (_solver->modelValue(make_xvar_id(k, j, i)) == l_True) {
                    if (_verbose > 1)
                        std::cout << "var x(" << i << ", " << j << ", " << k << ", "
                                  << make_xvar_id(k, j, i) << ") is true" << std::endl;
                    sol.add(k, j);
                }
            }
        }
    }
    if (!sol.check())
        std::cerr << "Something went wrong with solution." << std::endl;

    return sol;
}

inline Glucose::Var SimplifiedEncoder::make_xvar_id(int agent_id, int vertex_id, int timestep) const {
    return timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
           + vertex_id * (_instance.n_agents() + 1)
           + agent_id;
}

inline Glucose::Var SimplifiedEncoder::make_evar_id(int vertex_id, int timestep) const {
    return timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
           + vertex_id * (_instance.n_agents() + 1)
           + _instance.n_agents();
}

inline int SimplifiedEncoder::get_agent_id_x(int var_id) {
    return var_id % (_instance.n_vertices() * (_instance.n_agents() + 1));
}

inline int SimplifiedEncoder::get_vertex_id_x(int var_id) {
    int int_div = static_cast<int>(std::floor(var_id / (_instance.n_agents() + 1)));
    return int_div % _instance.n_vertices();
}

inline int SimplifiedEncoder::get_timestep_x(int var_id) {
    return static_cast<int>(std::floor(var_id / (_instance.n_vertices() * (_instance.n_agents() + 1))));
}