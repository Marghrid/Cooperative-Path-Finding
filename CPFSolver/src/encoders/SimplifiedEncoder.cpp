#include "SimplifiedEncoder.h"

#include "Instance.h"
#include "Solution.h"
#include "core/SolverTypes.h"
#include "mtl/Vec.h"
#include "simp/SimpSolver.h"

void SimplifiedEncoder::create_vars_for_makespan(int makespan) {
    if(_verbose > 0)
        std::cout << "creating vars for makespan " << makespan << std::endl;

    while(_created_vars_makespan < makespan) {
        ++_created_vars_makespan;

        for(int j = 0; j < _instance.n_vertices(); ++j) {
            for(int k = 0; k < _instance.n_agents(); ++k) {

                if(_verbose > 1) 
                    std::cout << "var x for agent " << k
                        << ", vertex " << j
                        << ", and timestep " << _created_vars_makespan
                        << " has id " << make_xvar_id(k, j, _created_vars_makespan);

                while (make_xvar_id(k, j, _created_vars_makespan) >= _solver->nVars())
                    _solver->newVar();

                if(_verbose > 1) std::cout << std::endl;
            }
            if(_verbose > 1) 
                std::cout << "var epsilon for vertex " << j
                    << ", and timestep " << _created_vars_makespan
                    << " has id " << make_evar_id(j, _created_vars_makespan) << std::endl;

            while (make_evar_id(j, _created_vars_makespan) >= _solver->nVars())
                _solver->newVar();
        }
    }
}

void SimplifiedEncoder::create_clauses_for_makespan(int makespan) {
	int mu  = _instance.n_agents();
    int n   = _instance.n_vertices();

	while(_created_clauses_makespan < makespan) {
        int i = ++_created_clauses_makespan;
        // At most one agent is placed in each vertex at each time step

    	create_vars_for_makespan(_created_clauses_makespan);

        if(_created_clauses_makespan == 0) {
        	// Initial arragement:
			if(_verbose > 0)
			    std::cout << "Initial arragement..." << std::endl;
			for(int j = 0; j < _instance.n_vertices(); ++j) {
			    for(int k = 0; k < _instance.n_agents(); ++k) {
			        if(j == _instance.agent(k).initial_position().id()) {
			            if(_verbose > 1)
			                std::cout << "adding clause: x(" << k << ", " << j << ", "
			                    << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
			            _solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0)));
			        } else {
			            if(_verbose > 1)
			                std::cout << "adding clause: ~x(" << k << ", " << j << ", "
			                    << 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
			            _solver->addClause(Glucose::mkLit(make_xvar_id(k, j, 0), true));
			        }
			    }

			    if(_instance.starts_empty(j)) {
			        if(_verbose > 1)
			            std::cout << "adding clause: e(" << j << ", "
			                << 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
			        _solver->addClause(Glucose::mkLit(make_evar_id(j, 0)));
			    }
			}
        }

        if(_verbose > 0)
            std::cout << "At most one agent is placed in each vertex at each time step..."
                << std::endl;

        for(int j = 0; j < n; ++j) {
            for(int h = 0; h < mu; ++h) {
                for(int k = 0; k < h; ++k) {
                    Glucose::Lit l1 = Glucose::mkLit(make_xvar_id(k, j, i), true);
                    Glucose::Lit l2 = Glucose::mkLit(make_xvar_id(h, j, i), true);

                    if(_verbose > 1)
                        std::cout << "adding clause: ~x(" << k << ", "
                            << j << ", " << i << ", "
                            << make_xvar_id(k, j, i) << ") V ~x("
                            << h << ", " << j << ", " << i << ", "
                            << make_xvar_id(h, j, i) << ")" << std::endl;

                    _solver->addClause(l1, l2);
                }
            }
        }

        if(i > 0) /* Clause that relate current timestep i with timestep i-1 */{
            // An agent moves along an edge, or not at all:
            if(_verbose > 0)
                std::cout << "An agent moves along an edge, or not at all..."
                << std::endl;
            for(int j = 0; j < n; ++j){
                for(int k = 0; k < mu; ++k) {
                    Glucose::vec<Glucose::Lit> lit_vec;
                    lit_vec.push( Glucose::mkLit(make_xvar_id(k, j, i-1), true) );
                    lit_vec.push( Glucose::mkLit(make_xvar_id(k, j, i  ), false) );
                    if(_verbose > 1)
                        std::cout << "adding clause: ~x(" << k << ", "
                            << j << ", " << i-1 << ", "
                            << make_xvar_id(k, j, i-1) << ") V x("
                            << k << ", " << j << ", " << i << ", "
                            << make_xvar_id(k, j, i) << ") ";

                    for(auto &v: _instance.get_neighbours(j)) {
                        lit_vec.push( Glucose::mkLit(make_xvar_id(k, v.id(), i)) );
                        if(_verbose > 1)
                            std::cout << "V x(" << k << ", "
                                << v.id() << ", " << i << ", "
                                << make_xvar_id(k, v.id(), i) << ")";
                    }
                    if(_verbose > 1) std::cout << std::endl;
                    _solver->addClause(lit_vec);
                }
            }
        
            // The target vertex is vacant before the move,
            // and the source vertex will be vacant after it
            if(_verbose > 0)
                std::cout << "The target vertex is vacant before the move, \n"
                    << "and the source vertex will be vacant after it..."
                    << std::endl;
            for(int k = 0; k < mu; ++k) {
                for(auto &e: _instance.bidirectional_edges()) {
                    Glucose::vec<Glucose::Lit> lit_vec;
                    lit_vec.push( Glucose::mkLit(make_xvar_id(k, e.start().id(), i-1), true)  );
                    lit_vec.push( Glucose::mkLit(make_xvar_id(k, e.end().id(),   i  ), true)  );
                    lit_vec.push( Glucose::mkLit(make_evar_id(e.end().id(),      i-1), false) );
                    lit_vec.push( Glucose::mkLit(make_evar_id(e.start().id(),    i  ), false) );
                    if(_verbose > 1)
                        std::cout << "adding clause: ~x(" << k << ", "
                            << e.start().id() << ", " << i-1 << ", "
                            << make_xvar_id(k, e.start().id(), i-1) << ") V ~x("
                            << k << ", " << e.end().id() << ", " << i << ", "
                            << make_xvar_id(k, e.end().id(), i  ) << ") V e("
                            << e.end().id() << ", " << i-1 << ", "
                            << make_evar_id(e.end().id(), i-1) << ") V e("
                            << e.start().id() << ", " << i << ", "
                            << make_evar_id(e.start().id(), i) <<")" << std::endl;
                    _solver->addClause(lit_vec);
                }
            }
        }

        // relation between variables
        if(_verbose > 0)
            std::cout << "Establishing relation between epsilon and X variables..." << std::endl;
        for(int j = 0; j < n; ++j) {
            Glucose::Lit l1 = Glucose::mkLit(make_evar_id(j, i), true);
            for(int k = 0; k < _instance.n_agents(); ++k) {
                _solver->addClause(l1, Glucose::mkLit(make_xvar_id(k, j, i), true));
                if(_verbose > 1)
                    std::cout << "adding clause: ~e(" << j << ", " << i << ", "
                        << make_evar_id(j, i) << ") V ~x("
                        << k << ", " << j << ", " << i << ", "
                        << make_xvar_id(k, j, i) << ")" << std::endl;
            }
        }
    }
}

void SimplifiedEncoder::create_goal_assumptions(Glucose::vec<Glucose:: Lit> &assumptions, int makespan) {
	if(_verbose > 0)
        std::cout << "Goal arragements..." << std::endl;
    
    for(int k = 0; k < _instance.n_agents(); ++k) {
        for(int j = 0; j < _instance.n_vertices(); ++j) {
            if(j == _instance.agent(k).goal_position().id()) {
                assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), false) );
                
                if(_verbose > 1)
                    std::cout << "Creating assumption: x(" << k << ", " << j << ", "
                        << makespan << ", " << make_xvar_id(k, j, makespan) << ")" << std::endl;
            } else {
                assumptions.push(Glucose::mkLit(make_xvar_id(k, j, makespan), true) );
                
                if(_verbose > 1)
                    std::cout << "Creating assumption: ~x(" << k << ", " << j << ", "
                        << makespan << ", " << make_xvar_id(k, j, makespan) << ")" << std::endl;
            }
            // Missing the empty vertices!
        }
    }
}

Solution SimplifiedEncoder::get_solution(int makespan) {
    Solution sol;
    for(int i = 0; i < makespan; ++i) {
        for(int k = 0; k < _instance.n_agents(); ++k) {
            for(int j = 0; j < _instance.n_vertices(); ++j) {
                if(_solver->modelValue(make_xvar_id(k, j, i)) == l_True)
                    sol.add(i, k, j);
            }
        }
    }
    if(!sol.check(_instance))
        std::cerr << "Something went wrong with solution." << std::endl;

    return sol;
}

inline Glucose::Var SimplifiedEncoder::make_xvar_id(int agent_id, int vertex_id, int timestep) {
    return timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
        + vertex_id * (_instance.n_agents() + 1)
        + agent_id;
}

inline Glucose::Var SimplifiedEncoder::make_evar_id(int vertex_id, int timestep) {
    return timestep * _instance.n_vertices() * (_instance.n_agents()+1)
        + vertex_id * (_instance.n_agents() + 1)
        + _instance.n_agents();
}

inline int SimplifiedEncoder::get_agent_id_x(int var_id) {
    return var_id % (_instance.n_vertices() * (_instance.n_agents()+1) );
}

inline int SimplifiedEncoder::get_vertex_id_x(int var_id) {
    int intdiv = std::floor( var_id/ (_instance.n_agents() + 1) );
    return intdiv % _instance.n_vertices();
}

inline int SimplifiedEncoder::get_timestep_x(int var_id) {
    return std::floor(var_id / ( _instance.n_vertices() * (_instance.n_agents()+1) ) );
}