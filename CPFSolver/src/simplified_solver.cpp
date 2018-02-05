#include "simplified_solver.h"
//#include "cnf.h"
#include "simp/SimpSolver.h"
#include <cmath>

Simplified_solver::Simplified_solver(Instance inst, int max_makespan)
	: _instance(inst) {
		_max_makespan = max_makespan;
		//_cnf_file = cnf_file;
		//_cnf_file_stream.open(_cnf_file);
	}

Glucose::Var Simplified_solver::make_xvar(int agent_id, int vertex_id, int timestep, int eta) {
	return agent_id * _instance.n_vertices() * eta
		+ vertex_id * eta
		+ timestep;
}

Glucose::Var Simplified_solver::make_evar(int vertex_id, int timestep, int eta) {
	return _instance.n_agents() * _instance.n_vertices() * eta
		+ vertex_id * eta
		+ timestep;
}

int Simplified_solver::get_agent_id_x(int var_id, int eta) {
	return std::floor(var_id/(_instance.n_vertices()*eta));
}

int Simplified_solver::get_vertex_id_x(int var_id, int eta) {
	int argvar = std::floor(var_id/eta);
	return argvar % _instance.n_vertices();
}

int Simplified_solver::get_timestep_x(int var_id, int eta) {
	return var_id%eta;
}

bool Simplified_solver::solve_for_makespan(Glucose::SimpSolver solver, int eta) {
	int mu  = _instance.n_agents();
	int n   = _instance.n_vertices();

	std::cout << "mu = " << mu
		<< ", n = " << n 
		<< ", eta = " << eta
		<< std::endl;

	// Create a variable for each agent, vertex and time step.
	std::cout << "Create a variable for each agent, vertex and time step..."
		<< std::endl;
	for(int j = 0; j < n; ++j) {
		for(int i = 0; i < eta; ++i) {
			for(int k = 0; k < mu; ++k) {
/*				std::cout << "var x for agent " << k
					<< ", vertex " << j
					<< ", and timestep " << i
					<< " has id " << make_xvar(k, j, i, eta) << std::endl;
					*/
					while (make_xvar(k, j, i, eta) >= solver.nVars()) solver.newVar();
			}

/*			std::cout << "var epsilon for vertex " << j
					<< ", and timestep " << i
					<< " has id " << make_evar(j, i, eta) << std::endl;
					*/
					while (make_evar(j, i, eta) >= solver.nVars()) solver.newVar();
		}
	}

	// At most one agent is placed in each vertex at each time step
	std::cout << "At most one agent is placed in each vertex at each time step..."
		<< std::endl;
	for(int i = 0; i < eta; ++i) {
		for(int j = 0; j < n; ++j){
			for(int h = 0; h < mu; ++h) {
				for(int k = 0; k < h; ++k) {
					Glucose::Lit l1 = Glucose::mkLit(make_xvar(k, j, i, eta), true);
					Glucose::Lit l2 = Glucose::mkLit(make_xvar(h, j, i, eta), true);
					solver.addClause(l1, l2);
				}
			}
		}
	}

	// An agent moves along an edge, or not at all:
	std::cout << "An agent moves along an edge, or not at all..."
		<< std::endl;
	for(int i = 0; i < eta-1; ++i) {
		for(int j = 0; j < n; ++j){
			for(int k = 0; k < mu; ++k) {
				Glucose::vec<Glucose::Lit> lit_vec;
				lit_vec.push( Glucose::mkLit(make_xvar(k, j, i, eta),   true) );
				lit_vec.push( Glucose::mkLit(make_xvar(k, j, i+1, eta), false) );

				for(auto &v: _instance.get_neighbours(j)) {
					lit_vec.push( Glucose::mkLit(make_xvar(k, v.id(), i+1, eta)) );
				}

				solver.addClause(lit_vec);
			}
		}
	}
	

	// The target vertex is vacant before the move,
	// and the source vertex will be vacant after it
	std::cout << "The target vertex is vacant before the move, \n"
		<< "and the source vertex will be vacant after it..."
		<< std::endl;
	for(int i = 0; i < eta - 1; ++i) {
		for(int k = 0; k < mu; ++k) {
			for(auto &e: _instance.bidirectional_edges()) {
				Glucose::vec<Glucose::Lit> lit_vec;
				lit_vec.push( Glucose::mkLit(make_xvar(k, e.start().id(), i, eta), true) );
				lit_vec.push( Glucose::mkLit(make_xvar(k, e.end().id(), i+1, eta), true) );
				lit_vec.push( Glucose::mkLit(make_evar(e.end().id(),      i, eta), false) );
				lit_vec.push( Glucose::mkLit(make_evar(e.start().id(),  i+1, eta), false) );
				solver.addClause(lit_vec);
			}
		}
	}

	// relation between variables
	std::cout << "Establishing relation between epsilon and X variables..." << std::endl;
	for(int i = 0; i < eta; ++i) {
		for(int j = 0; j < n; ++j) {
			Glucose::Lit l1 = Glucose::mkLit(make_evar(j, i, eta), true);
			for(int k = 0; k < _instance.n_agents(); ++k) {
				solver.addClause(l1, Glucose::mkLit(make_xvar(k, j, i, eta), true));
			}

		}
	}


	// Initial and goal arragements:
	std::cout << "Initial and goal arragements..." << std::endl;
	for(int k = 0; k < mu; ++k) {
		for(int j = 0; j < n; ++j) {
			if(j == _instance.agent(k).initial_position().id()) {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, 0, eta)));
			} else {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, 0, eta), true));
			}

			if(j == _instance.agent(k).goal_position().id()) {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, eta-1, eta) ) );
			} else {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, eta-1, eta), true) );
			}
		}
	}

	std::cout << "Start solving..." << std::endl;
	solver.simplify();
	bool satisfiable = solver.solve();

	if(!satisfiable) return false;

	// Show results:
	for(int i = 0; i < eta; ++i) {
		std::cout << "On timestep " << i << ":" << std::endl;
		for(int k = 0; k < mu; ++k) {
			for(int j = 0; j < n; ++j) {
				if(solver.modelValue(make_xvar(k, j, i, eta)) == l_True)
					std::cout << "\tAgent " << k
						<< " is on vertex " << j
						<< std::endl;
			}
		}
	}

	return satisfiable;

}

bool Simplified_solver::solve() {
	// DIRECT:
	if(!_instance.check()) {
		//exception
		std::cerr << "The instance wasn't ready to convert" << std::endl;
	}

	Glucose::SimpSolver solver;

	bool solved = false;

	int eta = 1;
	for(eta = 1; eta < _max_makespan && solved != true; ++eta) {
		std::cout << "trying eta " << eta << std::endl;
		solved = solve_for_makespan(solver, eta);
	}
	
	if(solved) {
		std::cout << "Solved for makespan = " << --eta << std::endl;
	}
	return solved;
	// Create variables: In Minisat/Glucose variables are but integers.

	
}
