#include "simplified_solver.h"
//#include "cnf.h"
#include "simp/SimpSolver.h"
#include <cmath>

Simplified_solver::Simplified_solver(Instance inst, int makespan)
	: _instance(inst) {
		_makespan = makespan;
		//_cnf_file = cnf_file;
		//_cnf_file_stream.open(_cnf_file);
	}

Glucose::Var Simplified_solver::make_xvar(int agent_id, int vertex_id, int timestep) {
	return agent_id * _instance.n_vertices() * _makespan
		+ vertex_id * _makespan
		+ timestep;
}

Glucose::Var Simplified_solver::make_evar(int vertex_id, int timestep) {
	return _instance.n_agents() * _instance.n_vertices() * _makespan
		+ vertex_id * _makespan
		+ timestep;
}

int Simplified_solver::get_agent_id_x(int var_id) {
	return std::floor(var_id/(_instance.n_vertices()*_makespan));
}

int Simplified_solver::get_vertex_id_x(int var_id) {
	int argvar = std::floor(var_id/_makespan);
	return argvar % _instance.n_vertices();
}

int Simplified_solver::get_timestep_x(int var_id) {
	return var_id%_makespan;
}

void Simplified_solver::convert() {
	// DIRECT:
	if(!_instance.check()) {
		//exception
		std::cerr << "The instance wasn't ready to convert" << std::endl;
	}

	Glucose::SimpSolver solver;

	// Create variables: In Minisat/Glucose variables are but integers.

	int mu  = _instance.n_agents();
	int n   = _instance.n_vertices();
	int eta = _makespan;

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
				std::cout << "var x for agent " << k
					<< ", vertex " << j
					<< ", and timestep " << i
					<< " has id " << make_xvar(k, j, i) << std::endl;
					while (make_xvar(k, j, i) >= solver.nVars()) solver.newVar();
			}

			std::cout << "var epsilon for vertex " << j
					<< ", and timestep " << i
					<< " has id " << make_evar(j, i) << std::endl;
					while (make_evar(j, i) >= solver.nVars()) solver.newVar();
		}
	}

	// At most one agent is placed in each vertex at each time step
	std::cout << "At most one agent is placed in each vertex at each time step..."
		<< std::endl;
	for(int i = 0; i < eta; ++i) {
		for(int j = 0; j < n; ++j){
			for(int h = 0; h < mu; ++h) {
				for(int k = 0; k < h; ++k) {
					Glucose::Lit l1 = Glucose::mkLit(make_xvar(k, j, i), true);
					Glucose::Lit l2 = Glucose::mkLit(make_xvar(h, j, i), true);
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
				lit_vec.push( Glucose::mkLit(make_xvar(k, j, i),   true) );
				lit_vec.push( Glucose::mkLit(make_xvar(k, j, i+1), false) );

				for(auto &v: _instance.get_neighbours(j)) {
					lit_vec.push( Glucose::mkLit(make_xvar(k, v.id(), i+1)) );
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
				lit_vec.push( Glucose::mkLit(make_xvar(k, e.start().id(), i), true) );
				lit_vec.push( Glucose::mkLit(make_xvar(k, e.end().id(), i+1), true) );
				lit_vec.push( Glucose::mkLit(make_evar(e.end().id(),      i), false) );
				lit_vec.push( Glucose::mkLit(make_evar(e.start().id(),  i+1), false) );
				solver.addClause(lit_vec);
			}
		}
	}

	// relation between variables
	std::cout << "Establishing relation between epsilon and X variables..." << std::endl;
	for(int i = 0; i < eta; ++i) {
		for(int j = 0; j < n; ++j) {
			Glucose::Lit l1 = Glucose::mkLit(make_evar(j, i), true);
			for(int k = 0; k < _instance.n_agents(); ++k) {
				solver.addClause(l1, Glucose::mkLit(make_xvar(k, j, i), true));
			}

		}
	}


	// Initial and goal arragements:
	std::cout << "Initial and goal arragements..." << std::endl;
	for(int k = 0; k < mu; ++k) {
		for(int j = 0; j < n; ++j) {
			if(j == _instance.agent(k).initial_position().id()) {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, 0)));
				std::cout << "Agent " << k << " starts at " << j << std::endl;
			} else {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, 0), true));
				std::cout << "Agent " << k << " does not start at " << j << std::endl;
			}

			if(j == _instance.agent(k).goal_position().id()) {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, _makespan-1) ) );
			} else {
				solver.addClause(Glucose::mkLit(make_xvar(k, j, _makespan-1), true) );
			}
		}
	}

	std::cout << "Start solving..." << std::endl;
	solver.simplify();
	bool satisfiable = solver.solve();

	if(!satisfiable) return;

	// Show results:
	for(int i = 0; i < eta; ++i) {
		std::cout << "On timestep " << i << ":" << std::endl;
		for(int k = 0; k < mu; ++k) {
			for(int j = 0; j < n; ++j) {
				if(solver.modelValue(make_xvar(k, j, i)) == l_True)
					std::cout << "\tAgent " << k
						<< " is on vertex " << j
						<< std::endl;
			}
		}
	}

}
