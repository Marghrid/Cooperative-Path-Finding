#include "simplified_solver.h"
//#include "cnf.h"
#include "simp/SimpSolver.h"
#include <cmath>

Simplified_solver::Simplified_solver(Instance inst, int max_makespan, int verbose)
: _instance(inst) {
	_max_makespan = max_makespan;
	_created_vars_makespan = -1;
	_created_clauses_makespan = -1;
	_verbose = verbose;
}

Glucose::Var Simplified_solver::make_xvar_id(int agent_id, int vertex_id, int timestep) {
	return timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
	+ vertex_id * (_instance.n_agents() + 1)
	+ agent_id;
}

Glucose::Var Simplified_solver::make_evar_id(int vertex_id, int timestep) {
	return timestep * _instance.n_vertices() * (_instance.n_agents() + 1)
	+ vertex_id * (_instance.n_agents() + 1)
	+ _instance.n_agents();
}

int Simplified_solver::get_agent_id_x(int var_id, int eta) {
	return -1;//std::floor(var_id/(_instance.n_vertices()*eta));
}

int Simplified_solver::get_vertex_id_x(int var_id, int eta) {
	//int argvar = std::floor(var_id/eta);
	return -1;//argvar % _instance.n_vertices();
}

int Simplified_solver::get_timestep_x(int var_id, int eta) {
	return -1;//var_id%eta;
}

void Simplified_solver::create_vars(int current_makespan) {

	if(_verbose > 0)
		std::cout << "creating vars for makespan " << current_makespan << std::endl;

	while(_created_vars_makespan < current_makespan) {
		++_created_vars_makespan;
		for(int j = 0; j < _instance.n_vertices(); ++j) {
			for(int k = 0; k < _instance.n_agents(); ++k) {
				if(_verbose > 1) 
					std::cout << "var x for agent " << k
					<< ", vertex " << j
					<< ", and timestep " << _created_vars_makespan
					<< " has id " << make_xvar_id(k, j, _created_vars_makespan);

				while (make_xvar_id(k, j, _created_vars_makespan) >= _solver.nVars())
					_solver.newVar();

				if(_verbose > 1) std::cout << std::endl;
			}
			if(_verbose > 1) 
				std::cout << "var epsilon for vertex " << j
					<< ", and timestep " << _created_vars_makespan
					<< " has id " << make_evar_id(j, _created_vars_makespan) << std::endl;

			while (make_evar_id(j, _created_vars_makespan) >= _solver.nVars())
				_solver.newVar();
		}
	}
}


bool Simplified_solver::solve_for_makespan(int eta) {
	int mu  = _instance.n_agents();
	int n   = _instance.n_vertices();

	if(_verbose > 0)
		std::cout << "Solving for makespan eta = " << eta << ":" << std::endl;

	while(_created_clauses_makespan < eta) {
		int i = ++_created_clauses_makespan;
		// At most one agent is placed in each vertex at each time step
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

					_solver.addClause(l1, l2);
				}
			}
		}

		
		if(i > 0) {

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
					_solver.addClause(lit_vec);
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
					_solver.addClause(lit_vec);
				}
			}
		}

		// relation between variables
		if(_verbose > 0)
			std::cout << "Establishing relation between epsilon and X variables..." << std::endl;
		for(int j = 0; j < n; ++j) {
			Glucose::Lit l1 = Glucose::mkLit(make_evar_id(j, i), true);
			for(int k = 0; k < _instance.n_agents(); ++k) {
				_solver.addClause(l1, Glucose::mkLit(make_xvar_id(k, j, i), true));
				if(_verbose > 1)
					std::cout << "adding clause: ~e(" << j << ", " << i << ", "
						<< make_evar_id(j, i) << ") V ~x("
						<< k << ", " << j << ", " << i << ", "
						<< make_xvar_id(k, j, i) << ")" << std::endl;
			}
		}
	}

	// Goal arragements:
	if(_verbose > 0)
		std::cout << "Goal arragements..." << std::endl;
	
	Glucose::vec<Glucose::Lit> assumptions;
	for(int k = 0; k < mu; ++k) {
		for(int j = 0; j < n; ++j) {
			if(j == _instance.agent(k).goal_position().id()) {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, eta), false) );
				
				if(_verbose > 1)
					std::cout << "Creating assumption: x(" << k << ", " << j << ", "
						<< eta << ", " << make_xvar_id(k, j, eta) << ")" << std::endl;
			} else {
				assumptions.push(Glucose::mkLit(make_xvar_id(k, j, eta), true) );
				
				if(_verbose > 1)
					std::cout << "Creating assumption: ~x(" << k << ", " << j << ", "
						<< eta << ", " << make_xvar_id(k, j, eta) << ")" << std::endl;
			}
			// Missing the empty vertices!
		}
	}

	// The (false, true) arguments on solve() prenvent the solver from
	//  performing optimizations which could result in variable elimination.
	//  (view lines 172-187 on include/glucose/simp/SimpSolver.cc)
	// This is a problem because I use variables from one iteration to another,
	//  resulting in a call of solve() between their creation, and the creation
	//  of clauses containing them.
	// I should find out how much impact these optimizations I'm preventing
	//  actually have.
	bool satisfiable = _solver.solve(assumptions, false, true);
	
	if(!satisfiable) {
		std::cout << "No solution for makespan " << eta << std::endl;
		return false;
	}

	// Show results:
	std::cout << "Solution found for makespan " << eta << std::endl;
	for(int i = 0; i < eta; ++i) {
		std::cout << "On timestep " << i << ":" << std::endl;
		for(int k = 0; k < mu; ++k) {
			for(int j = 0; j < n; ++j) {
				if(_solver.modelValue(make_xvar_id(k, j, i)) == l_True)
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

	bool solved = false;

	int current_makespan = get_initial_makespan();

	create_vars(current_makespan);

	// Initial arragement:
	if(_verbose > 0)
		std::cout << "Initial arragement..." << std::endl;
	for(int j = 0; j < _instance.n_vertices(); ++j) {
		for(int k = 0; k < _instance.n_agents(); ++k) {
			if(j == _instance.agent(k).initial_position().id()) {
				if(_verbose > 1)
					std::cout << "adding clause: x(" << k << ", " << j << ", "
						<< 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
				_solver.addClause(Glucose::mkLit(make_xvar_id(k, j, 0)));
			} else {
				if(_verbose > 1)
					std::cout << "adding clause: ~x(" << k << ", " << j << ", "
						<< 0 << ", " << make_xvar_id(k, j, 0) << ")" << std::endl;
				_solver.addClause(Glucose::mkLit(make_xvar_id(k, j, 0), true));
			}
		}

		if(_instance.starts_empty(j)) {
			if(_verbose > 1)
				std::cout << "adding clause: e(" << j << ", "
					<< 0 << ", " << make_evar_id(j, 0) << ")" << std::endl;
			_solver.addClause(Glucose::mkLit(make_evar_id(j, 0)));
		}
	}

	for( ; current_makespan < _max_makespan && solved != true; ++current_makespan) {
		if(_created_vars_makespan <= current_makespan)
			create_vars(current_makespan);

		std::cout << "Trying eta = " << current_makespan << std::endl;
		solved = solve_for_makespan(current_makespan);
	}
	
	if(solved) {
		std::cout << "Solved for makespan = " << --current_makespan << std::endl;
	}
	return solved;
	// Create variables: In Minisat/Glucose variables are but integers.

	
}
