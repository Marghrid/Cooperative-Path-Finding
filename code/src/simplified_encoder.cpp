#include "simplified_encoder.h"
//#include "cnf.h"
#include "simp/SimpSolver.h"
#include <cmath>

Direct_encoder::Direct_encoder(Instance inst, int makespan, std::string cnf_file)
	: _instance(inst) {
		_makespan = makespan;
		_cnf_file = cnf_file;
		_cnf_file_stream.open(_cnf_file);
	}

int Direct_encoder::make_xvar_id(int agent_id, int vertex_id, int timestep) {
	return agent_id * _instance.n_vertices() * _makespan
		+ vertex_id * _makespan
		+ _makespan;
}

int Direct_encoder::make_evar_id(int vertex_id, int timestep) {
	return _instance.n_agents() * _instance.n_vertices() * _makespan
		+ vertex_id * _makespan
		+ _makespan;
}

int Direct_encoder::get_agent_id_x(int var_id) {
	return std::floor(var_id/(_instance.n_vertices()*_makespan));
}

int Direct_encoder::get_vertex_id_x(int var_id) {
	int argvar = std::floor(var_id/_makespan);
	return argvar % _instance.n_vertices();
}

int Direct_encoder::get_timestep_x(int var_id) {
	return var_id%_makespan;
}

void Direct_encoder::convert() {
	// DIRECT:
	if(!_instance.check()) {
		//exception
		std::cerr << "The instance wasn't ready to convert" << std::endl;
	}

	Minisat::SimpSolver solver;

	// Create variables: In Minisat/Glucose variables are but integers.

	// Create a variable for each agent, vertex and time step.
	for(int i = 0; i < _instance.n_agents(); ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++i) {
			for(int k = 0; k < _makespan; ++k) {
				std::cout << "var x for agent " << i
					<< ", vertex " << j
					<< ", and timestep " << i
					<< "has id " << make_xvar_id(k, j, i) << std::endl;
			}
		}
	}

	// at most one agent is placed in each vertex at each time step

	for(int i = 0; i < _makespan; ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++j){
			for(int h = 0; h < _instance.n_agents(); ++h) {
				for(int k = 0; k < h; ++k) {
					Minisat::Lit l1 = Minisat::mkLit(make_xvar_id(k, j, i), true);
					Minisat::Lit l2 = Minisat::mkLit(make_xvar_id(h, j, i), true);
					solver.addClause(l1, l2);
				}
			}
		}
	}


	// an agent moves along an edge, or not at all:
	
	for(int i = 0; i < _makespan - 1; ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++j){
			for(int k = 0; k < _instance.n_agents(); ++k) {
				Minisat::vec<Minisat::Lit> lit_vec;

				lit_vec.push( Minisat::mkLit(make_xvar_id(k, j, i),   true) );
				lit_vec.push( Minisat::mkLit(make_xvar_id(k, j, i+1), false) );

				for(auto &v: _instance.get_neighbours(j)) {
					lit_vec.push( Minisat::mkLit(make_xvar_id(k, v.id(), i+1)) );
				}
				solver.addClause(lit_vec);
			}
		}
	}
	


	// the target vertex is vacant before the move,
	// and the source vertex will be vacant after it

	for(int i = 0; i < _makespan - 1; ++i) {
		for(int k = 0; k < _instance.n_agents(); ++k) {
			for(auto &e: _instance.bidirectional_edges()) {
				Minisat::vec<Minisat::Lit> lit_vec;
				lit_vec.push( Minisat::mkLit(make_xvar_id(k, e.start().id(), i), true) );
				lit_vec.push( Minisat::mkLit(make_xvar_id(k, e.end().id(), i+1), true) );
				lit_vec.push( Minisat::mkLit(make_evar_id(e.start().id(), i), false) );
				lit_vec.push( Minisat::mkLit(make_evar_id(e.end().id(), i+1), false));
				solver.addClause(lit_vec);
			}
		}
	}

	for(int i = 0; i < _makespan - 1; ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++j) {
			Minisat::Lit l1 = Minisat::mkLit(make_evar_id(j, i), true);
			for(int k = 0; k < _instance.n_agents(); ++k) {
				solver.addClause(l1, Minisat::mkLit(make_xvar_id(k, j, i)));
			}

		}
	}

}
