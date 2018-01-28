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

int Direct_encoder::make_var_id(int agent_id, int vertex_id, int timestep) {
	return agent_id * _instance.n_vertices() * _makespan
		+ vertex_id * _makespan
		+ _makespan;
}

int Direct_encoder::get_agent_id(int var_id) {
	return std::floor(var_id/(_instance.n_vertices()*_makespan));
}

int Direct_encoder::get_vertex_id(int var_id) {
	int argvar = std::floor(var_id/_makespan);
	return argvar % _instance.n_vertices();
}

int Direct_encoder::get_timestep(int var_id) {
	return var_id%_makespan;
}

void Direct_encoder::convert() {
	// DIRECT:
	if(!_instance.check()) {
		//exception
		std::cerr << "The instance wasn't ready to convert" << std::endl;
	}
 /*
	std::vector<Variable> variables;
	variables.resize(_instance.n_agents() * _instance.n_vertices() * _makespan);

	// Create a variable for each agent, vertex and time step.
	for(int i = 0; i < _instance.n_agents(); ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++i) {
			for(int k = 0; k < _makespan; ++k) {
				Variable v(make_var_id(k, j, i));
				variables.at(v.id()) = v;
			}
		}
	}

	// at most one agent is placed in each vertex at each time step

	for(int i = 0; i < _makespan; ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++j){
			for(int h = 0; h < _instance.n_agents(); ++h) {
				for(int k = 0; k < _instance.n_agents(); ++k) {
					Clause c;
					c.add_literal(variables.at(make_var_id(k, j, i)), false);
					c.add_literal(variables.at(make_var_id(h, j, i)), false);
					_formula.add_clause(c);
				}
			}
		}
	}
*/
	Minisat::SimpSolver solver;

	// Create variables: In Minisat/Glucose variables are but integers.

	// Create a variable for each agent, vertex and time step.
	for(int i = 0; i < _instance.n_agents(); ++i) {
		for(int j = 0; j < _instance.n_vertices(); ++i) {
			for(int k = 0; k < _makespan; ++k) {
				Variable v = make_var_id(k, j, i);
			}
		}
	}
}
