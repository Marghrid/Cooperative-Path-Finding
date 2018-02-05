#ifndef __SIMPLIFIED_SOLVER__
#define	__SIMPLIFIED_SOLVER__

#include <iostream>
#include <fstream>
#include <string>

#include "instance.h"
#include "simp/SimpSolver.h"

class Simplified_solver {
private:
	// std::ostream* fp = &cout
	Instance _instance;
	int _max_makespan;
	//std::ofstream _cnf_file_stream;
	//std::string   _cnf_file;

	Glucose::Var make_xvar(int agent_id, int vertex_id, int timestep, int eta);

	Glucose::Var make_evar(int vertex_id, int timestep, int eta);

	int get_agent_id_x(int var_id, int eta);

    int get_vertex_id_x(int var_id, int eta);

	int get_timestep_x(int var_id, int eta);

	bool solve_for_makespan(Glucose::SimpSolver solver, int eta);

public:
	Simplified_solver(Instance inst, int makespan);

	bool solve();

	// std::ostream* fp = &cout

//	Simplified_solver(std::string instance_file) {
//		_instance_file = instance_file;
//		initialize(false);
//	}

/*	Simplified_solver(std::string instance_file, std::string cnf_file) {
		_instance_file = instance_file;
		_cnf_file      = cnf_file;
		initialize(true);
	}
*/
/*	Simplified_solver(Instance &inst):
		_instance(inst) {
//		initialize(false);
	}
*/
};

#endif