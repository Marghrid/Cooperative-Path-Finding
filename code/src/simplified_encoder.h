#ifndef __SIMPLIFIED_SOLVER__
#define	__SIMPLIFIED_SOLVER__

#include <iostream>
#include <fstream>
#include <string>

#include "instance.h"


class Simplified_solver {
private:
	// std::ostream* fp = &cout
	Instance _instance;
	int _makespan;
	//std::ofstream _cnf_file_stream;
	//std::string   _cnf_file;

	int make_xvar_id(int agent_id, int vertex_id, int timestep);

	int make_evar_id(int vertex_id, int timestep);

	int get_agent_id_x(int var_id);

    int get_vertex_id_x(int var_id);

	int get_timestep_x(int var_id);

public:
	Simplified_solver(Instance inst, int makespan);

	void convert();

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