#ifndef _DIRECT_ENCODER_
#define	_DIRECT_ENCODER_

#include <iostream>
#include <fstream>
#include <string>

#include "cnf.h"
#include "instance.h"


class Direct_encoder {
private:
	// std::ostream* fp = &cout
	Instance _instance;
	Formula _formula;
	int _makespan;
	std::ofstream _cnf_file_stream;
	std::string   _cnf_file;

	int make_var_id(int agent_id, int vertex_id, int timestep);

	int get_agent_id(int var_id);

    int get_vertex_id(int var_id);

	int get_timestep(int var_id);

public:
	Direct_encoder(Instance inst, int makespan, std::string cnf_file);

	void convert();

	// std::ostream* fp = &cout

//	Direct_encoder(std::string instance_file) {
//		_instance_file = instance_file;
//		initialize(false);
//	}

/*	Direct_encoder(std::string instance_file, std::string cnf_file) {
		_instance_file = instance_file;
		_cnf_file      = cnf_file;
		initialize(true);
	}
*/
/*	Direct_encoder(Instance &inst):
		_instance(inst) {
//		initialize(false);
	}
*/
};

#endif