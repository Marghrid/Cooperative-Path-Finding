#ifndef __SIMPLIFIED_SOLVER__
#define __SIMPLIFIED_SOLVER__

#include <iostream>
#include <fstream>
#include <string>

#include "Instance.h"
#include "simp/SimpSolver.h"

class Simplified_solver {
private:
    // std::ostream* fp = &cout
    Instance _instance;
    int _max_makespan;
    int _created_vars_makespan;
    int _created_clauses_makespan;
    Glucose::SimpSolver _solver;
    int _verbose;
    //std::ofstream _cnf_file_stream;
    //std::string   _cnf_file;


    int get_initial_makespan() { return 0; }

    Glucose::Var make_xvar_id(int agent_id, int vertex_id, int timestep);

    Glucose::Var make_evar_id(int vertex_id, int timestep);

    int get_agent_id_x(int var_id);

    int get_vertex_id_x(int var_id);

    int get_timestep_x(int var_id);

    bool solve_for_makespan(int eta);

    void create_vars(int current_makespan);

public:
    Simplified_solver(Instance inst, int makespan, int verbose = 0);

    bool solve();

    // std::ostream* fp = &cout

//  Simplified_solver(std::string instance_file) {
//      _instance_file = instance_file;
//      initialize(false);
//  }

/*  Simplified_solver(std::string instance_file, std::string cnf_file) {
        _instance_file = instance_file;
        _cnf_file      = cnf_file;
        initialize(true);
    }
*/
/*  Simplified_solver(Instance &inst):
        _instance(inst) {
//      initialize(false);
    }
*/
};

#endif