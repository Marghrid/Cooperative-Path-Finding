#include "Instance.h"
#include "Graph.h"
#include "Parser.h"
#include "CPFSolver.h"

#include <iostream>
#include <cstdlib>
#include <ctime>

void print_usage_instructions() {
    std::cout << "-------->>>  CPFSolver  <<<--------" << "\n";
    std::cout << "" << "\n";
    std::cout << "Different ways to use:" << "\n";
    std::cout << "bin/CPFSolver <input-file>" << "\n";
    std::cout << "  <input-file> is a path to a CPF file." << "\n";
    std::cout << "  Will solve <input-file> with maximum makespan equal to the number of" << "\n";
    std::cout << "  vertices. The solution and statistics will be presented on standard output." << "\n";
    std::cout << "" << "\n";
    std::cout << "bin/CPFSolver <input-file> <max-makespan>" << "\n";
    std::cout << "  <input-file> is a path to a CPF file." << "\n";
    std::cout << "  <max-makespan> is an integer." << "\n";
    std::cout << "  Will solve <input-file> with maximum makespan <max-makespan>." << "\n";
    std::cout << "" << "\n";
    std::cout << "bin/CPFSolver -i|-input-file   <input-file>"   << "\n";
    std::cout << "              -o|-output-file  <output-file>"  << "\n";
    std::cout << "              -s|-stats-file   <stats-file>"   << "\n";
    std::cout << "            -max|-max-makespan <max-makespan>" << "\n";
    std::cout << "              -e|-encoding     <encoding>"     << "\n";
    std::cout << "                 -search       <search>"       << "\n";
    std::cout << "              -v|-verbose      <verbosity>"    << "\n";

    std::cout << "  <input-file>:   the path to an existing CPF file." << "\n";
    std::cout << "  <output-file>:  the path to where the solution file will be created." << "\n";
    std::cout << "  <stats-file>:   the path to where the statistics file will be created." << "\n";
    std::cout << "  <max-makespan>: an integer" << "\n";
    std::cout << "  <encoding>:     simplified|other_encoding_that_i_will_implement_soon" << "\n";
    std::cout << "  <search>:       UNSAT-SAT|SAT-UNSAT|binary" << "\n";
    std::cout << "  <verbosity>:    0|1|2" << "\n";
    std::cout << "  Will solve <input-file> with maximum makespan <max-makespan>" << "\n";
    std::cout << "  with <verbosity> verbosity level. The solution will be saved" << "\n";
    std::cout << "  at <output-file> and the SAT solver statistics at <stats-file>." << "\n";
    std::cout << "" << "\n";
    std::cout << "Default values:" << "\n";
    std::cout << "  <input-file>: ../instances/grid_4x4_a6_o0.1_s616.cpf" << "\n";
    std::cout << "  <max-makespan>: the number of vertices of the instance." << "\n";
    std::cout << "  <encoding>: simplified." << "\n";
    std::cout << "  <search>: UNSAT-SAT." << "\n";
    std::cout << "  <verbose>: 0" << "\n";
    std::cout << "" << std::endl;
}

void print_stats(std::ostream &os, Instance &instance, CPFSolver &solver,
	std::string encoding, std::string search,
	double parsing_cpu, double solving_cpu) {
	//ofs << "Wall clock time:" << "\n";
    //ofs << "  Parsing: " << parsing_wall << "\n";
    //ofs << "  Solving: " << solving_wall << "\n";
	os << "Instance size:" << "\n";
	os << "  agents: "   << instance.n_agents() << "\n";
	os << "  vertices: " << instance.n_vertices() << "\n";
	os << "  edges: "    << instance.n_edges() << "\n";
	os << "" << "\n";

	os << "Solver settings:" << "\n";
	os << "  encoding: " << encoding << "\n";
	os << "  search: "   << search << "\n";
	os << "" << "\n";

    os << "CPU time:"   << "\n";
    os << "  Parsing: " << parsing_cpu << " s" << "\n";
	os << "  Solving: " << solving_cpu << " s" << "\n";
	os << "" << std::endl;

    solver.write_stats(os);
}

int main(int argc, const char **argv) {
    std::string input_file = "../instances/grid_04x04_a0006_o0.1_s616.cpf";
    std::string output_file = "";
    std::string stats_file = "";
    std::string encoding = "simplified";
    std::string search = "unsat-sat";

    int max_makespan = -1;
    int verbose = 0;

    bool out_of_memory = false;

    //double wall0;
    double cpu0;

    //double parsing_wall;
	double parsing_cpu;

	//double solving_wall;
	double solving_cpu;

    if(argc == 1) {
        print_usage_instructions();
        std::cout << "solving for default file: grid_4x4_a6_o0.1_s616.cpf" << std::endl;
    }
    else if(argc == 2) {
        input_file = argv[1];
    }
    else if(argc == 3) {
        input_file = argv[1];
        max_makespan = atoi(argv[2]);
    }
    else {
        for(int i = 1; i < argc-1; ++i) {
            std::string arg(argv[i]);

            if(arg == "-i" || arg == "-input") {
                input_file = argv[i+1];
            }
            else if(arg == "-o" || arg == "-output") {
                output_file = argv[i+1];
            }
            else if(arg == "-s" || arg == "-stats") {
                stats_file = argv[i+1];
            }
            else if(arg == "-max" || arg == "-max-makespan") {
                max_makespan = atoi(argv[i+1]);
            }
            else if(arg == "-e" || arg == "-encoding") {
                encoding = argv[i+1];
            }
            else if(arg == "-search") {
                search = argv[i+1];
            }
            else if(arg == "-v" || arg == "-verbose") {
                verbose = atoi(argv[i+1]);
            }
        }
    }


    //wall0 = std::get_wall_time();
	cpu0  = std::clock();

    Parser parser(input_file);

    Instance instance = parser.parse();

    //parsing_wall = std::get_wall_time() - wall0;
    parsing_cpu  = (std::clock() - cpu0) / CLOCKS_PER_SEC; 

    if(verbose > 0)
        std::cout << instance << std::endl;

    if(max_makespan < 0)
        max_makespan = instance.n_vertices() * instance.n_agents();

    std::cout << "Solving instance with encoding " << encoding
    	<< " and search " << search << "." << std::endl;

    //wall0 = std::get_wall_time();
	cpu0  = std::clock();

    CPFSolver solver(instance, encoding, search, max_makespan, verbose);

    Solution solution(instance);

    try {
        solution = solver.solve();
    } catch (std::runtime_error e) {
        std::string error_message(e.what());
        if (error_message.compare("Out of memory.") == 0)    
            out_of_memory = true;
        else
            return -1;
    }

	//solving_wall = std::get_wall_time() - wall0;
	solving_cpu  = (std::clock() - cpu0) / CLOCKS_PER_SEC;

    if (stats_file != "" && out_of_memory) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "Out of memory." << std::endl;
        ofs.close();
    }
    else if ( stats_file != "" && !solution.is_empty() ) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "Solution found." << std::endl;
        print_stats(ofs, instance, solver, encoding, search, parsing_cpu, solving_cpu);
        ofs.close();
    } else if ( stats_file != "" ) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "No solution found." << std::endl;
        print_stats(ofs, instance, solver, encoding, search, parsing_cpu, solving_cpu);
        ofs.close();
    }

    if(output_file != "") {
        std::ofstream ofs;
        ofs.open(output_file);
        ofs << solution << std::endl;
		ofs.close();
    }

    if(verbose > 0 || output_file == "") {
        std::cout << solution << std::endl;
    }

    if(verbose > 0 || stats_file == "") {
    	print_stats(std::cout, instance, solver, encoding, search, parsing_cpu, solving_cpu);
    }
    return 0;
}
