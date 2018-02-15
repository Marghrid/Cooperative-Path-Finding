#include "Instance.h"
#include "Graph.h"
#include "Parser.h"
#include "CPFSolver.h"

#include <iostream>
#include <cstdlib>
#include <ctime>

void print_usage_instructions() {
    std::cout << "-------->>>  CPFSolver  <<<--------" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Different ways to use:" << std::endl;
    std::cout << "bin/CPFSolver <input-file>" << std::endl;
    std::cout << "  <input-file> is a path to a CPF file." << std::endl;
    std::cout << "  Will solve <input-file> with maximum makespan equal to the number of" << std::endl;
    std::cout << "  vertices. The solution and statistics will be presented on standard output." << std::endl;
    std::cout << "" << std::endl;
    std::cout << "bin/CPFSolver <input-file> <max-makespan>" << std::endl;
    std::cout << "  <input-file> is a path to a CPF file." << std::endl;
    std::cout << "  <max-makespan> is an integer." << std::endl;
    std::cout << "  Will solve <input-file> with maximum makespan <max-makespan>." << std::endl;
    std::cout << "" << std::endl;
    std::cout << "bin/CPFSolver -i|-input-file   <input-file>"   << std::endl;
    std::cout << "              -o|-output-file  <output-file>"  << std::endl;
    std::cout << "              -s|-stats-file   <stats-file>"   << std::endl;
    std::cout << "            -max|-max-makespan <max-makespan>" << std::endl;
    std::cout << "              -e|-encoding     <encoding>"     << std::endl;
    std::cout << "                 -search       <search>"       << std::endl;
    std::cout << "              -v|-verbose      <verbosity>"    << std::endl;

    std::cout << "  <input-file>:   the path to an existing CPF file." << std::endl;
    std::cout << "  <output-file>:  the path to where the solution file will be created." << std::endl;
    std::cout << "  <stats-file>:   the path to where the statistics file will be created." << std::endl;
    std::cout << "  <max-makespan>: an integer" << std::endl;
    std::cout << "  <encoding>:     simplified|other_encoding_that_i_will_implement_soon" << std::endl;
    std::cout << "  <search>:       UNSAT-SAT|SAT-UNSAT|binary" << std::endl;
    std::cout << "  <verbosity>:    0|1|2" << std::endl;
    std::cout << "  Will solve <input-file> with maximum makespan <max-makespan>" << std::endl;
    std::cout << "  with <verbosity> verbosity level. The solution will be saved" << std::endl;
    std::cout << "  at <output-file> and the SAT solver statistics at <stats-file>." << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Default values:" << std::endl;
    std::cout << "  <input-file>: ../instances/grid_4x4_a6_o0.1_s616.cpf" << std::endl;
    std::cout << "  <max-makespan>: the number of vertices of the instance." << std::endl;
    std::cout << "  <encoding>: simplified." << std::endl;
    std::cout << "  <search>: UNSAT-SAT." << std::endl;
    std::cout << "  <verbose>: 0" << std::endl;
    std::cout << "" << std::endl;
}

void print_stats(std::ostream &os, Instance &instance, CPFSolver &solver,
	std::string encoding, std::string search,
	double parsing_cpu, double solving_cpu) {
	//ofs << "Wall clock time:" << std::endl;
    //ofs << "  Parsing: " << parsing_wall << std::endl;
    //ofs << "  Solving: " << solving_wall << std::endl;
	os << "Instance size:" << std::endl;
	os << "  agents: "   << instance.n_agents() << std::endl;
	os << "  vertices: " << instance.n_vertices() << std::endl;
	os << "  edges: "    << instance.n_edges() << std::endl;
	os << "" << std::endl;

	os << "Solver settings:" << std::endl;
	os << "  encoding: " << encoding << std::endl;
	os << "  search: "   << search << std::endl;
	os << "" << std::endl;

    os << "CPU time:"   << std::endl;
    os << "  Parsing: " << parsing_cpu << " s" << std::endl;
	os << "  Solving: " << solving_cpu << " s" << std::endl;
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

    Solution solution = solver.solve();

	//solving_wall = std::get_wall_time() - wall0;
	solving_cpu  = (std::clock() - cpu0) / CLOCKS_PER_SEC;

    if(solution.is_empty()) {
        std::cout << "No solution found." << std::endl;
        return 0;
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

	// Unfortunately, the solver statistics cannot be written to a file, since
	// the glucose function writes them directly to standard output.
	// NOTE: Redirect standard output? Use dup().

    if(stats_file != "") {
    	std::ofstream ofs;
    	ofs.open(stats_file);
    	print_stats(ofs, instance, solver, encoding, search, parsing_cpu, solving_cpu);
    	ofs.close();
    }

    if(verbose > 0 || stats_file == "") {
    	print_stats(std::cout, instance, solver, encoding, search, parsing_cpu, solving_cpu);
    }
    return 0;
}
