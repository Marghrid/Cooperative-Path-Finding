#include <iostream>
#include <cstdlib>
#include "Instance.h"
#include "Graph.h"
#include "Parser.h"
#include "CPFSolver.h"

void print_usage_instructions() {
    std::cout << "-------->>>  CPFSolver  <<<--------" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Different ways to use:" << std::endl;
    std::cout << "bin/CPFSolver <input-file>" << std::endl;
    std::cout << "\t <input-file> is a path to a CPF file." << std::endl;
    std::cout << "\t Will solve <input-file> with maximum makespan equal to the number of" << std::endl;
    std::cout << "\t vertices. The solution and statistics will be presented on standard output." << std::endl;
    std::cout << "" << std::endl;
    std::cout << "bin/CPFSolver <input-file> <max-makespan>" << std::endl;
    std::cout << "\t <input-file> is a path to a CPF file." << std::endl;
    std::cout << "\t <max-makespan> is an integer." << std::endl;
    std::cout << "\t Will solve <input-file> with maximum makespan <max-makespan>." << std::endl;
    std::cout << "" << std::endl;
    std::cout << "bin/CPFSolver -i|-input-file   <input-file>" << std::endl;
    std::cout << "              -o|-output-file  <output-file>" << std::endl;
    std::cout << "              -s|-stats-file   <stats-file>" << std::endl;
    std::cout << "            -max|-max-makespan <max-makespan>" << std::endl;
    std::cout << "              -v|-verbose      <verbosity>" << std::endl;
    std::cout << "\t <input-file> is a path to an existing CPF file." << std::endl;
    std::cout << "\t <output-file> is a path where the solution file will be created." << std::endl;
    std::cout << "\t <stats-file> is a path where the statistics file will be created." << std::endl;
    std::cout << "\t <max-makespan> is an integer." << std::endl;
    std::cout << "\t <verbosity> is an integer from 0 to 2." << std::endl;
    std::cout << "\t Will solve <input-file> with maximum makespan <max-makespan>" << std::endl;
    std::cout << "\t with <verbosity> verbosity level. The solution will be saved" << std::endl;
    std::cout << "\t at <output-file> and the SAT-solver statistics at <stats-file>." << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Default values:" << std::endl;
    std::cout << "\t <input-file>: ../instances/grid_4x4_a6_o0.1_s616.cpf" << std::endl;
    std::cout << "\t <max-makespan>: the number of vertices of the instance." << std::endl;
    std::cout << "\t <verbose>: 0" << std::endl;
    std::cout << "" << std::endl;
}

int main(int argc, const char **argv) {
    std::string input_file = "../instances/grid_4x4_a6_o0.1_s616.cpf";
    std::string output_file = "";
    std::string stats_file = "";
    int max_makespan = -1;
    int verbose = 0;

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
        for(int i = 3; i < argc-1; ++i) {
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
            else if(arg == "-v" || arg == "-verbose") {
                verbose = atoi(argv[i+1]);
            }
        }
    }

    Parser parser(input_file);

    Instance inst = parser.parse();
    
    if(verbose > 0)
        std::cout << inst << std::endl;

    if(max_makespan < 0)
        max_makespan = inst.n_vertices();

    std::cout << "Solving instance:" << std::endl;

    CPFSolver s1(inst, "simplified", "binary", max_makespan, verbose);

    Solution s = s1.solve();

    if(s.is_empty()) {
        std::cout << "No solution found." << std::endl;
        return 0;
    }

    if(output_file != "") {
        std::ofstream ofs;
        ofs.open(output_file);
        ofs << s << std::endl;
    }

    if(verbose > 0 || output_file == "") {
        std::cout << s << std::endl;
    }

    return 0;
}