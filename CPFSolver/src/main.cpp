#include "OutOfMemoryException.h"
#include "TimeoutException.h"
#include "Instance.h"
#include "Parser.h"
#include "CPFSolver.h"

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
    std::cout << "              -t|-timeout      <timeout>"      << "\n";

    std::cout << "  <input-file>:   the path to an existing CPF file."                      << "\n";
    std::cout << "  <output-file>:  the path to where the solution file will be created."   << "\n";
    std::cout << "  <stats-file>:   the path to where the statistics file will be created." << "\n";
    std::cout << "  <max-makespan>: an integer"                                             << "\n";
    std::cout << "  <encoding>:     simplified|other_encoding_that_i_will_implement_soon"   << "\n";
    std::cout << "  <search>:       UNSAT-SAT|SAT-UNSAT|binary"                             << "\n";
    std::cout << "  <verbosity>:    0|1|2"                                                  << "\n";
    std::cout << "  <timeout>:      an integer, in seconds."                                << "\n";
    std::cout << "  Will solve <input-file> with maximum makespan <max-makespan>"           << "\n";
    std::cout << "  with <verbosity> verbosity level. The solution will be saved"           << "\n";
    std::cout << "  at <output-file> and the SAT solver statistics at <stats-file>."        << "\n";
    std::cout << ""                                                                         << "\n";
    std::cout << "Default values:"                                                          << "\n";
    std::cout << "  <input-file>: ../instances/grid_4x4_a6_o0.1_s616.cpf"                   << "\n";
    std::cout << "  <max-makespan>: the number of vertices of the instance."                << "\n";
    std::cout << "  <encoding>: simplified."                                                << "\n";
    std::cout << "  <search>: UNSAT-SAT."                                                   << "\n";
    std::cout << "  <verbose>: 0"                                                           << "\n";
    std::cout << "" << std::endl;
}


int main(int argc, const char **argv) {
    std::string input_file = "src/example.cpf";
    std::string output_file;
    std::string stats_file;
    std::string encoding = "simplified";
    std::string search = "unsat-sat";

    long max_makespan = -1;
    short verbose = 0;
    long timeout = -1;

    bool timed_out = false;
    bool out_of_memory = false;
    bool unknown_error = false;

    // to use in strtol
    char* aux;

    if(argc == 1) {
        print_usage_instructions();
        std::cout << "solving for default file: example.cpf" << std::endl;
    }
    else if(argc == 2) {
        input_file = argv[1];
    }
    else if(argc == 3) {
        input_file = argv[1];
        max_makespan = strtol(argv[2], &aux, 10);
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
                max_makespan = strtol(argv[i+1], &aux, 10);
            }
            else if(arg == "-e" || arg == "-encoding") {
                encoding = argv[i+1];
            }
            else if(arg == "-search") {
                search = argv[i+1];
            }
            else if(arg == "-v" || arg == "-verbose") {
                verbose = static_cast<short>(strtol(argv[i + 1], &aux, 10));
            }
            else if(arg == "-t" || arg == "-timeout") {
                timeout = strtol(argv[i+1], &aux, 10);
            }
        }
    }

    Parser parser(input_file);

    Instance instance = parser.parse();

    if(verbose > 0)
        std::cout << instance << std::endl;

    if(max_makespan < 0)
        max_makespan = instance.n_vertices() * instance.n_agents();

    std::cout << "Solving instance with encoding " << encoding
    	<< " and search " << search << "." << std::endl;

    CPFSolver solver(instance, encoding, search, verbose, timeout);

    Solution solution(instance);

    try {
        solution = solver.solve();
    } catch (const TimeoutException& e) {
        timed_out = true;
    } catch (const OutOfMemoryException& e) {
        out_of_memory = true;
    } catch (const std::runtime_error& e) {
        unknown_error = true;
    }

    if (!stats_file.empty() && out_of_memory) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "Out of memory." << std::endl;
        ofs.close();
        out_of_memory = false;
    }
    else if (!stats_file.empty() && timed_out) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "Solver timed out." << std::endl;
        ofs.close();
        timed_out = false;
    }
    else if (!stats_file.empty() && unknown_error) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "Unknown error." << std::endl;
        ofs.close();
        unknown_error = false;
    }
    else if (!stats_file.empty() && !solution.is_empty() ) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "Solution found." << std::endl;
        solver.print_stats(ofs);
        ofs.close();
    }
    else if (!stats_file.empty()) {
        std::ofstream ofs;
        ofs.open(stats_file);
        ofs << "No solution found." << std::endl;
        solver.print_stats(ofs);
        ofs.close();
    }

    if(!output_file.empty()) {
        std::ofstream ofs;
        ofs.open(output_file);
        ofs << solution << std::endl;
		ofs.close();
    }

    if(verbose > 0 || output_file.empty()) {
        std::cout << solution << std::endl;
    }

    if(verbose > 0 || stats_file.empty()) {
    	solver.print_stats(std::cout);
    }
    return 0;
}
