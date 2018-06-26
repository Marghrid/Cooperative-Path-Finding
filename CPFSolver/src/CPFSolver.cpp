/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: CPFSolver.cpp                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "OutOfMemoryException.h"
#include "TimeoutException.h"
#include "CPFSolver.h"

#include "SimplifiedEncoder.h"

#include "UNSAT_SATSearch.h"
#include "SAT_UNSATSearch.h"
#include "BinarySearch.h"

#include <ctime>

CPFSolver::CPFSolver(Instance &instance, std::string encoding, std::string search,
                     int verbose, long timeout, int max_makespan)
        : _instance(instance), _solution(instance), _solver(),
          _verbose(verbose), _max_makespan(max_makespan), _timeout(timeout) {

    if (_timeout < 0) _timeout = 3600;
    if (_max_makespan < 0) _max_makespan = instance.max_makespan();
    _solver.setIncrementalMode();
    _solver.verbEveryConflicts = 10;
    _solver.verbosity = 0;
    create_encoder(encoding);
    create_search(search);
}

Solution CPFSolver::solve() {
    double cpu0;
    if (!_instance.check()) {
        //exception
        std::cerr << "The instance wasn't ready to solve" << std::endl;
    }

    // The initial values of currently_solved and current_makespan must be
    //  attributed by the Search, so they will be coherent with the break test.
    bool currently_solved = _search->get_initial_solved();
    int current_makespan = _search->get_initial_makespan();

    // Start counting time:
    cpu0 = std::clock();

    // break test delegated to Search
    while (!_search->break_test(currently_solved)) {

        // Just a precaution. Should not happen.
        if (current_makespan < 0)
            throw std::runtime_error("Unexpected makespan value");

        if (_verbose > 0)
            std::cout << "Trying makespan = " << current_makespan << std::endl;

        currently_solved = solve_for_makespan(current_makespan);

        if (currently_solved) {
            // _solution dependent on the Encoder's interpretation of the variables.
            _solution = _encoder->get_solution(current_makespan);
        }

        current_makespan = _search->get_next_makespan(currently_solved);

        _solve_time = (std::clock() - cpu0) / CLOCKS_PER_SEC;
        if (_verbose > 0)
            std::cout << "Time elapsed: " << _solve_time << std::endl;
        if (_solve_time > _timeout) {
            _status = -2;
            throw TimeoutException("Solver timed out.");
        }
    }

    if (_search->success()) {
        _status = 1;
        std::cout << "Solved for makespan = " << _search->get_successful_makespan() << std::endl;
    }

    if (_solution.is_empty()) {
        _status = 2;
    }

    _solve_time = (std::clock() - cpu0) / CLOCKS_PER_SEC;

    return _solution;
}

bool CPFSolver::solve_for_makespan(int makespan) {
    if (_verbose > 0)
        std::cout << "Solving for makespan " << makespan << ":" << std::endl;

    _encoder->create_clauses_for_makespan(makespan);

    Glucose::vec<Glucose::Lit> assumptions;
    _encoder->create_goal_assumptions(assumptions, makespan);

    bool satisfiable = false;

    // The (false, true) arguments on solve() prenvent the solver from
    //  performing optimizations which could result in variable elimination.
    //  (view lines 172-187 on include/glucose/simp/SimpSolver.cc)
    // This is a problem because I use variables from one iteration to another,
    //  resulting in a call of solve() between their creation, and the creation
    //  of clauses containing them.
    // NOTE: I should find out how much impact these optimizations I'm
    //  preventing actually have.
    try {
        satisfiable = _solver.solve(assumptions, false, true);
    } catch (Glucose::OutOfMemoryException e) {
        _status = -1;
        throw OutOfMemoryException("Out of memory declared by Glucose.");
    }

    if (!satisfiable) {
        ++_n_unsat_calls;
        if (_verbose > 0)
            std::cout << "No solution for makespan " << makespan << std::endl;
        return false;
    }

    ++_n_sat_calls;
    return true;
}

// Used on constructors.
void CPFSolver::create_encoder(std::string encoding) {
    for (auto &a: encoding) a = static_cast<char>(tolower(a));

    if (encoding == "simplified")
        _encoder = new SimplifiedEncoder(_instance, &_solver, _verbose);
    else
        std::cerr << "Unknown encoding: " << encoding << std::endl;
}

// Used on constructors.
void CPFSolver::create_search(std::string search) {
    for (auto &a: search) a = static_cast<char>(tolower(a));

    if (search == "unsat-sat")
        _search = new UNSAT_SATSearch(_instance.min_makespan(), _max_makespan);

    else if (search == "sat-unsat")
        _search = new SAT_UNSATSearch(_instance.min_makespan(), _max_makespan);

    else if (search == "binary")
        _search = new BinarySearch(_instance.min_makespan(), _max_makespan);

    else
        std::cerr << "Unknown search: " << search << std::endl;
}

void CPFSolver::print_status(std::ostream &os) const {
    switch (_status) {
        case 1:
            os << "Solution found (SAT) for makespan " << _solution.n_timesteps() << std::endl;
            break;
        case 2:
            os << "No solution (UNSAT)." << std::endl;
            break;
        case -1:
            os << "Out of memory." << std::endl;
            break;
        case -2:
            os << "Solver timed out." << std::endl;
        default:
            break;
    }
}

void CPFSolver::print_stats(std::ostream &os) const {
    print_status(os);

    os << "Instance size:" << "\n";
    os << "  agents: " << _instance.n_agents() << "\n";
    os << "  vertices: " << _instance.n_vertices() << "\n";
    os << "  edges: " << _instance.n_edges() << "\n";
    os << "" << "\n";

    os << "Solver settings:" << "\n";
    os << "  encoding: " << _encoder->name() << "\n";
    os << "  search: " << _search->name() << "\n";
    os << "" << "\n";

    os << "Solving CPU time: " << _solve_time << " s" << "\n";
    os << "" << std::endl;
}
