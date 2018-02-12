#include "CPFSolver.h"

#include "Encoder.h"
#include "SimplifiedEncoder.h"

#include "Search.h"
#include "UNSAT_SATSearch.h"
#include "SAT_UNSATSearch.h"
#include "BinarySearch.h"

bool CPFSolver::solve() {
    // DIRECT:
    if(!_instance.check()) {
        //exception
        std::cerr << "The instance wasn't ready to convert" << std::endl;
    }

    bool solved = _search->get_initial_solved();

    int current_makespan = _search->get_initial_makespan();

    std::cout << "Starting search:" << std::endl;
    while(!_search->break_test(solved)) {

        if(current_makespan < 0) break;

        std::cout << "Trying makespan = " << current_makespan << std::endl;
        solved = solve_for_makespan(current_makespan);
        current_makespan = _search->get_next_makespan(solved);
    }
    
    if(_search->success()) {
        std::cout << "Solved for makespan = " << _search->get_successful_makespan() << std::endl;
        _encoder->show_results(_search->get_successful_makespan());
    }
    return solved;
}

bool CPFSolver::solve_for_makespan(int makespan) {
    if(_verbose > 0)
        std::cout << "Solving for makespan " << makespan << ":" << std::endl;

    _encoder->create_clauses_for_makespan(makespan);

    Glucose::vec<Glucose::Lit> assumptions;
    _encoder->create_goal_assumptions(assumptions, makespan);

    // The (false, true) arguments on solve() prenvent the solver from
    //  performing optimizations which could result in variable elimination.
    //  (view lines 172-187 on include/glucose/simp/SimpSolver.cc)
    // This is a problem because I use variables from one iteration to another,
    //  resulting in a call of solve() between their creation, and the creation
    //  of clauses containing them.
    // I should find out how much impact these optimizations I'm preventing
    //  actually have.
    bool satisfiable = _solver.solve(assumptions, false, true);
    
    if(!satisfiable) {
        std::cout << "No solution for makespan " << makespan << std::endl;
        return false;
    }

    return satisfiable;
}

void CPFSolver::create_encoder(std::string encoding) {
    for(auto &a: encoding) a = tolower(a);

    if(encoding == "simplified")
        _encoder = new SimplifiedEncoder(_instance, &_solver, _verbose);
    else
        std::cerr << "Unknown encoding: " << encoding << std::endl;
}

void CPFSolver::create_search(std::string search) {
    for(auto &a: search) a = tolower(a);

    if(search == "unsat-sat")
        _search = new UNSAT_SATSearch(_max_makespan);

    else if(search == "sat-unsat")
        _search = new SAT_UNSATSearch(_max_makespan);

    else if(search == "binary")
      _search = new BinarySearch(_max_makespan);

    else
        std::cerr << "Unknown search: " << search << std::endl;
}
