#include "CPFSolver.h"

#include "Instance.h"
#include "Solution.h"
#include "Encoder.h"
#include "SimplifiedEncoder.h"

#include "Search.h"
#include "UNSAT_SATSearch.h"
#include "SAT_UNSATSearch.h"
#include "BinarySearch.h"

Solution CPFSolver::solve() {
    // DIRECT:
    if(!_instance.check()) {
        //exception
        std::cerr << "The instance wasn't ready to convert" << std::endl;
    }

    bool current_solved = _search->get_initial_solved();

    int current_makespan = _search->get_initial_makespan();

    while(!_search->break_test(current_solved)) {

        if(current_makespan < 0) break;

        if(_verbose > 0)
        	std::cout << "Trying makespan = " << current_makespan << std::endl;
        current_solved = solve_for_makespan(current_makespan);
        current_makespan = _search->get_next_makespan(current_solved);
    }

    Solution solution;
    
    if(_search->success()) {
        std::cout << "Solved for makespan = " << _search->get_successful_makespan() << std::endl;
        solution = _encoder->get_solution(_search->get_successful_makespan());
    }

    _solver.printIncrementalStats();

    return solution;
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
    	if(_verbose > 0)
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
