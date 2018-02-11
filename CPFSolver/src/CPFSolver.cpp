#include "CPFSolver.h"
#include "SimplifiedEncoder.h"

bool CPFSolver::solve_for_makespan(int makespan) {
    if(_verbose > 0)
        std::cout << "Solving for makespan " << makespan << ":" << std::endl;

    _encoder.create_clauses_for_makespan(makespan);

    Glucose::vec<Glucose::Lit> assumptions;
    _encoder.create_goal_assumptions(assumptions, makespan);

    // The (false, true) arguments on solve() prenvent the solver from
    //  performing optimizations which could result in variable elimination.
    //  (view lines 172-187 on include/glucose/simp/SimpSolver.cc)
    // This is a problem because I use variables from one iteration to another,
    //  resulting in a call of solve() between their creation, and the creation
    //  of clauses containing them.
    // I should find out how much impact these optimizations I'm preventing
    //  actually have.

    std::cout << "vars " << _solver.nVars() << std::endl;
    bool satisfiable = _solver.solve(assumptions, false, true);
    
    if(!satisfiable) {
        std::cout << "No solution for makespan " << makespan << std::endl;
        return false;
    }

    _encoder.show_results(makespan);

    return satisfiable;
}

#include "CPFSolver.h"

bool CPFSolver::solve() {
    // DIRECT:
    if(!_instance.check()) {
        //exception
        std::cerr << "The instance wasn't ready to convert" << std::endl;
    }

    bool solved = false;

    int current_makespan = get_initial_makespan();

    for( ; current_makespan < _max_makespan && solved != true; ++current_makespan) {
        std::cout << "Trying makespan = " << current_makespan << std::endl;
        solved = solve_for_makespan(current_makespan);
    }
    
    if(solved) {
        std::cout << "Solved for makespan = " << --current_makespan << std::endl;
    }
    return solved;
}