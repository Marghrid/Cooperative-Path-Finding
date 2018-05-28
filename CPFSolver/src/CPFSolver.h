/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: CPFSolver.h                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __CPFSOLVER__
#define __CPFSOLVER__

#include <utility>
#include "Instance.h"
#include "Parser.h"
#include "Encoder.h"
#include "Search.h"
#include "simp/SimpSolver.h"

class CPFSolver {
private:
    Instance _instance;
    Solution _solution;
    Glucose::SimpSolver _solver;

    /* Strategies: */
    Encoder *_encoder;
    Search *_search;

    int _verbose;
    int _max_makespan;
    long _timeout;

    int _n_sat_calls = 0;
    int _n_unsat_calls = 0;
    double _solve_time = 0;

    // 0 - Unsolved
    // 1 - Solution found (SAT)
    // 2 - Solution does not exist (UNSAT)
    // -1 - Out of memory
    // -2 - Timed out
    short _status = 0;

public:
    // Complete constructor:
    CPFSolver(Instance &instance, std::string encoding, std::string search,
              int verbose, long timeout, int max_makespan);

    // Constructor with default maximum makespan.
    CPFSolver(Instance &instance, std::string encoding, std::string search, int verbose, long timeout)
            : CPFSolver(instance, std::move(encoding), std::move(search), verbose, timeout, instance.max_makespan()) {}

    // Constructor with default maximum makespan and timeout.
    CPFSolver(Instance &instance, std::string encoding, std::string search, int verbose)
            : CPFSolver(instance, std::move(encoding), std::move(search), verbose,
                        172800 /* 48 hours */, instance.max_makespan()) {}

    // Constructor with default maximum makespan, timeout and verbosity.
    CPFSolver(Instance &instance, std::string encoding, std::string search)
            : CPFSolver(instance, std::move(encoding),
                        std::move(search), 0, 172800 /* 48 hours */, instance.max_makespan()) {}

    // Minimal constructor. Default maximum makespan, timeout, verbosity, encoding and search.
    explicit CPFSolver(Instance instance, int verbose = 0)
            : CPFSolver(instance, (std::string &) "simplified", (std::string &) "UNSAT-SAT",
                        0, 172800 /* 48 hours */, instance.max_makespan()) {}

    ~CPFSolver() {
        delete _encoder;
        delete _search;
    }

    // Main solver functionality. Provides a solution for its instance.
    Solution solve();

    double get_solving_time() { return _solve_time; }

    void print_status(std::ostream &os) const;

    void print_stats(std::ostream &os) const;

private:

    // Auxiliary function. Used on solve(). Tries to find a solution for
    //   the instance with a given makespan
    bool solve_for_makespan(int makespan);

    // Auxiliary function. Used on constructors.
    void create_encoder(std::string encoding);

    // Auxiliary function. Used on constructors.
    void create_search(std::string search);
};

#endif
