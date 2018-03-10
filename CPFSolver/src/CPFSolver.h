#ifndef __CPFSOLVER__
#define __CPFSOLVER__

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
    Encoder *_encoder{};
    Search *_search{};

    int _verbose;
    long _max_makespan;
    long _timeout;

    int _n_sat_calls = 0;
    int _n_unsat_calls = 0;
    double _solve_time = 0;

public:
    // Complete constructor:
    CPFSolver(Instance &instance, std::string &encoding, std::string &search,
              int verbose, long timeout, int max_makespan);

    // Constructor with default maximum makespan.
    CPFSolver(Instance &instance, std::string &encoding, std::string &search, int verbose, long timeout)
            : CPFSolver(instance, encoding, search, verbose, timeout, instance.max_makespan()) {}

    // Constructor with default maximum makespan and timeout.
    CPFSolver(Instance &instance, std::string &encoding, std::string &search, int verbose)
            : CPFSolver(instance, encoding, search, verbose, 172800 /* 48 hours */, instance.max_makespan()) {}

    // Constructor with default maximum makespan, timeout and verbosity.
    CPFSolver(Instance &instance, std::string &encoding, std::string &search)
            : CPFSolver(instance, encoding, search, 0, 172800 /* 48 hours */, instance.max_makespan()) {}

    // Minimal constructor. Default maximum makespan, timeout, verbosity, encoding and search.
    CPFSolver(Instance instance, int verbose = 0)
            : CPFSolver(instance, (std::string &) "simplified", (std::string &) "UNSAT-SAT",
                        0, 172800 /* 48 hours */, instance.max_makespan()) {}

    ~CPFSolver() {
        delete _encoder;
        delete _search;
    }

    // Main constructor functionality. Provides a solution for its instance.
    Solution solve();

    // Write SAT solver statistics to a file.
    void write_stats(std::ostream &os);

    double get_solving_time() { return _solve_time; }

private:

    // Auxiliary function. Used on solve(). Tries to find a solution for
    //   the instance with a given makespan
    bool solve_for_makespan(int makespan);

    // Auxiliary function. Used on constructors.
    void create_encoder(std::string &encoding);

    // Auxiliary function. Used on constructors.
    void create_search(std::string &search);

};

#endif
