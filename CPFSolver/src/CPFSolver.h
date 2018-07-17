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
#include "Group.h"
#include "EncoderID.h"


/* Independence detection:
 * Create a Solution for each group, in the end, merge all solutions.
 *
 * -----------------------------------------------------------------------------
 * Surynek:
 *
 *  assign each agent to a group;
 *  plan a path for each group G1, ..., Gk using SAT solver;
 *  fill conflict avoidance table
 *  while conflicting groups exist:
 *      G1, G2 = conflicting groups;
 *      if G1, G2 not conflicted before:
 *          replan G1 using SAT solver considering G - G1
 *
 *          if failed to replan G1:
 *              replan G2 using SAT solver considering G - G2
 *          endif
 *      endif
 *
 *      if no alternative paths for G1, G2:
 *          merge G1, G2;
 *          plan a path for new group using SAT solver;
 *      endif
 *      update conflict avoidance table
 *
 *  end
 *  return combined paths for all groups
 *
 * -----------------------------------------------------------------------------
 * Margarida:
 *
 *  for each group G1:
 *      plan path considering all already planned groups;
 *      if a conflict is found:
 *          G2 = other group in conflict;
 *          replan G1 using SAT solver with current makespan
 *
 *          if failed to replan G1:
 *              replan G2 using SAT solver with current makespan
 *          endif
 *
 *          if failed to replan G2:
 *              Gx = merge G1, G2;
 *              plan Gx using SAT solver with current makespan
 *          endif
 *
 *          if failed to replan Gx:
 *              try from the begining with new makespan
 *          endif
 *      endif
 */

class CPFSolver {
private:
	Instance _instance;
	Solution _solution;
	Glucose::SimpSolver _solver;

	/* Strategies: */
	EncoderID *_encoder;
	Search *_search;

	// Increasing order of complexity
	std::list<std::shared_ptr<Group>> _unplanned_groups;

	// Increasing order of complexity
	std::list<std::shared_ptr<Group>> _planned_groups;

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
	CPFSolver(Instance &instance, std::string encoding, std::string search, int verbose,
	          long timeout)
			: CPFSolver(instance, std::move(encoding), std::move(search), verbose, timeout,
			            instance.max_makespan()) {}

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

	bool solve_group_for_makespan(std::shared_ptr<Group> group, int makespan);

	void group();

	// Auxiliary function. Used on constructors.
	void create_encoder(std::string encoding);

	// Auxiliary function. Used on constructors.
	void create_search(std::string search);

	std::shared_ptr<Group> get_conflicting_group(std::shared_ptr<Group> group);

	void merge(std::shared_ptr<Group> group1, std::shared_ptr<Group> group2);

	Solution merge_solutions();

	void merge_all();
};

#endif
