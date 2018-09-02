/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: CPFSolver.h                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __CPFSOLVER__
#define __CPFSOLVER__

#include <vector>
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
 */

class CPFSolver {
private:
	Instance _instance;
	Solution _solution;
	Glucose::SimpSolver _solver;

	/* Strategies: */
	EncoderID *_encoder;
	Search *_search;

	int _force_ID;
	int _verbose;
	int _max_makespan;
	long _timeout;

	int _n_sat_calls = 0;
	int _n_unsat_calls = 0;
	double _solve_time = 0;

	// Increasing order of complexity
	std::vector<std::shared_ptr<Group>> _groups;

	// 0 - Unsolved
	// 1 - Solution found (SAT)
	// 2 - Solution does not exist (UNSAT)
	// -1 - Out of memory
	// -2 - Timed out
	short _status = 0;


public:
	// Complete constructor:
	CPFSolver(Instance &instance, std::string search, std::string encoding,
	          int force_ID, int max_makespan, long timeout, int verbose = 0);

	// Constructor with default timeout.
	CPFSolver(Instance &instance, std::string search, std::string encoding,
	          int force_ID, int max_makespan, int verbose = 0)
			: CPFSolver(instance, std::move(search), std::move(encoding), force_ID, max_makespan, 172800, verbose) {}

	// Constructor with default maximum makespan and timeout.
	CPFSolver(Instance &instance, std::string search, std::string encoding, int force_ID, int verbose = 0)
			: CPFSolver(instance, std::move(search), std::move(encoding),
			            force_ID, instance.max_makespan(), 172800, verbose) {}

	// Constructor with default maximum makespan, timeout and independence detection.
	CPFSolver(Instance &instance, std::string search, std::string encoding, int verbose = 0)
			: CPFSolver(instance, std::move(search), std::move(encoding), -1,
			            instance.max_makespan(), 172800, verbose) {}


	// Minimal constructor. Default maximum makespan, timeout, independence detection, encoding and search.
	explicit CPFSolver(Instance instance, int verbose = 0)
			: CPFSolver(instance, (std::string &) "UNSAT-SAT", (std::string &) "simplified",
			            -1, instance.max_makespan(), 172800 /* 48 hours */, verbose) {}

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

	bool solve_group_for_makespan(std::shared_ptr<Group> group, int makespan, Glucose::vec<Glucose::Lit> &assumptions);

	void group();

	// Auxiliary function. Used on constructors.
	void create_encoder(std::string encoding);

	// Auxiliary function. Used on constructors.
	void create_search(std::string search);

	std::shared_ptr<Group> get_conflicting_group(std::shared_ptr<Group> group);

	void merge(unsigned int g1_idx, unsigned int g2_idx);

	Solution merge_solutions();

	void merge_all();

	bool check_conflict(std::shared_ptr<Group> &g1, std::shared_ptr<Group> &g2);
};

#endif
