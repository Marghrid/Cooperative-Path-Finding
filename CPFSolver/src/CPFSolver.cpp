/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: CPFSolver.cpp                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "OutOfMemoryException.h"
#include "TimeoutException.h"
#include "CPFSolver.h"

#include "SimplifiedID.h"

#include "UNSAT_SATSearch.h"
#include "SAT_UNSATSearch.h"
#include "BinarySearch.h"

#include "mtl/Vec.h"
#include "core/SolverTypes.h"

#include <ctime>

CPFSolver::CPFSolver(Instance &instance, std::string search, std::string encoding, int force_ID, int max_makespan,
                     long timeout, int verbose)
		: _instance(instance), _solution(instance), _solver(),
		  _verbose(verbose), _max_makespan(max_makespan), _timeout(timeout) {

	if (_timeout < 0) _timeout = 3600;
	if (_max_makespan < 0) _max_makespan = instance.max_makespan();
	_force_ID = force_ID;
	_solver.setIncrementalMode();
	_solver.verbEveryConflicts = 10;
	_solver.verbosity = 0;
	create_encoder(encoding);
	create_search(search);
}

bool CPFSolver::solve_group_for_makespan(std::shared_ptr<Group> group, int makespan) {
	Glucose::vec<Glucose::Lit> assumptions;
	return solve_group_for_makespan(group, makespan, assumptions);
}

bool CPFSolver::solve_group_for_makespan(std::shared_ptr<Group> group, int makespan,
                                         Glucose::vec<Glucose::Lit> &assumptions) {
	bool satisfiable;

	if (group->last_solved_makespan == makespan) return true;

	if (_verbose > 0)
		std::cout << "Solving group " << *group << " for makespan " << makespan << ":" << std::endl;

	_encoder->create_clauses_for_makespan(group.get(), makespan);
	_encoder->create_goal_assumptions(group.get(), assumptions, makespan);

	/* The (false, true) arguments on solve() prevent the solver from
	 *  performing optimizations which could result in variable elimination.
	 *  (view lines 172-187 on include/glucose/simp/SimpSolver.cc)
	 * This is a problem because I use variables from one iteration to another,
	 *  resulting in a call of solve() between their creation, and the creation
	 *  of clauses containing them.
	 * NOTE: I should find out how much impact these optimizations I'm
	 *  preventing actually have.
	 */
	try {
		satisfiable = group->solver->solve(assumptions, false, true);
	} catch (Glucose::OutOfMemoryException e) {
		_status = -1;
		throw OutOfMemoryException("Out of memory declared by Glucose.");
	}

	if (satisfiable) {
		if (_verbose > 0)
			std::cout << "A solution was found for group " << *group
			          << " with makespan " << makespan << std::endl;
		++_n_sat_calls;
		group->last_solved_makespan = makespan;
		_encoder->get_solution(group.get(), makespan);

		return true;
	} else {
		if (_verbose > 0)
			std::cout << "Group " << *group << " has no solution for makespan "
			          << makespan << std::endl;
		++_n_unsat_calls;
		return false;
	}
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
	unsigned current_makespan = _search->get_initial_makespan();

	group();

	// Start counting time:
	cpu0 = std::clock();

	// break test delegated to Search
	while (!_search->break_test(currently_solved)) {

		if (_verbose > 0)
			std::cout << "Trying makespan = " << current_makespan << std::endl;

		// Solve each group independently
		bool solved_all_groups = true;
		for (std::shared_ptr<Group> group : _groups) {
			if (!solve_group_for_makespan(group, current_makespan)) {
				std::cout << "Couldn't plan group " << *group << std::endl;
				solved_all_groups = false;
				break;
			} else {
				if (_verbose > 2)
					std::cout << "Group " << *group << " solution: \n" << group->solution << std::endl;
			}
		}

		if (!solved_all_groups) {
			currently_solved = false;
			current_makespan = _search->get_next_makespan(currently_solved);
			group();
			continue;
		}

		if (_verbose > 1)
			std::cout << "Planned all groups. Checking for conflicts:" << std::endl;
		bool merged = false;
		for (int i = _groups.size() - 1; i > 0; --i) {
			for (int j = i - 1; j >= 0; --j) {
				if (check_conflict(_groups[i], _groups[j])) {
					//replan _groups[i]
					if (_verbose > 1)
						std::cout << "Found a conflict between groups " << *_groups[i] << " and " << *_groups[j]
						          << ". Replaning " << *_groups[i] << "." << std::endl;
					std::vector<std::shared_ptr<Group>> planned(_groups);
					planned.erase(planned.begin() + i);
					Glucose::vec<Glucose::Lit> assumptions_i;
					_encoder->create_planned_groups_assumptions(_groups[i], planned, assumptions_i);

					if (!solve_group_for_makespan(_groups[i], current_makespan, assumptions_i) ||
					    check_conflict(_groups[i], _groups[j])) {
						//replan _groups[j]
						if (_verbose > 1)
							std::cout << "Could not replan " << *_groups[i]
							          << ". Replaning " << *_groups[j] << "." << std::endl;
						std::vector<std::shared_ptr<Group>> planned(_groups);
						planned.erase(planned.begin() + j);
						Glucose::vec<Glucose::Lit> assumptions_j;
						_encoder->create_planned_groups_assumptions(_groups[j], planned, assumptions_j);

						if (!solve_group_for_makespan(_groups[j], current_makespan, assumptions_j) ||
						    check_conflict(_groups[i], _groups[j])) {
							if (_verbose > 1)
								std::cout << "Merging." << std::endl;
							merge(i, j);
							merged = true;
							break;
						}
					}
				} else {
					if (_verbose > 1)
						std::cout << "No conflict between " << *_groups[i] << " and " << *_groups[j] << "."
						          << std::endl;
				}
			}
			if (merged) {
				break;
			}
		}
		if (merged) {
			currently_solved = false;
			if (_verbose > 1) {
				std::cout << "Current groups:" << std::endl;
				for (const auto &g : _groups)
					std::cout << *g << " ";

				std::cout << std::endl;
			}
			continue;

		} else {
			currently_solved = true;
			_solution = merge_solutions();
		}

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

// Used on constructors.
void CPFSolver::create_encoder(std::string encoding) {
	for (auto &a: encoding) a = static_cast<char>(tolower(a));

	if (encoding == "simplified")
		_encoder = new SimplifiedID(_instance, &_solver, _verbose);
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
			os << "Solution found (SAT) for makespan " << _solution.n_timesteps() - 1 << std::endl;
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
	os << "  ID: " << ((_force_ID > 0) ? "yes" : "no") << "\n";;
	os << "" << "\n";

	os << "Solving CPU time: " << _solve_time << " s" << "\n";
	os << "" << std::endl;
}

void CPFSolver::group() {
	if (_verbose > 1)
		std::cout << "Grouping agents:" << std::endl;
	_groups.clear();

	if (_force_ID == 0 ||
	    (_force_ID == -1 && (double) _instance.n_agents() / (double) _instance.n_vertices() > 0.5)) {
		_force_ID = 0;
		// If there are many agents, all agents go in one big group
		std::shared_ptr<Group> all = std::make_shared<Group>(_instance);
		for (std::shared_ptr<Agent> &agent : _instance.agents()) {
			all->add_agent(agent);
		}
		_groups.push_back(all);
	} else {
		_force_ID = 1;
		std::vector<std::shared_ptr<Agent>> aux(_instance.agents());
		while (!aux.empty()) {
			// Each agent is added to its own group, and the groups are put in _groups
			// The groups are added in order of increasing distance to goal.
			std::shared_ptr<Group> group = std::make_shared<Group>(_instance);
			group->add_agent(aux.back());
			aux.pop_back();
			_groups.push_back(group);
		}
	}
	if (_verbose > 1) {
		for (auto g : _groups)
			std::cout << *g << " ";

		std::cout << std::endl;
	}
}

Solution CPFSolver::merge_solutions() {
	auto pg_it = _groups.begin();

	Solution solution = (*pg_it)->solution;
	++pg_it;

	for (; pg_it != _groups.end(); ++pg_it) {
		solution.merge((*pg_it)->solution);
		if (_verbose > 3)
			std::cout << "Merged " << solution;
	}
	return solution;
}

void CPFSolver::merge_all() {
	// Create new group with the agents from both
	std::shared_ptr<Group> new_group = std::make_shared<Group>(_instance);
	for (std::shared_ptr<Group> &group : _groups) {
		for (std::shared_ptr<Agent> a : group->agents)
			new_group->add_agent(a);
	}

	_groups.clear();
	// Add new group to unplanned.
	_groups.push_back(new_group);

}

bool CPFSolver::check_conflict(std::shared_ptr<Group> &g1, std::shared_ptr<Group> &g2) {
	// Go through solutions and search for conflicts

	unsigned n_timesteps = (unsigned int) std::min(g1->solution.n_timesteps(), g2->solution.n_timesteps());

	for (const std::shared_ptr<Agent> &g1_agent : g1->agents) {
		for (const std::shared_ptr<Agent> &g2_agent : g2->agents) {
			for (unsigned i = 0; i < n_timesteps; ++i) {
				if (g1->solution.get_position(g1_agent, i) == g2->solution.get_position(g2_agent, i)) {
					// Two agents in the same position at the same time
					if (_verbose > 1)
						std::cout << "Agent " << *g1_agent << " from group " << *g1 << " and "
						          << "agent " << *g2_agent << " from group " << *g2 << " are both at vertex "
						          << g1->solution.get_position(g1_agent, i) << " on timestep " << i << "." << std::endl;
					return true;
				}


				if (i < n_timesteps &&
				    g1->solution.get_position(g1_agent, i) == g2->solution.get_position(g2_agent, i + 1) &&
				    g2->solution.get_position(g2_agent, i) == g1->solution.get_position(g1_agent, i + 1)) {

					// Agents swap places
					if (_verbose > 1)
						std::cout << "Agent " << *g1_agent << " from group " << *g1 << " and "
						          << "agent " << *g2_agent << " from group " << *g2 << " swapped vertices from "
						          << g1->solution.get_position(g1_agent, i) << " to "
						          << g2->solution.get_position(g2_agent, i)
						          << " between timesteps " << i << " and " << i + 1 << std::endl;
					return true;
				}
			}
		}
	}
	return false;
}

void CPFSolver::merge(unsigned int g1_idx, unsigned int g2_idx) {
	// Create new group with the agents from both
	std::shared_ptr<Group> new_group = std::make_shared<Group>(_instance);
	for (std::shared_ptr<Agent> a : _groups[g1_idx]->agents)
		new_group->add_agent(a);
	for (std::shared_ptr<Agent> a : _groups[g2_idx]->agents)
		new_group->add_agent(a);


	// Add new group to the back of unplanned.
	_groups.push_back(new_group);

	if (g1_idx > g2_idx) {
		_groups.erase(_groups.begin() + g1_idx);
		_groups.erase(_groups.begin() + g2_idx);
	} else {
		_groups.erase(_groups.begin() + g2_idx);
		_groups.erase(_groups.begin() + g1_idx);
	}

}
