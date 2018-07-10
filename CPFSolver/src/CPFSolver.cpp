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

#include "mtl/Vec.h"
#include "core/SolverTypes.h"

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

bool CPFSolver::solve_group_for_makespan(std::shared_ptr<Group> group, int makespan) {
	bool satisfiable;

	if (_verbose > 0)
		std::cout << "Solving group " << *group << " for makespan " << makespan << ":" << std::endl;

	_encoder->create_clauses_for_group_makespan(group.get(), makespan);
	Glucose::vec<Glucose::Lit> assumptions;
	_encoder->create_group_goal_assumptions(group.get(), assumptions, makespan);
	_encoder->create_planned_groups_assumptions(group, _planned_groups,
	                                            assumptions, makespan);

	/* The (false, true) arguments on solve() prenvent the solver from
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

	if (!satisfiable) {
		++_n_unsat_calls;
		if (_verbose > 0)
			std::cout << "No solution for makespan " << makespan << std::endl;

		std::cout << "conflict: " << _solver.conflict.size() << std::endl;
		while (_solver.conflict.size() > 0) {
			std::cout << " " << _solver.conflict.last().x << std::endl;
			_solver.conflict.pop();
		}
		return false;
	}

	_encoder->get_group_solution(group.get(), makespan);
	++_n_sat_calls;
	return true;
}

/*
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
}*/


Solution CPFSolver::solve() {
	double cpu0;
	if (!_instance.check()) {
		//exception
		std::cerr << "The instance wasn't ready to solve" << std::endl;
	}

	group();

	// The initial values of currently_solved and current_makespan must be
	//  attributed by the Search, so they will be coherent with the break test.
	bool currently_solved = _search->get_initial_solved();
	unsigned current_makespan = _search->get_initial_makespan();

	// Start counting time:
	cpu0 = std::clock();

	// break test delegated to Search
	while (!_search->break_test(currently_solved)) {

		if (_verbose > 0)
			std::cout << "Trying makespan = " << current_makespan << std::endl;

		while (!_unplanned_groups.empty()) {
			std::shared_ptr<Group> group = _unplanned_groups.back();
			currently_solved = solve_group_for_makespan(group, current_makespan);

			if (currently_solved) {
				_unplanned_groups.pop_back();
				_planned_groups.push_back(group);
			} else {
				std::shared_ptr<Group> conflicting_group = get_conflicting_group(group);
				//TODO try to replan conflicting_group
				merge(group, conflicting_group);
				current_makespan = _search->get_next_makespan(currently_solved);
			}
		}


		if (currently_solved) {
			// _solution dependent on the Encoder's interpretation of the variables.
			//_solution = _encoder->get_group_solution(_unplanned_groups[0], current_makespan);
			for (std::shared_ptr<Group> group : _planned_groups) {
				std::cout << group->solution << std::endl;
			}
			_solution = merge_solutions();

			//std::cout << "\nSOLUTION:" << _solution << std::endl;
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

bool CPFSolver::solve_for_makespan(int makespan) {
	if (_verbose > 0)
		std::cout << "Solving for makespan " << makespan << ":" << std::endl;

	_encoder->create_clauses_for_makespan(makespan);

	Glucose::vec<Glucose::Lit> assumptions;
	_encoder->create_goal_assumptions(assumptions, makespan);

	bool satisfiable = false;

	/* The (false, true) arguments on solve() prenvent the solver from
	 *  performing optimizations which could result in variable elimination.
	 *  (view lines 172-187 on include/glucose/simp/SimpSolver.cc)
	 * This is a problem because I use variables from one iteration to another,
	 *  resulting in a call of solve() between their creation, and the creation
	 *  of clauses containing them.
	 * NOTE: I should find out how much impact these optimizations I'm
	 *  preventing actually have.
	 */
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
	os << "" << "\n";

	os << "Solving CPU time: " << _solve_time << " s" << "\n";
	os << "" << std::endl;
}

std::shared_ptr<Group> CPFSolver::get_conflicting_group(std::shared_ptr<Group> group) {
	return group;
}

void CPFSolver::merge(std::shared_ptr<Group> group1, std::shared_ptr<Group> group2) {
	// Remove these 2 groups from their vectors
	_unplanned_groups.remove(*std::find(_unplanned_groups.begin(), _unplanned_groups.end(), group1));
	_unplanned_groups.remove(*std::find(_unplanned_groups.begin(), _unplanned_groups.end(), group2));
	_planned_groups.remove(*std::find(_planned_groups.begin(), _planned_groups.end(), group1));
	_planned_groups.remove(*std::find(_planned_groups.begin(), _planned_groups.end(), group2));

	// Create new group with the agents from both
	std::shared_ptr<Group> new_group = std::make_shared<Group>(_instance);
	for (std::shared_ptr<Agent> a : group1->agents)
		new_group->add_agent(a);
	for (std::shared_ptr<Agent> a : group2->agents)
		new_group->add_agent(a);

	// Move all planned groups to unplanned in reverse order.
	std::reverse(std::begin(_planned_groups), std::end(_planned_groups));
	_unplanned_groups.insert(_unplanned_groups.end(), _planned_groups.begin(),
	                         _planned_groups.end());
	// Add new group to the back of unplanned.
	_unplanned_groups.push_back(new_group);

	_planned_groups.clear();
}

void CPFSolver::group() {
	// For now, all agents go in one big group

	std::cout << "Grouping agents:" << std::endl;
	if ((double) _instance.n_agents() / (double) _instance.n_vertices() > 0.4) {
		std::shared_ptr<Group> all = std::make_shared<Group>(_instance);
		for (std::shared_ptr<Agent> &a : _instance.agents()) {
			all->add_agent(a);
		}
		_unplanned_groups.push_back(all);
	} else {
		std::vector<std::shared_ptr<Agent>> aux(_instance.agents());
		while (!aux.empty()) {
			// Each agent is added to its own group, and the groups are put in _unplanned_groups
			// The groups are added in order of increasing distance to goal.
			std::shared_ptr<Group> g = std::make_shared<Group>(_instance);
			g->add_agent(aux.back());
			aux.pop_back();
			_unplanned_groups.push_back(g);
		}
	}

	std::cout << "Grouped:" << std::endl;
	for (auto g : _unplanned_groups)
		std::cout << *g << " ";

	std::cout << std::endl;
}

Solution CPFSolver::merge_solutions() {
	auto pg_it = _planned_groups.begin();
	Solution solution = (*pg_it)->solution;
	++pg_it;

	for (; pg_it != _planned_groups.end(); ++pg_it) {
		solution.merge((*pg_it)->solution);
		std::cout << "Merged " << solution;
	}
	return solution;
}
