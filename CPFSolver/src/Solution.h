/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.h:                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __SOLUTION__
#define __SOLUTION__

#include "Instance.h"

#include <vector>

class Solution {
private:
	std::vector<std::vector<int>> _positions;
	// Outer vector element represents a timestep.
	// Inner vector element represents an agent.
	// int on the inner vector represents the agent's
	//  position (vertex#) on each timestep.

	Instance _instance;
	int _current_timestep = -1;

public:

	explicit Solution(Instance &instance) : _instance(instance) {};

	void add(int agentID, int position);

	void increment_timestep();

	int get_position(unsigned timestep, unsigned agent) {
		return _positions.at(timestep).at(agent);
	}

	unsigned long n_timesteps() const { return _positions.size() - 1; }

	unsigned long n_agents() const { check(); return _positions.at(0).size(); }

	bool check() const;

	bool is_empty() const { return _positions.empty(); }

	void merge(Solution &solution);

	friend std::ostream &operator<<(std::ostream &os, const Solution &sol);
};

#endif
