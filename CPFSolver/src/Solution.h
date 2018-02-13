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
	// Outer vector index represents a timestep.
	// Inner vector index represents an agent
	// int on the inner vertex represents the agent's
	//  position (vertex#) on each timestep.

public:
	void add(int timestep, int agent, int position);

	int get_position(int timestep, int agent) {
		return _positions.at(timestep).at(agent);
	}

	int n_timesteps() { return _positions.size(); }

	bool check(Instance instance);

	bool is_empty() { return _positions.size() == 0; }

	friend std::ostream& operator<<(std::ostream& os, const Solution& sol);
};

#endif
