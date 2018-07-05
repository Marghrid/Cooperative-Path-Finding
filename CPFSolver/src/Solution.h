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
#include <map>

class Solution {
private:
	//std::vector<std::vector<int>> _positions;
	// Outer vector element represents a timestep.
	// Inner vector element represents an agent.
	// int on the inner vector represents the agent's
	//  position (vertex#) on each timestep.

	std::map<std::shared_ptr<Agent>, std::vector<int>> _positions;
	Instance _instance;

public:

	explicit Solution(Instance &instance) : _instance(instance) {};

	void add(std::shared_ptr<Agent> agent, int position, unsigned timestep);

	void add(unsigned agentID, int position, unsigned timestep);

	int get_position(std::shared_ptr<Agent> agent, unsigned timestep) {
		//return _positions.at(agent).second.at(timestep);
		return _positions.find(agent)->second.at(timestep);
	}

	unsigned long n_timesteps() const { return _positions.begin()->second.size(); }

	unsigned long n_agents() const { check(); return _positions.size(); }

	bool check() const;

	bool is_empty() const { return _positions.empty(); }

	void merge(Solution &solution);

	friend std::ostream &operator<<(std::ostream &os, const Solution &sol);
};

#endif
