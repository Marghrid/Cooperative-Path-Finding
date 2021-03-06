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
	// std::vector<int> represents the agent's position in each timestep
	std::map<std::shared_ptr<Agent>, std::vector<int>> _positions;
	Instance _instance;

public:

	explicit Solution(Instance &instance) : _instance(instance) {};

	void add(std::shared_ptr<Agent> agent, int position, unsigned timestep);

	void add(unsigned agentID, int position, unsigned timestep);

	int get_position(std::shared_ptr<Agent> agent, unsigned timestep) {
		for (auto agent_pair : _positions) {
			if (agent_pair.first->id() == agent->id())
				return agent_pair.second[timestep];
		}
		return -1;
	}

	unsigned long n_timesteps() const { return _positions.begin()->second.size(); }

	unsigned long n_agents() const { check(); return _positions.size(); }

	bool check() const;

	bool is_empty() const { return _positions.empty(); }

	void merge(Solution &solution);

	Instance &instance() {
		return _instance;
	}

	friend std::ostream &operator<<(std::ostream &os, const Solution &sol);
};

#endif
