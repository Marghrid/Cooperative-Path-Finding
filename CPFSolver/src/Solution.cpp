/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Solution.h"

bool Solution::check() const {
	//TODO Is there any unassigned agent for each timestep?
	for (auto &agent: _positions) {
		for (auto &agent_pos: agent.second)
			if (agent_pos < 0 || agent_pos > _instance.n_vertices())
				return false;
	}
	return true;
}

std::ostream &operator<<(std::ostream &os, const Solution &sol) {
	//FIXME The agents aren't printed ordered by ID
	if (sol.is_empty()) {
		os << "Empty solution!" << std::endl;
		return os;
	}

	os << "Solution makespan: " << sol._positions.begin()->second.size() - 1 << std::endl;

	for (unsigned timestep = 0; timestep < sol.n_timesteps(); ++timestep) {
		os << "Timestep " << timestep << ":" << std::endl;
		for (auto it = sol._positions.begin(); it != sol._positions.end(); ++it) {
			os << "\t" << it->first->id() << " @ " << it->second.at(timestep)
			   << std::endl;
		}
	}
	return os;
}

void Solution::merge(Solution &other) {
	/*FIXME: the agents in the solution are not ordered
	 * according to their ID and I am assuming that they are. */
	//std::cout << "merging" << std::endl;
	if (other.n_timesteps() != this->n_timesteps())
		throw std::runtime_error(
				"Can't merge two solutions with a different number of timesteps: " +
				std::to_string(this->n_timesteps()) + " and " +
				std::to_string(other.n_timesteps()) + ".");

	for (auto &agent_pair : other._positions) {
		for (unsigned i = 0; i < agent_pair.second.size(); ++i) {
			for (auto &local_agent_pair : this->_positions) {
				if (local_agent_pair.second[i] == agent_pair.second[i]) {
					throw std::runtime_error(
							"Couldn't merge the solutions, there is a conflict at timestep " +
							std::to_string(i) + " and agent " +
							std::to_string(agent_pair.first->id()));
				}
			}
		}
		this->_positions.insert(agent_pair);
	}
}

void Solution::add(std::shared_ptr<Agent> agent, int position, unsigned timestep) {
	/* If the agent exists in this solution, either change or reassign the value in the positions vector;
	 * Otherwise, add agent to the map, and then the value in the positions vector
	 */
	auto agent_pair = _positions.find(agent);
	if (agent_pair != _positions.end()) {
		// The agent already exists in this solution
		if (timestep == 0) {
			// If building the first timestep, everything is ok.
			if (timestep >= agent_pair->second.size())
				agent_pair->second.resize(timestep + 1);
			agent_pair->second[timestep] = position;
			return;
		}
		// Else, check if the agent made a valid move.
		int previous = agent_pair->second[timestep - 1];
		// Either the agent stayed at the same vertex
		if (previous == position) {
			if (timestep >= agent_pair->second.size())
				agent_pair->second.resize(timestep + 1);
			agent_pair->second[timestep] = position;
			return;
		}
		// Or it moved to a neighbouring vertex.
		for (auto &v: _instance.get_neighbours(previous)) {
			if (v == position) {
				if (timestep >= agent_pair->second.size())
					agent_pair->second.resize(timestep + 1);
				agent_pair->second[timestep] = position;
				return;
			}
		}
		// If it reaches here, the agent appeared in the instance.
	} else {
		// This agent did not yet exist in this solution
		std::vector<int> empty_vector;
		_positions[agent] = empty_vector;

		agent_pair = _positions.find(agent);
		if (timestep >= agent_pair->second.size())
			agent_pair->second.resize(timestep + 1);
		agent_pair->second[timestep] = position;
	}
}

void Solution::add(unsigned agentID, int position, unsigned timestep) {
	for (auto pair : _positions) {
		if (pair.first->id() == agentID) {
			this->add(pair.first, position, timestep);
			return;
		}
	}
	std::shared_ptr<Agent> new_agent = std::make_shared<Agent>(agentID);
	add(new_agent, position, timestep);
}
