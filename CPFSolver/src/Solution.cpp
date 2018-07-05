/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Solution.h"

/*
void Solution::add(int agentID, int position) {
	if ((unsigned) agentID + 1 > _positions.at(_current_timestep).size()) {
		//_positions.at(_current_timestep).resize(agentID + 1, -1);
		for (std::vector<int> &pos : _positions) {
			pos.resize(agentID + 1, -1);
		}
	}

	// If building the first timestep, everything is ok.
	// Maybe check if coherent with initial arrangement on instance?
	if (_current_timestep == 0) {
		_positions.at(_current_timestep).at(agentID) = position;
		return;
	}
	// Else, check if the agent made a valid move.
	int previous = _positions.at(_current_timestep - 1).at(agentID);
	// Either the agent stayed at the same vertex
	if (previous == position) {
		_positions.at(_current_timestep).at(agentID) = position;
		return;
	}
	// Or it moved to a neighbouring vertex.
	for (auto &v: _instance.get_neighbours(previous)) {
		if (v == position) {
			_positions.at(_current_timestep).at(agentID) = position;
			return;
		}
	}
}
*/


bool Solution::check() const {
	//TODO Is there any unassigned agent for each timestep?
	for (auto &agent: _positions) {
		//if (timestep.size() != (unsigned) _instance.n_agents())
		//	return false;
		for (auto &agent_pos: agent.second)
			if (agent_pos < 0 || agent_pos > _instance.n_vertices())
				return false;
	}
	//TODO order agents by ID, and make sure that the IDs are sequential and unique

	/*for (unsigned t = 0; t < _positions.size(); ++t) {
		std::sort(_positions[t].begin(), _positions[t].end(),
		          [this](const std::shared_ptr<Agent> &a1, const std::shared_ptr<Agent> &a2) {
			          return a1->id() < a2->id();
		          });
	}*/


	return true;
}

std::ostream &operator<<(std::ostream &os, const Solution &sol) {
	if (sol.is_empty()) {
		os << "Empty solution!" << std::endl;
		return os;
	}

	os << "Solution makespan: " << sol._positions.begin()->second.size() - 1 << std::endl;

	auto blaaaaaa = sol.n_timesteps();
	for (unsigned timestep = 0; timestep < blaaaaaa; ++timestep) {
		os << "Timestep " << timestep << ":" << std::endl;
		for (auto &agent_pair : sol._positions) {
			os << "\t" << agent_pair.first->id() << " # " << agent_pair.second.at(timestep)
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
		// Make sure there are no collisions
		for (unsigned i = 0; i < agent_pair.second.size(); ++i) {
			for (auto &local_agent_pair : this->_positions) {
				if (local_agent_pair.second[i] == agent_pair.second[i]) {
					throw std::runtime_error("Collision in merge");
				}
			}
		}
		this->_positions.insert(agent_pair);
	}

	//FIXME in the end, the agents must be ordered by id

	/*for (unsigned i = 0; i <= this->n_timesteps(); ++i) {
		for (unsigned k = 0; k < other.n_agents(); ++k) {
			for (const int &vertexID : this->_positions[i]) {
				if (vertexID == other.get_position(k, i))
					throw std::runtime_error(
							"Couldn't merge the solutions, there is a conflict at timestep " +
							std::to_string(i) + " and agent at index " + std::to_string(k));
			}

			this->_positions[i].push_back(other.get_position(k, i));


		}
	}*/
}

void Solution::add(std::shared_ptr<Agent> agent, int position, unsigned timestep) {
	/* If the agent exists in this solution, either change or reassign the value in the positions vector;
	 * Otherwise, add agent to the map, and then the value in the positions vector
	 */
	auto search = _positions.find(agent);
	if (search != _positions.end()) {
		// The agent already exists in this solution
		if (timestep == 0) {
			// If building the first timestep, everything is ok.
			if (timestep >= search->second.size())
				search->second.resize(timestep + 1);
			search->second[timestep] = position;
			return;
		}
		// Else, check if the agent made a valid move.
		int previous = search->second[timestep - 1];
		// Either the agent stayed at the same vertex
		if (previous == position) {
			if (timestep >= search->second.size())
				search->second.resize(timestep + 1);
			search->second[timestep] = position;
			return;
		}
		// Or it moved to a neighbouring vertex.
		for (auto &v: _instance.get_neighbours(previous)) {
			if (v == position) {
				if (timestep >= search->second.size())
					search->second.resize(timestep + 1);
				search->second[timestep] = position;
				return;
			}
		}
		throw std::runtime_error("Could not add agent to solution");
	} else {
		// This agent did not yet exist in this solution
		std::vector<int> empty_vector;
		_positions.insert(std::pair<std::shared_ptr<Agent>, std::vector<int>>(agent, empty_vector));

		auto agent_pair = _positions.find(agent);
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
