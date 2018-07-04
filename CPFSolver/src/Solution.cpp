/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Solution.h"

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

void Solution::increment_timestep() {
	_positions.resize(_positions.size() + 1);
	_current_timestep = _positions.size() - 1;
}

bool Solution::check() const {
	// Is there any unassigned agent for each timestep?
	for (auto &timestep: _positions) {
		//if (timestep.size() != (unsigned) _instance.n_agents())
		//	return false;
		for (auto &agent_pos: timestep)
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

	os << "Solution makespan: " << sol._positions.size() - 1 << std::endl;
	for (unsigned timestep = 0; timestep < sol._positions.size(); ++timestep) {
		os << "Timestep " << timestep << ":" << std::endl;
		for (unsigned agent = 0; agent < sol._positions.at(timestep).size(); ++agent) {
			os << "\t" << agent << " # " << sol._positions.at(timestep).at(agent) << std::endl;
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

	for (unsigned i = 0; i <= this->n_timesteps(); ++i) {
		for (unsigned k = 0; k < other.n_agents(); ++k) {
			for (const int &vertexID : this->_positions[i]) {
				if (vertexID == other.get_position(i, k))
					throw std::runtime_error(
							"Couldn't merge the solutions, there is a conflict at timestep " +
							std::to_string(i) + " and agent at index " + std::to_string(k));
			}

			this->_positions[i].push_back(other.get_position(i, k));

			//FIXME in the end, the agents must be ordered by id
		}
	}
}
