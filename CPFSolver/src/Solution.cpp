/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Solution.h"
#include "Instance.h"
#include "Graph.h"

void Solution::add(int agent, int position)  {
	if((unsigned)agent+1 > _positions.at(_current_timestep).size()) {
		_positions.at(_current_timestep).resize(agent+1, -1);
	}

	if(_current_timestep == 0) {
		_positions.at(_current_timestep).at(agent) = position;
		return;
	}

	int previous = _positions.at(_current_timestep-1).at(agent);
	if(previous == position) {
		_positions.at(_current_timestep).at(agent) = position;
		return;
	}

	for(auto &v: _instance.get_neighbours(previous)) {
		if(v.id() == position) {
			_positions.at(_current_timestep).at(agent) = position;
			return;
		}
	}
}

void Solution::increment_timestep() {
	_positions.resize(_positions.size() + 1);
	_current_timestep = _positions.size()-1;
}

bool Solution::check(Instance instance) {
	for(auto &timestep: _positions) {
		if(timestep.size() != (unsigned)instance.n_agents())
			return false;
		for(auto &agent_pos: timestep)
			if(agent_pos < 0 || agent_pos > instance.n_vertices())
				return false;
	}
	return true;
}

std::ostream& operator<<(std::ostream& os, const Solution& sol) {
	os << "Solution makespan:" << sol._positions.size()-1 << std::endl;
	for(unsigned timestep = 0; timestep < sol._positions.size(); ++timestep) {
		os << "Timestep " << timestep << ":" << std::endl;
		for(unsigned agent = 0; agent < sol._positions.at(timestep).size(); ++agent) {
			os << "\t" << agent << " # " << sol._positions.at(timestep).at(agent) << std::endl;
		}
	}
	return os;
}