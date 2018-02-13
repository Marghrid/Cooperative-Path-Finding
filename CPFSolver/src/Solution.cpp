/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Solution.h"
#include "Instance.h"

void Solution::add(int timestep, int agent, int position)  {
	if((unsigned)timestep+1 > _positions.size()) {
		_positions.resize(timestep+1);
	}
	if((unsigned)agent+1 > _positions.at(timestep).size()) {
		_positions.at(timestep).resize(agent+1, -1);
	}

	_positions.at(timestep).at(agent) = position;
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