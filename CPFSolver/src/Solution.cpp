/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Solution.h"

void Solution::add(int agent, int position) {
    if ((unsigned) agent + 1 > _positions.at(_current_timestep).size()) {
        _positions.at(_current_timestep).resize(agent + 1, -1);
    }

    // If building the first timestep, everything is ok.
    // Maybe check if coherent with initial arrangement on instance?
    if (_current_timestep == 0) {
        _positions.at(_current_timestep).at(agent) = position;
        return;
    }

    // Else, check if the agent made a valid move.
    int previous = _positions.at(_current_timestep - 1).at(agent);
    // Either the agent stayed at the same vertex
    if (previous == position) {
        _positions.at(_current_timestep).at(agent) = position;
        return;
    }

    // Or it moved to a neighbour of that vertex.
    for (auto &v: _instance.get_neighbours(previous)) {
        if (v == position) {
            _positions.at(_current_timestep).at(agent) = position;
            return;
        }
    }

    // If this agent was left unassigned, something went wrong.
    if( _positions.at(_current_timestep).at(agent) == -1) {
        throw std::runtime_error("An agent was left unassigned while building" +
            "solution. Something went wrong while solving.")
    }
}

void Solution::increment_timestep() {
    _positions.resize(_positions.size() + 1);
    _current_timestep = _positions.size() - 1;
}

bool Solution::check() const {
    // Is there any unassigned agent for each timestep?
    for (auto &timestep: _positions) {
        if (timestep.size() != (unsigned) _instance.n_agents())
            return false;
        for (auto &agent_pos: timestep)
            if (agent_pos < 0 || agent_pos > _instance.n_vertices())
                return false;
    }
    return true;
}

std::ostream &operator<<(std::ostream &os, const Solution &sol) {
    if(sol.is_empty()) {
        os << "Empty solution!" << std::endl;
        return os;
    }
    
    os << "Solution makespan:" << sol._positions.size() - 1 << std::endl;
    for (unsigned timestep = 0; timestep < sol._positions.size(); ++timestep) {
        os << "Timestep " << timestep << ":" << std::endl;
        for (unsigned agent = 0; agent < sol._positions.at(timestep).size(); ++agent) {
            os << "\t" << agent << " # " << sol._positions.at(timestep).at(agent) << std::endl;
        }
    }
    return os;
}