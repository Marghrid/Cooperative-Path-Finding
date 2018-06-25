/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Agent.h:                                              *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __AGENT__
#define __AGENT__

#include "Graph.h"

class Agent {
private:
	unsigned _id;
	Vertex _initial_position;
	Vertex _goal_position;
public:
	explicit Agent(unsigned int aid)
			: _id(aid), _initial_position(static_cast<Vertex>(-1)),
			  _goal_position(static_cast<Vertex>(-1)) {}

	/*Agent(unsigned int aid, Vertex initial_pos, Vertex goal_pos)
			: _id(aid), _initial_position(initial_pos), _goal_position(goal_pos) {}

	Agent(unsigned int aid, Vertex initial_pos) : Agent(aid, initial_pos, static_cast<Vertex>(-1)) {}
*/
	unsigned int id() { return _id; }

	Vertex initial_position() const { return _initial_position; }

	Vertex goal_position() const { return _goal_position; }

	void set_initial_position(Vertex v) { _initial_position = v; }

	void set_goal_position(Vertex v) { _goal_position = v; }

	friend std::ostream &operator<<(std::ostream &os, const Agent &inst) {
		os << inst._id << "[" << inst._initial_position << ", " << inst._goal_position << "]";
		return os;
	}

	bool operator<(const Agent &rhs) const { return _id < rhs._id; }

	bool operator>(const Agent &rhs) const { return rhs < *this; }

	bool operator<=(const Agent &rhs) const { return !(rhs < *this); }

	bool operator>=(const Agent &rhs) const { return !(*this < rhs); }
};

#endif