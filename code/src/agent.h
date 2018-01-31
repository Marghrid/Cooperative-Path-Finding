#ifndef __AGENT__
#define __AGENT__

#include "graph.h"

class Agent {
private:
	int _id;
	Vertex _initial_position;
	Vertex _goal_position;
public:
	Agent(int aid)
	: _initial_position(-1), _goal_position(-1){
		_id = aid;
	}

	Agent(int aid, Vertex initial_pos, Vertex goal_pos)
	: _initial_position(initial_pos), _goal_position(goal_pos) {
		_id = aid;
	}

	Agent(int aid, int initial_pos_id, int goal_pos_id)
	: _initial_position(initial_pos_id), _goal_position(goal_pos_id) {
		_id = aid;
	}

	Agent(int aid, int initial_pos_id)
	: _initial_position(initial_pos_id), _goal_position(-1) {
		_id = aid;
	}

	Vertex initial_position() { return _initial_position; }
	Vertex goal_position() { return _goal_position; }

	void set_initial_position(Vertex v) {
		_initial_position = v;
	}

	void set_goal_position(Vertex v) {
		_goal_position = v;
	}

	void set_initial_position(int vid) {
		set_initial_position(Vertex(vid));
	}

	void set_goal_position(int vid) {
		set_goal_position(Vertex(vid));
	}

	friend std::ostream& operator<<(std::ostream& os, const Agent& inst) {
		os << inst._id << "[" << inst._initial_position << ", " << inst._goal_position << "]";
		return os;
	}
};


#endif