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
    int _id;
    Vertex _initial_position;
    Vertex _goal_position;
public:
    Agent(int aid)
    : _id(aid), _initial_position(-1), _goal_position(-1) { }

    Agent(int aid, Vertex initial_pos, Vertex goal_pos)
    : _id(aid), _initial_position(initial_pos), _goal_position(goal_pos) { }

    Agent(int aid, Vertex initial_pos) : Agent(aid, initial_pos, -1) { }

    int    id() { return _id; }
    Vertex initial_position() { return _initial_position; }
    Vertex goal_position() { return _goal_position; }

    void set_initial_position(Vertex v) {
        _initial_position = v;
    }

    void set_goal_position(Vertex v) {
        _goal_position = v;
    }

    friend std::ostream& operator<<(std::ostream& os, const Agent& inst) {
        os << inst._id << "[" << inst._initial_position << ", " << inst._goal_position << "]";
        return os;
    }
};

#endif