#ifndef _INSTANCE_
#define _INSTANCE_

#include "graph.h"
#include "agent.h"
#include <iostream>

class Instance {
private:
	Graph _environment;
	std::vector<Agent> _agents;

public:
	Instance(Graph environment, std::vector<Agent> agents) {
		_environment = environment;
		_agents      = agents;
	}

	Instance(Graph environment) { _environment = environment; }

	Instance() {}

	int n_vertices() const {
		return _environment.n_vertices();
	}

	int n_edges() const { return _environment.n_edges(); }

	int n_agents() const { return _agents.size(); }

	Agent agent(int id) const { return _agents.at(id); }

	void add_vertex(int v_id) { _environment.add_vertex(v_id); }

	void add_edge(int start_id, int end_id) { _environment.add_edge(start_id, end_id); }
 	
 	void add_agent(int a_id);

	void add_agent(Agent a) {

	}

	void set_agent_initial_position(int aid, int initial_pos_id) {
		_agents.at(aid).set_initial_position(initial_pos_id);
	}

	void set_agent_initial_position(int aid, Vertex initial_pos) {
		_agents.at(aid).set_initial_position(initial_pos);
	}

	void set_agent_goal_position(int aid, int goal_pos_id) {
		_agents.at(aid).set_goal_position(goal_pos_id);
	}

	void set_agent_goal_position(int aid, Vertex goal_pos) {
		_agents.at(aid).set_goal_position(goal_pos);
	}

	bool check() const;


	friend std::ostream& operator<<(std::ostream& os, const Instance& inst);
};

#endif