/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Instance.h                                            *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef _INSTANCE_
#define _INSTANCE_

#include "Graph.h"
#include "Agent.h"

#include <memory>
#include <iostream>

class Instance {
private:
	Graph _environment;

	/* In the end, agents are sorted by decreasing distance to goal: _agents[0] is farther away */
	std::vector<std::shared_ptr<Agent>> _agents;

	/* The indices are vertex IDs */
	std::vector<bool> _vertex_starts_empty;
	std::vector<bool> _vertex_ends_empty;

public:
	// Constructors:
	Instance(Graph &environment, std::vector<std::shared_ptr<Agent>> &agents)
			: _environment(environment), _agents(agents) {}

	explicit Instance(Graph &environment) : _environment(environment),
	                                        _vertex_starts_empty(environment.n_vertices(), true),
	                                        _vertex_ends_empty(environment.n_vertices(), true) {}

	// Agents:
	unsigned n_agents() const { return static_cast<unsigned int>(_agents.size()); }

	Agent &agent(unsigned a_id) const;

	std::vector<std::shared_ptr<Agent>> &agents() { return _agents; }

	void add_agent(Agent &a_id);

	void set_agent_initial_position(unsigned aid, Vertex initial_pos) {
		_agents.at(aid)->set_initial_position(initial_pos);
	}

	void set_agent_goal_position(unsigned aid, Vertex goal_pos) {
		_agents.at(aid)->set_goal_position(goal_pos);
	}

	// Environment:
	unsigned n_vertices() const { return _environment.n_vertices(); }

	unsigned n_edges() const { return _environment.n_edges(); }

	std::vector<Vertex> get_neighbours(Vertex v_id) const {
		return _environment.get_neighbours(v_id);
	}

	std::vector<Edge> edges() const { return _environment.edges(); }

	std::vector<Edge> bidirectional_edges() const { return _environment.bidirectional_edges(); }

	void add_vertex(Vertex v_id) { _environment.add_vertex(v_id); }

	void add_edge(Vertex start_id, Vertex end_id) { _environment.add_edge(start_id, end_id); }

	bool starts_empty(int v_id) const { return _vertex_starts_empty[v_id]; }

	bool ends_empty(int v_id) const { return _vertex_ends_empty[v_id]; }

	//Instance:
	int max_makespan() const { return n_vertices() * 4; }

	int min_makespan() const;

	bool check();

	friend std::ostream &operator<<(std::ostream &os, const Instance &inst);

	unsigned int distance(Vertex v1, Vertex v2) const;
};

#endif
