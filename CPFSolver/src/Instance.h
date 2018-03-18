/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Instance.cpp                                          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef _INSTANCE_
#define _INSTANCE_

#include "Graph.h"
#include "Agent.h"

#include <iostream>

class Instance {
private:
    Graph _environment;
    std::vector<Agent> _agents;
    std::vector<bool> _vertex_starts_empty;
    std::vector<bool> _vertex_ends_empty;

public:
    Instance(Graph &environment, std::vector<Agent> &agents)
            : _environment(environment), _agents(agents) {}

    Instance(Graph &environment) : _environment(environment) {}

    Instance() = default;

    int n_vertices() const {
        return _environment.n_vertices();
    }

    int n_edges() const { return _environment.n_edges(); }

    unsigned long n_agents() const { return _agents.size(); }

    Agent agent(int id) const { return _agents.at(id); }

    std::vector<Vertex> get_neighbours(int v_id) const {
        return _environment.get_neighbours(v_id);
    }

    std::vector<Edge> edges() const { return _environment.edges(); }

    std::vector<Edge> bidirectional_edges() const { return _environment.bidirectional_edges(); }

    void add_vertex(int v_id) { _environment.add_vertex(v_id); }

    void add_edge(int start_id, int end_id) { _environment.add_edge(start_id, end_id); }

    void add_agent(int a_id);

    void set_start_empty(int v_id, bool b);

    bool starts_empty(int v_id) const { return _vertex_starts_empty[v_id]; }

    void set_end_empty(int v_id, bool b);

    bool ends_empty(int v_id) const { return _vertex_ends_empty[v_id]; }

    void set_agent_initial_position(int aid, Vertex initial_pos) {
        _agents.at(aid).set_initial_position(initial_pos);
    }

    void set_agent_goal_position(int aid, Vertex goal_pos) {
        _agents.at(aid).set_goal_position(goal_pos);
    }

    int max_makespan() const { return n_vertices() * 4; }

    int min_makespan() const;

    bool check() const;

    friend std::ostream &operator<<(std::ostream &os, const Instance &inst);
};

#endif
