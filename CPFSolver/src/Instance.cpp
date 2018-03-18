/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Instance.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Instance.h"

#include <queue>
#include <set>

void Instance::add_agent(int a_id) {
    if ((size_t) a_id >= _agents.size()) {
        for (int i = _agents.size(); i <= a_id; ++i) {
            Agent a(i);
            _agents.push_back(a);
            _vertex_starts_empty.push_back(true);
        }
    }
}

void Instance::set_start_empty(int a_id, bool b) {
    while ((size_t) a_id >= _vertex_starts_empty.size()) {
        _vertex_starts_empty.push_back(false);
    }
    _vertex_starts_empty[a_id] = b;
}

void Instance::set_end_empty(int a_id, bool b) {
    while ((size_t) a_id >= _vertex_ends_empty.size()) {
        _vertex_ends_empty.push_back(false);
    }
    _vertex_ends_empty[a_id] = b;
}


std::ostream &operator<<(std::ostream &os, const Instance &inst) {
    os << "---- CPF INSTANCE ----" << std::endl;
    os << "Environment: " << std::endl;
    os << "N vertices: " << inst.n_vertices() << std::endl;
    os << "N edges: " << inst.n_edges() << std::endl;
    for (auto &e: inst._environment.bidirectional_edges()) {
        os << e << std::endl;
    }
    os << std::endl;
    os << "N agents: " << inst._agents.size() << std::endl;
    for (auto &a: inst._agents) {
        os << a << std::endl;
    }
    return os;
}

bool Instance::check() const {
    return true;
}

int Instance::min_makespan() const {
    /*int max_distance = 0;
    int current_distance = 0;

    //std::cout << "Agents' distances to their goals:" << std::endl;

    for (Agent a : _agents) {
        current_distance = _environment.distance(a.initial_position(), a.goal_position());
        //std::cout << "Agent " << a.id() << " distance: " << current_distance << std::endl;
        //std::cout << "Agent " << a.id() << " initial pos: " << a.initial_position() << std::endl;

        if (current_distance > max_distance)
            max_distance = current_distance;
    }

    return max_distance;*/
    return 0;
}
