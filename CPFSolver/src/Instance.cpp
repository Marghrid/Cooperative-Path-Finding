/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Instance.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Instance.h"

#include <queue>
#include <set>

void Instance::add_agent(Agent &a) {
	_agents.push_back(std::make_shared<Agent>(a));
	_vertex_starts_empty[a.initial_position()] = false;
	_vertex_ends_empty[a.goal_position()] = false;
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
		os << *a << std::endl;
	}
	return os;
}

bool Instance::check() {
	// Are there as many non-empty vertices at the beginning and the end as agents?
	int count = 0;
	for (bool val : _vertex_starts_empty)
		if (!val) ++count;
	if (count != _agents.size()) return false;

	count = 0;
	for (bool val : _vertex_ends_empty)
		if (!val) ++count;
	if (count != _agents.size()) return false;

	// Are the IDs unique and sequential?
	bool ok = false;
	for (int i = 0; i < _agents.size(); ++i) {
		for (const std::shared_ptr<Agent> &a : _agents) {
			if (a->id() == i) ok = true;
		}
	}
	if (!ok) return false;

	std::sort(_agents.begin(), _agents.end(), [this](const std::shared_ptr<Agent> &a1, const std::shared_ptr<Agent> &a2) {
		return distance(a1->initial_position(), a1->goal_position())
		       > distance(a2->initial_position(), a2->goal_position());
	});

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
	return distance(_agents[0]->initial_position(), _agents[0]->goal_position());
}

Agent &Instance::agent(unsigned a_id) const {
	for (const std::shared_ptr<Agent> &a : _agents) {
		if (a->id() == a_id) return *a;
	}
	throw std::runtime_error(std::string("Nonexistent agent #" + std::to_string(a_id)));
}

unsigned Instance::distance(Vertex v1, Vertex v2) const {
	return _environment.distance(v1, v2);
}