/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Instance.cpp:                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 #include "Instance.h"

#include <vector>
#include <queue>
#include <set>
#include <iostream>

void Instance::add_agent(int a_id) {
	if ((size_t)a_id >= _agents.size()) {
		for (int i = _agents.size(); i <= a_id; ++i) {
			Agent a(i);
			_agents.push_back(a);
			_vertex_starts_empty.push_back(true);
		}
	}
}

void Instance::set_start_empty(int a_id, bool b) {
	while ((size_t)a_id >= _vertex_starts_empty.size()) {
		_vertex_starts_empty.push_back(false);
	}
	_vertex_starts_empty[a_id] = b;
}

void Instance::set_end_empty(int a_id, bool b) {
	while ((size_t)a_id >= _vertex_ends_empty.size()) {
		_vertex_ends_empty.push_back(false);
	}
	_vertex_ends_empty[a_id] = b;
}


std::ostream& operator<<(std::ostream& os, const Instance& inst) {
	os << "---- CPF INSTANCE ----" << std::endl;
	os << "Environment: " << std::endl;
	os << "N vertices: "  << inst.n_vertices() << std::endl;
	os << "N edges: "     << inst.n_edges()    << std::endl;
	for (auto& e: inst._environment.bidirectional_edges()) {
		os << e << std::endl;
	}
	os << std::endl;
	os << "N agents: " << inst._agents.size() << std::endl;
	for (auto& a: inst._agents) {
		os << a << std::endl;
	}
	return os;
}

bool Instance::check() const {
	return true;
}

int Instance::min_makespan() const {
	std::vector<int> distances(n_agents(), 0);
	std::vector<int> found(n_agents(), 0);

	std::cout << "dist  " << distances.size() << std::endl;
	std::cout << "agent " << n_agents()  << std::endl;

	std::cout << "found " << found.size() << std::endl;

	std::set<Vertex> this_level;
	std::set<Vertex> next_level;

	this_level.insert(0);

	int current_depth = 0;
	bool end = false;

	while (!end) {

		end = true;
		for (Agent a : _agents) {
			for (Vertex v : this_level) {
				if (a.initial_position() == v && found[a.id()] == 0) {
					distances[a.id()] = current_depth;
					found[a.id()] = 1;
					break;
				}

				if (a.goal_position() == v && found[a.id()] == 1) {
					distances[a.id()] = current_depth - distances[a.id()];
					found[a.id()] = 2;
					break;
				}

				std::cout << "  v " << v << " id " << a.id() << "  found " << found[a.id()] << "? " << (found[a.id()] < 2) << std::endl;

				if (found[a.id()] < 2) {
					end = false;
				}
				
			}
			
		}

		for (Vertex v : this_level) {
			for (Vertex n : _environment.get_neighbours(v) ) {
				next_level.insert(n);
			}
		}
		this_level = next_level;
		next_level.clear();
		++current_depth;
		std::cout << "depth " << current_depth << std::endl;
		//std::cout << "after clear" << std::endl;
	}

	//std::cout << "Agents' distances to their goals:" << std::endl;

	
	for (int i =0; i < n_agents(); ++i) {
		std::cout << distances[i] << " ";
	}

	//std::cout << "after for" << std::endl;

	
	// Start in vertex 0, BFS

	return 0;
}
