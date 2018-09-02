/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Graph.cpp:                                            *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Graph.h"

#include <queue>

std::ostream &operator<<(std::ostream &os, const Edge &e) {
	os << "(" << e._start << ", " << e._end << ")";
	return os;
}

std::vector<Edge> Graph::edges() const {
	std::vector<Edge> ret;
	for (auto &ee: _adjacencies) {
		for (auto &e: ee) {
			ret.push_back(e);
		}
	}
	return ret;
}

std::vector<Edge> Graph::bidirectional_edges() const {
	std::vector<Edge> ret;
	for (auto &ee: _adjacencies) {
		for (auto &e: ee) {
			if (e.start() < e.end())
				ret.push_back(e);
		}
	}
	return ret;
}

std::vector<Vertex> Graph::get_neighbours(Vertex v) const {
	std::vector<Vertex> neighbours;
	for (auto &a: _adjacencies[v]) {
		neighbours.push_back(a.end());
	}
	return neighbours;
}

void Graph::add_edge(Vertex start, Vertex end, bool directed) {
	if (start > _n_vertices - 1) {
		add_vertex(start);

	}
	if (end > _n_vertices - 1) {
		add_vertex(end);
	}

	Edge e1(start, end);
	_adjacencies.at(start).push_back(e1);

	if (!directed) {
		Edge e2(end, start);
		_adjacencies.at(end).push_back(e2);
	}
	++_n_edges;
}

void Graph::add_vertex(Vertex id) {
	if (id >= _n_vertices) {
		_adjacencies.resize(id + 1);
		_n_vertices = id + 1;
	}
}

unsigned int Graph::distance(Vertex v1, Vertex v2) const {
	std::vector<bool> visited(_n_vertices, false);
	std::queue<std::pair<Vertex, unsigned> > open;

	if (v1 == v2) return 0;

	visited[v1] = true;
	open.push(std::pair<Vertex, int>(v1, 0));

	while (!open.empty()) {
		Vertex v = open.front().first;
		unsigned depth = open.front().second;
		open.pop();

		for (Vertex n : get_neighbours(v)) {
			if (n == v2) return depth + 1;

			if (!visited[n]) {
				visited[n] = true;
				open.push(std::pair<Vertex, int>(n, depth + 1));
			}
		}
	}
	throw std::runtime_error("Couldn't compute distance between vertices " + std::to_string(v1) +
	                         " and " + std::to_string(v2) + ".");
}
