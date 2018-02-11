#include "graph.h"

std::ostream& operator<<(std::ostream& os, const Vertex& v) {
	os << v.id();
	return os;
}

std::ostream& operator<<(std::ostream& os, const Edge& e) {
	os << "(" << e._start.id() << ", " << e._end.id() << ")";
	return os;
}

std::vector<Edge> Graph::edges() const {
	std::vector<Edge> ret;
	for(auto& ee: _adjacencies) {
		for(auto& e: ee) {
			ret.push_back(e);
		}
	}
	return ret;
}

std::vector<Edge> Graph::bidirectional_edges() const {
	std::vector<Edge> ret;
	for(auto& ee: _adjacencies) {
		for(auto& e: ee) {
			if(e.start() < e.end())
				ret.push_back(e);
		}
	}
	return ret;
}

std::vector<Vertex> Graph::get_neighbours(int v_id) const {
	std::vector<Vertex> neighbours;
	for(auto& a: _adjacencies[v_id]) {
		neighbours.push_back(a.end());
	}
	return neighbours;
}

void Graph::add_edge(Vertex start, Vertex end, bool directed) {
	if(start.id() > _n_vertices-1) {
		add_vertex(start.id());

	}
	if(end.id() > _n_vertices-1) {
		add_vertex(end.id());
	}

	//std::cout << "Adding (" << start.id() << ", " << end.id() << ")" << std::endl;
	Edge e1(start, end);
	_adjacencies.at(start.id()).push_back(e1);

	if(!directed) {
		Edge e2(end, start);
		_adjacencies.at(end.id()).push_back(e2);
	}

	++_n_edges;
}

void Graph::add_edge(int start_id, int end_id, bool directed) {
	add_edge(Vertex(start_id), Vertex(end_id), directed);
}

void Graph::add_vertex(int id) {
	if (id >= _n_vertices) {
		_adjacencies.resize(id+1);
		_n_vertices = id+1;
	}
}
