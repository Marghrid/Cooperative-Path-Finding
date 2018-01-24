#include "graph.h"

std::ostream& operator<<(std::ostream& os, const Vertex& v) {
	os << v.id;
	return os;
}

std::ostream& operator<<(std::ostream& os, const Edge& e) {
	os << "(" << e._start.id << ", " << e._end.id << ")";
	return os;
}

std::vector<Edge> Graph::edges() const {
	std::vector<Edge> ret;
	for(auto& ee: _edges) {
		for(auto& e: ee) {
			ret.push_back(e);
		}
	}
	return ret;
}

void Graph::add_edge(Vertex start, Vertex end, int weight, bool directed) {
	if(start.id > _n_vertices-1) {
		add_vertex(start.id);

	}
	if(end.id > _n_vertices-1) {
		add_vertex(end.id);
	}

	Edge e(start, end, weight);
	_edges.at(start.id).push_back(e);

	if(!directed) {
		Edge e(end, start, weight);
		_edges.at(end.id).push_back(e);
	}

	++_n_edges;
}

void Graph::add_edge(int start_id, int end_id, int weight, bool directed) {
	add_edge(Vertex(start_id), Vertex(end_id), weight, directed);
}

void Graph::add_vertex(int id) {
	if (id >= _n_vertices) {
		_edges.resize(id+1);
		_n_vertices = id+1;
	}
}
