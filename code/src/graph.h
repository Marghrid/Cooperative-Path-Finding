#ifndef __GRAPH__
#define __GRAPH__

#include <iostream>
#include <vector>
#include <list>

class Vertex {
public:
	int id;

	Vertex(int vid) { id = vid; }

	friend std::ostream& operator<<(std::ostream& os, const Vertex& v);
};

class Edge {
private:
	Vertex _start;
	Vertex _end;
	int _weight;

public:
	Edge(Vertex start, Vertex end, int weight = 0)
	: _start(start), _end(end) { _weight = weight; }

	Edge(int start_id, int end_id, int weight = 0)
	: _start(start_id), _end(end_id) { _weight = weight; }

	friend std::ostream& operator<<(std::ostream& os, const Edge& e);
};

class Graph {
private:
	std::vector< std::list<Edge> > _edges;
	int _n_vertices = 0;
	int _n_edges = 0;

public:
	Graph() {}

	int n_vertices() const { return _n_vertices; }

	int n_edges() const { return _n_edges; }

	std::vector<Edge> edges() const;

	void add_edge(Vertex start, Vertex end, int weight = 0, bool directed = false);

	void add_edge(int start_id, int end_id, int weight = 0, bool directed = false);

	void add_vertex(int id);
};

#endif
