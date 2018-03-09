/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Graph.h:                                              *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __GRAPH__
#define __GRAPH__

#include <iostream>
#include <vector>
#include <list>

typedef int Vertex;

class Edge {
private:
    Vertex _start;
    Vertex _end;

public:
    Edge(Vertex start, Vertex end)
    : _start(start), _end(end) { }

    Vertex start() const { return _start; }
    Vertex end()   const { return _end; }

    friend std::ostream& operator<<(std::ostream& os, const Edge& e);
};

class Graph {
private:
    std::vector< std::list<Edge> > _adjacencies;
    int _n_vertices = 0;
    int _n_edges = 0;

public:
    Graph() {}

    int n_vertices() const { return _n_vertices; }

    int n_edges() const { return _n_edges; }

    std::vector<Edge> edges() const;

    std::vector<Edge> bidirectional_edges() const;

    std::vector<Vertex> get_neighbours(Vertex v) const;

    void add_edge(Vertex start, Vertex end, bool directed = false);

    void add_vertex(int id);

    int distance(Vertex v1, Vertex v2) const;
};

#endif
