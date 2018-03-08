/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Graph.cpp:                                            *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Graph.h"

std::ostream& operator<<(std::ostream& os, const Edge& e) {
    os << "(" << e._start << ", " << e._end << ")";
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

std::vector<Vertex> Graph::get_neighbours(Vertex v) const {
    std::vector<Vertex> neighbours;
    for(auto& a: _adjacencies[v]) {
        neighbours.push_back(a.end());
    }
    return neighbours;
}

void Graph::add_edge(Vertex start, Vertex end, bool directed) {
    if(start > _n_vertices-1) {
        add_vertex(start);

    }
    if(end > _n_vertices-1) {
        add_vertex(end);
    }

    //std::cout << "Adding (" << start << ", " << end << ")" << std::endl;
    Edge e1(start, end);
    _adjacencies.at(start).push_back(e1);

    if(!directed) {
        Edge e2(end, start);
        _adjacencies.at(end).push_back(e2);
    }
    ++_n_edges;
}

void Graph::add_vertex(int id) {
    if (id >= _n_vertices) {
        _adjacencies.resize(id+1);
        _n_vertices = id+1;
    }
}
