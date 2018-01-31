#include <iostream>
#include "instance.h"
#include "graph.h"
#include "parser.h"
#include "simplified_solver.h"

int main() {

	Graph env1;

	env1.add_edge(0, 1);
	env1.add_edge(0, 2);
	env1.add_edge(1, 3);
	env1.add_edge(2, 3);

	Agent a1(0, 0, 3);
	Agent a2(1, 3, 0);

	std::vector<Agent> agents1;
	agents1.push_back(a1);
	agents1.push_back(a2);

	Instance inst1(env1, agents1);


	Graph env2;

	env2.add_edge(0, 1);
	env2.add_edge(1, 2);
	env2.add_edge(3, 4);
	env2.add_edge(4, 5);
	env2.add_edge(6, 7);
	env2.add_edge(7, 8);

	env2.add_edge(0, 3);
	env2.add_edge(3, 6);
	env2.add_edge(1, 4);
	env2.add_edge(4, 7);
	env2.add_edge(2, 5);
	env2.add_edge(5, 8);

	Agent a3(0, 0, 5);
	Agent a4(1, 2, 3);
	Agent a5(2, 6, 2);

	std::vector<Agent> agents2;
	agents2.push_back(a3);
	agents2.push_back(a4);
	agents2.push_back(a5);

	Instance inst2(env2, agents2);


	std::cout << inst1 << std::endl;
	std::cout << "------\n" << std::endl;

	Simplified_solver s(inst2, 5);
	s.convert();
}