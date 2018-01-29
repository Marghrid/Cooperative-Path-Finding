#include <iostream>
#include "instance.h"
#include "graph.h"
#include "parser.h"
#include "simplified_encoder.h"

int main() {
	Graph env;

	Parser parser("simple_grid.cpf");

	env.add_edge(0, 1, 1);
	env.add_edge(0, 2, 1);
	env.add_edge(1, 3, 1);
	env.add_edge(2, 3, 1);

	std::vector<Agent> agents;
	Agent a1(0, 0, 2);
	Agent a2(1, 2, 3);

	agents.push_back(a1);
	agents.push_back(a2);

	Instance i(env, agents);

	Instance i2 = parser.parse();

	//std::cout << i;

	std::cout << "------\n" << std::endl;

	std::cout << i2 << std::endl;
}