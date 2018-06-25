/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Parser.cpp:                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Parser.h"

#include <sstream>

Instance Parser::parse() {
	Graph environment;

	Vertex v_id;
	unsigned agent_starting_here;
	// int unknown_a_id;
	unsigned agent_finishing_here;

	int e_start_id;
	int e_end_id;
	// int e_unknown;

	std::string aux1;
	std::string aux2;
	std::string aux3;
	std::string aux4;
	std::string aux5;
	std::string aux6;

	std::getline(_instance_file_stream, aux1);

	if (aux1.empty()) {
		std::cerr << "File empty!" << std::endl;
		exit(-1);
	}

	if (aux1.at(0) != 'V') {
		std::cerr << "Error in Parsing" << std::endl;
		// exception
	}

	//Read vertices and _agents:
	while (std::getline(_instance_file_stream, aux1)) {
		if (aux1.at(0) == 'E') break;

		std::stringstream ss(aux1);

		std::getline(ss, aux1, '(');
		std::getline(ss, aux2, ':');
		std::getline(ss, aux3, '[');
		std::getline(ss, aux4, ':');
		std::getline(ss, aux5, ':');
		std::getline(ss, aux6, ']');

		v_id = std::stoul(aux2);
		agent_starting_here = std::stoul(aux4);
		// unknown_a_id = std::stoi(aux5);
		agent_finishing_here = std::stoul(aux6);

		environment.add_vertex(v_id);

		if (agent_starting_here > 0) {
			set_agent_start(--agent_starting_here, v_id);
		}

		if (agent_finishing_here > 0) {
			set_agent_goal(--agent_finishing_here, v_id);
		}
	}

	//read edges:
	while (std::getline(_instance_file_stream, aux1)) {

		std::stringstream ss(aux1);
		std::getline(ss, aux1, '{');
		std::getline(ss, aux2, ',');
		std::getline(ss, aux3, '}');
		std::getline(ss, aux4, '(');
		std::getline(ss, aux5, ')');

		e_start_id = std::stoi(aux2);
		e_end_id = std::stoi(aux3);
		// e_unknown  = std::stoi(aux5);
		environment.add_edge(e_start_id, e_end_id);

	}

	Instance instance(environment);
	for (std::shared_ptr<Agent> a : _agents) {
		instance.add_agent(*a);
	}

	if (!instance.check())
		throw std::runtime_error("Error parsing instance");

	return instance;
}

void Parser::set_agent_start(unsigned int a_id, Vertex v_id) {
	std::cout << "agent " << a_id << " starts at " << v_id << std::endl;
	for (std::shared_ptr<Agent> a : _agents) {
		if (a->id() == a_id) {
			a->set_initial_position(v_id);
			return;
		}
	}
	std::shared_ptr<Agent> new_agent = std::make_shared<Agent>(a_id);
	new_agent->set_initial_position(v_id);
	_agents.push_back(new_agent);
}

void Parser::set_agent_goal(unsigned int a_id, Vertex v_id) {
	std::cout << "agent " << a_id << " ends at " << v_id << std::endl;
	for (std::shared_ptr<Agent> a : _agents) {
		if (a->id() == a_id) {
			a->set_goal_position(v_id);
			return;
		}
	}
	std::shared_ptr<Agent> new_agent = std::make_shared<Agent>(a_id);
	new_agent->set_goal_position(v_id);
	_agents.push_back(new_agent);
}

