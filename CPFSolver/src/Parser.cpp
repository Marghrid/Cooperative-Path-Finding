#include "Parser.h"
#include "Instance.h"

#include <iostream>
#include <sstream>
#include <string>

Instance Parser::parse() {
    Instance inst;

    int v_id;
    int initial_a_id;
    // int unknown_a_id;
    int goal_a_id;
    int largest_agent_id = 0;

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

    if(aux1.at(0) != 'V') {
        std::cerr << "Error in Parsing" << std::endl;
        // exception
    }

    //Read vertices and agents:
    while(std::getline(_instance_file_stream, aux1)) {

        if(aux1.at(0) == 'E') break;

        std::stringstream ss(aux1);

        std::getline(ss, aux1, '(');
        std::getline(ss, aux2, ':');
        std::getline(ss, aux3, '[');
        std::getline(ss, aux4, ':');
        std::getline(ss, aux5, ':');
        std::getline(ss, aux6, ']');

        v_id         = std::stoi(aux2);
        initial_a_id = std::stoi(aux4);
        // unknown_a_id = std::stoi(aux5);
        goal_a_id    = std::stoi(aux6);

        inst.add_vertex(v_id);

        if(initial_a_id > 0) {
            if(initial_a_id > largest_agent_id) {
                inst.add_agent(initial_a_id-1);
                largest_agent_id = initial_a_id;
            }
            inst.set_agent_initial_position(initial_a_id-1, v_id);
            inst.set_start_empty(v_id, false);
        } else {
            inst.set_start_empty(v_id, true);
        }

        if(goal_a_id > 0) {
            if(goal_a_id > largest_agent_id) {
                inst.add_agent(goal_a_id-1);
                largest_agent_id = goal_a_id;
            }
            inst.set_agent_goal_position(goal_a_id-1, v_id);
        }
    }

    //read edges:
    while(std::getline(_instance_file_stream, aux1)) {

        std::stringstream ss(aux1);
        std::getline(ss, aux1, '{');
        std::getline(ss, aux2, ',');
        std::getline(ss, aux3, '}');
        std::getline(ss, aux4, '(');
        std::getline(ss, aux5, ')');

        e_start_id = std::stoi(aux2);
        e_end_id   = std::stoi(aux3);
        // e_unknown  = std::stoi(aux5);
        inst.add_edge(e_start_id, e_end_id);

    }
    return inst;
}