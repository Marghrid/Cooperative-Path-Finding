/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: InvertedEncoder.cpp:                                *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "InvertedEncoder.h"

void InvertedEncoder::create_vars_for_makespan(int makespan) {

}

void InvertedEncoder::create_clauses_for_makespan(int makespan) {
}



void InvertedEncoder::create_goal_assumptions(Glucose::vec<Glucose::Lit> &assumptions, int makespan) {

}

Solution InvertedEncoder::get_solution(int makespan) {
    Solution sol(_instance);

    return sol;
}

void InvertedEncoder::create_clauses_for_group_makespan(Group *group, int makespan) {

}
