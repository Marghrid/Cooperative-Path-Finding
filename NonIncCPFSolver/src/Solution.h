/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Solution.h:                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __SOLUTION__
#define __SOLUTION__

#include "Instance.h"

#include <vector>

class Solution {
private:
    std::vector<std::vector<int>> _positions;
    // Outer vector index represents a timestep.
    // Inner vector index represents an agent
    // int on the inner vertex represents the agent's
    //  position (vertex#) on each timestep.

    Instance _instance;
    int _current_timestep = -1;

public:

    Solution(Instance& instance) : _instance(instance) {};

    void add(int agent, int position);

    void increment_timestep();

    int get_position(int timestep, int agent) {
        return _positions.at(timestep).at(agent);
    }

    unsigned long n_timesteps() const { return _positions.size() - 1; }

    bool check() const;

    bool is_empty() const { return _positions.empty(); }

    friend std::ostream &operator<<(std::ostream &os, const Solution &sol);
};

#endif
