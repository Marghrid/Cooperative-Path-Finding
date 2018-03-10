/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: SAT_UNSATSearch.h:                                    *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __SAT_UNSAT_SEARCH__
#define __SAT_UNSAT_SEARCH__

#include "Search.h"

class SAT_UNSATSearch : public Search {
private:
    int _count = _max_makespan;

public:
    SAT_UNSATSearch(int min_makespan, int max_makespan) : Search(min_makespan, max_makespan) {}

    bool get_initial_solved() override { return true; }

    int get_initial_makespan() override { return _max_makespan; }

    int get_next_makespan(bool solved) override { return --_count; }

    int get_successful_makespan() override { return _count + 2; }

    bool break_test(bool solved) override {
        //Returns true if search should stop:
        return !solved || _count == _min_makespan - 1;
    }

    bool success() override { return _count != -1; }
};

#endif