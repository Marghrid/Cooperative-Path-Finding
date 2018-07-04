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
    SAT_UNSATSearch(unsigned min_makespan, unsigned max_makespan) : Search(min_makespan, max_makespan) {}

    bool get_initial_solved() override { return true; }

    unsigned int get_initial_makespan() override { return _max_makespan; }

    unsigned get_next_makespan(bool solved) override { return --_count; }

    int get_successful_makespan() override { return _count + 2; }

    bool break_test(bool solved) override {
        //Returns true if search should stop:
        return !solved || _count == (int)_min_makespan - 1;
    }

    bool success() override { return _count != -1; }

    const std::string name() const override { return "SAT-UNSAT"; }

};

#endif