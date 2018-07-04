/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Search.h:                                             *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef __SEARCH__
#define __SEARCH__

class Search {
protected:
    unsigned _min_makespan;
    unsigned _max_makespan;

public:
    Search(unsigned min_makespan, unsigned max_makespan)
            : _min_makespan(min_makespan), _max_makespan(max_makespan) {}

    virtual ~Search() = default;

    virtual bool get_initial_solved() = 0;

    virtual unsigned int get_initial_makespan() = 0;

    virtual unsigned int get_next_makespan(bool solved) = 0;

    virtual int get_successful_makespan() = 0;

    virtual bool break_test(bool solved) = 0;

    virtual bool success() = 0;

    virtual const std::string name() const = 0;

};

#endif