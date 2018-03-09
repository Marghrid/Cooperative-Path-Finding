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
	int _min_makespan;
	int _max_makespan;

public:
	Search(int min_makespan, int max_makespan) 
	: _min_makespan(min_makespan), _max_makespan(max_makespan) {}
	virtual ~Search() {}

	virtual bool get_initial_solved() = 0;
	virtual int  get_initial_makespan() = 0;
	virtual int  get_next_makespan(bool solved) = 0;
	virtual int  get_successful_makespan() = 0;
	virtual bool break_test(bool solved) = 0;

	virtual bool success() = 0;
};

#endif