/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: BinarySearch.h:                                       *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef __BINARY_SEARCH__
#define __BINARY_SEARCH__

#include "Search.h"
#include <cmath>

class BinarySearch: public Search {
private:
	int _lower_bound = _min_makespan;
	int _upper_bound = _max_makespan + 1;
	// I wonder if starting lower will be better:
	int _average = std::floor((_lower_bound + _upper_bound)/8);

public:
	BinarySearch(int min_makespan, int max_makespan) : Search(min_makespan, max_makespan) {}
	
	bool get_initial_solved()    override { return false; }
	int  get_initial_makespan()  override { return _average; }

	int  get_next_makespan(bool solved) override {
		if(solved)
			_upper_bound = _average;
		else
			_lower_bound = _average + 1;

		return _average= std::floor((_lower_bound + _upper_bound)/2);
	}
	int  get_successful_makespan() override { return _lower_bound; }

	bool break_test(bool solved) override {
		//Returns true if search should stop:
		return (_lower_bound == _upper_bound);
	}

	bool success() override { return (_lower_bound != _max_makespan+1); }
};
	
#endif
