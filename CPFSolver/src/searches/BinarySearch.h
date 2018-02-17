#ifndef __BINARY_SEARCH__
#define __BINARY_SEARCH__

#include "Search.h"
#include <cmath>

class BinarySearch: public Search {
private:
	int _lower_bound = 0;
	int _upper_bound = _max_makespan + 1;
	// I wonder if starting lower will be better:
	int _average = std::floor((_lower_bound + _upper_bound)/4);

public:
	BinarySearch(int max_makespan) : Search(max_makespan) {}

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