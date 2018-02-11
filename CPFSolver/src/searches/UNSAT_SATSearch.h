#ifndef __UNSAT_SAT_SEARCH__
#define __UNSAT_SAT_SEARCH__

class UNSAT_SATSearch: public Search {
private:
	int _count = 0;

public:
	UNSAT_SATSearch(int max_makespan) : Search(max_makespan) {}

	bool get_initial_solved()    override { return false; }
	int  get_initial_makespan()  override { return 0; }

	int  get_next_makespan()     override { return ++_count; }
	int  get_previous_makespan() override { return _count-1; }

	bool break_test(int makespan, bool solved) override {
		//Returns true if search should stop:
		return solved || makespan == _max_makespan + 1 ;
	}

	bool success() override { return _count != _max_makespan + 1; }
};

#endif