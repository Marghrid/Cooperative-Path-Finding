#ifndef __SAT_UNSAT_SEARCH__
#define __SAT_UNSAT_SEARCH__

class SAT_UNSATSearch: public Search {
private:
	int _count = _max_makespan;

public:
	SAT_UNSATSearch(int max_makespan) : Search(max_makespan) {}

	bool get_initial_solved()    override { return true; }
	int  get_initial_makespan()  override { return _max_makespan; }

	int  get_next_makespan()     override { return --_count; }
	int  get_previous_makespan() override { return _count+2; }

	bool break_test(int makespan, bool solved) override {
		//Returns true if search should stop:
		return !solved || makespan == -1 ;
	}

	bool success() override { return _count != -1; }
};
	
#endif