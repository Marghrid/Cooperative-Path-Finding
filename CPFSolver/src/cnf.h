#ifndef __CNF__
#define __CNF__

#include <vector>
#include <map>

class Variable {
private:
	int _var_id = -1;

public:
	Variable() {}

	Variable(int var_id) {
		_var_id = var_id;
	}

	int id() { return _var_id; }

	bool operator<(const Variable& o) const {
		return this->_var_id < o._var_id;
	}
};

class Clause {
public:
	std::map<Variable, bool> _literals;

	void add_literal(Variable var, bool b = true) {
		_literals.insert( std::pair<Variable, bool> (var, b) );
	}

	int n_literals() const {
		return _literals.size();
	}
};

class Formula {
public:
	std::vector<Clause> _clauses;

	void add_clause(Clause c) {
		_clauses.push_back(c);
	}

	int n_clauses() const {
		return _clauses.size();
	}

	int n_literals() const {
		int t = 0;
		for(auto& it : _clauses)
			t += it.n_literals();
		return t;
	}
};

#endif
