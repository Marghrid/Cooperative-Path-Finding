#include <iostream>
#include "simp/SimpSolver.h"

int main()
{
	Glucose::SimpSolver solver;
	solver.newVar();
	solver.newVar();
	solver.addClause(Glucose::mkLit(0));
	solver.addClause(Glucose::mkLit(1));
	bool solved = solver.solve(Glucose::mkLit(1, true), Glucose::mkLit(0, true));

	if(solved) {
		std::cout << "solved" << std::endl;
		return 0;
	}

	std::cout << "\n\n";
	while (solver.conflict.size() != 0) {
		std::cout << solver.conflict.last().x << ", ";
		solver.conflict.pop();
	}

	std::cout << "\nend." << std::endl;

	return 0;
}