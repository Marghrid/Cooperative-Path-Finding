/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                   CPF Solver    2018                        *
 *                   Margarida Ferreira                        *
 *                                                             *
 * File: Group.cpp                                             *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 #include "Group.h"

std::ostream &operator<<(std::ostream &os, const Group &group) {
	os << "{ ";
	for (const std::shared_ptr<Agent> &a: group.agents)
		os << *a << " ";
	os <<"}";
	return os;
}
