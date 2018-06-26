#include "Group.h"

std::ostream &operator<<(std::ostream &os, const Group &group) {
	os << "agents { ";
	for (Agent *a: group.agents)
		os << *a << " ";
	os <<"}";
	return os;
}
