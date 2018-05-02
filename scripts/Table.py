from Instance import Instance
from Solution import Solution
from Cell import Cell
from Line import Line

class Table:
	def __init__(self):
		self.lines = []
		self.instances = []

		self.diff_encodings = []
		self.diff_searches  = []

	def add_instance(self, instance):
		self.instances += [instance]
		new_line = Line(instance.type, instance.n_agents, self)
		for line in self.lines:
			if line == new_line:
				return

		self.lines += [new_line]

	def add_solution(self, solution):
		if solution.encoding not in self.diff_encodings:
			self.diff_encodings += [solution.encoding]
			for line in self.lines:
				line.add_encoding(solution.encoding)

		if solution.search not in self.diff_searches:
			self.diff_searches += [solution.search]
			for line in self.lines:
				line.add_search(solution.search)

		for line in self.lines:
			if line.belongs(solution):
				line.add_solution(solution)
				return

		raise ValueError("Solution of an unknown type/n_agents")

	def get_n_instances_of_type(self, itype, n_agents):
		count = 0
		for instance in self.instances:
			if instance.type == itype and \
				instance.n_agents == n_agents:
				count += 1
		return count

	def print_to_file(self, filename):
		file = open(filename, "w+")

		self.diff_encodings.sort()
		self.diff_searches.sort(reverse = True)
		self.lines.sort()

		# First line:
		file.write("instance_type, n_agents")
		for encoding in self.diff_encodings:
			for search in self.diff_searches:
				file.write(", " + encoding + "_" + search)
		file.write("\n")

		# Remaining lines:
		for line in self.lines:
			file.write(str(line) + "\n")

		file.close()
