from os import walk
import sys

def get_cell_values(l):
	return (sum(l)/len(l) , len(l))

class Instance:
	def __init__(self, instance_type):
		print( "new inst of type " + instance_type)
		self._instance_type = instance_type
		self._n_vertices = 0
		self._n_agents = 0
		self._encoding = ""
		self._search = ""
		self._time = 0

class Table:
	def __init__(self):
		self.instances = []

		self.diff_types = []
		self.diff_n_agents = []
		self.diff_encodings = []
		self.diff_searches = []

	def add_instance(self, instance):
		self.instances += [instance]

		if instance._instance_type not in self.diff_types:
			print("new type: " + instance._instance_type)
			self.diff_types += [instance._instance_type]

		if instance._n_agents not in self.diff_n_agents:
			print("new n_agents: " + str(instance._n_agents))
			self.diff_n_agents += [instance._n_agents]

		if instance._encoding not in self.diff_encodings:
			self.diff_encodings += [instance._encoding]

		if instance._search not in self.diff_searches:
			self.diff_searches += [instance._search]

	def to_file(self, file):
		self.diff_types = sorted(self.diff_types)
		self.diff_n_agents = sorted(self.diff_n_agents)

		print(self.diff_types)
		print(self.diff_n_agents)

		header_line = "type of instance, \\#agents, "

		for e in self.diff_encodings:
			for s in self.diff_searches:
				header_line += e + "-" + s + " avg time, "
				header_line += e + "-" + s + " \\#solved, "

		header_line = header_line[0:-2]

		file.write(header_line + "\n")

		for instance_type in self.diff_types:
			for n_agents in self.diff_n_agents:
				line = ""
				category_instances = []
				for instance in self.instances:
					if instance._instance_type == instance_type and \
						instance._n_agents == n_agents:

						category_instances += [instance]

				line += instance_type + ", " + str(n_agents) + ", "
				cell = []
				for encoding in self.diff_encodings:
					for search in self.diff_searches:
						for instance in category_instances:
							if	instance._encoding == encoding and \
								instance._search == search:

								cell += [instance._time]

						if(len(cell) > 0):
							line += "%.4f" % (sum(cell)/len(cell)) + ", " + str(len(cell)) + ", "
						else:
							line = ""
				if(line != ""):
					file.write(line[:-2] + "\n")





# -------------------------   SCRIPT    ------------------------- #

directory = "."

if(len(sys.argv) > 0):
	directory = sys.argv[1]

print(directory)

_, _, files = next(walk(directory), (None, None, []))

table = Table()

for filename in files:
	if filename.startswith("grid"):
		instance_type = filename.split("_")[1]
		instance_type = "grid" + instance_type
		instance = Instance(instance_type)

	elif filename.startswith("hyper") :
		instance_type = filename.split("_")[1]
		instance_type = "hyper" + instance_type
		instance = Instance(instance_type)

	elif filename == __file__ or filename == "table.csv":
		pass

	else:
		print(filename + " of unknown file type.")
		break

	content = open(directory + "/" + filename)
	for line in content:
		print("line: " + line)
		if line.startswith("  vertices: "):
			instance._n_vertices = int(line.split(" ")[-1])

		elif line.startswith("  agents: "):
			print(line.split(" ")[-1])
			instance._n_agents = int(line.split(" ")[-1])

		elif line.startswith("  encoding: "):
			print(line.split(" ")[-1])
			instance._encoding = line.split(" ")[-1][:-1]

		elif line.startswith("  search: "):
			print(line.split(" ")[-1])
			instance._search = line.split(" ")[-1][:-1]

		if line.startswith("  Solving: "):
			print(line.split(" ")[-2])
			instance._time = float(line.split(" ")[-2])

	table.add_instance(instance)

outfile = open("table.csv", "w+")
table.to_file(outfile)
