import os
import statistics
import constants
import sys


def get_cell_values(l):
	return (sum(l)/len(l) , len(l))

def count_total_instances(instance_type, n_agents):
	_, _, files = next(os.walk(constants.instances_dir), (None, None, []))
	count = 0

	for file in files:
		if file.startswith(instance_type + "_a" + str(n_agents).zfill(3)):
			count += 1

	return count


class Instance:
	def __init__(self, instance_type):
		#print( "new inst of type " + instance_type)
		self._instance_type = instance_type
		self._n_vertices = 0
		self._n_agents = 0
		self._encoding = ""
		self._search = ""
		self._time = 0
		self._status = -1
		# 1: solved SAT
		# 2: solved UNSAT
		# 3: out of memory
		# 4: timed out

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
			#print("new type: " + instance._instance_type)
			self.diff_types += [instance._instance_type]

		if instance._n_agents not in self.diff_n_agents:
			#print("new n_agents: " + str(instance._n_agents))
			self.diff_n_agents += [instance._n_agents]

		if instance._encoding not in self.diff_encodings:
			#print("new encoding: " + instance._encoding)
			self.diff_encodings += [instance._encoding]

		if instance._search not in self.diff_searches:
			#print("new search: " + instance._search)
			self.diff_searches += [instance._search]

	def to_file(self, file):
		self.diff_types = sorted(self.diff_types)
		self.diff_n_agents = sorted(self.diff_n_agents)

		#print(self.diff_types)
		#print(self.diff_n_agents)

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
				total_instances = count_total_instances(instance_type, n_agents)

				if(total_instances == 0):
					continue
					
				for instance in self.instances:
					if instance._instance_type == instance_type and \
						instance._n_agents == n_agents and\
						(instance._status == 1 or instance._status == 2):

						category_instances += [instance]


				print(instance_type + ", " + str(n_agents) + " n:" + str(total_instances) + ": " +str(category_instances))
				line += instance_type + ", " + str(n_agents) + ", "
				#print(line)
				#print(len(category_instances))
				#print(self.diff_encodings)
				#print(self.diff_searches)
				for encoding in self.diff_encodings:
					for search in self.diff_searches:
						times = []
						for instance in category_instances:
							if instance._encoding == encoding and \
									instance._search == search:

								times += [instance._time]

						if len(times) > 0:
							#line += "%.4f" % statistics.median(times) + " s, "
							line += str(len(times)) + " of " + str(total_instances) + ", "
						else:
							line += "- , 0, "
						print(line + str(times))
				if not line.endswith("- , 0, - , 0, "):
					file.write(line[:-2] + "\n")





# -------------------------   SCRIPT    ------------------------- #

directory = constants.stat_files_dir 
table_file = constants.table_filename
if len(sys.argv) > 1:
	table_file = sys.argv[1]

_, _, files = next(os.walk(directory), (None, None, []))

table = Table()

for filename in files:
	if filename.startswith("grid"):
		instance_type = filename.split("_")[0] + "_" + filename.split("_")[1]
		instance = Instance(instance_type)

	elif filename.startswith("hypergrid") :
		instance_type = filename.split("_")[0]
		instance = Instance(instance_type)

	elif filename == __file__ or filename == "table.csv":
		continue

	else:
		#print(filename + " of unknown file type.")
		continue

	content = open(directory + "/" + filename)
	for line in content:
		if "vertices:" in line:
			instance._n_vertices = int(line.split(" ")[-1])

		elif "agents:" in line:
			#print(line.split(" ")[-1])
			instance._n_agents = int(line.split(" ")[-1])

		elif "encoding:" in line:
			#print(line.split(" ")[-1])
			instance._encoding = line.split(" ")[-1][:-1]

		elif "search:" in line:
			#print(line.split(" ")[-1])
			instance._search = line.split(" ")[-1][:-1]

		elif "CPU time:" in line:
			#print(line.split(" ")[-2])
			instance._time = float(line.split(" ")[-2])
		elif "(SAT)" in line:
			instance._status = 1

		elif "(UNSAT)" in line:
			instance._status = 2

		elif "Out of memory" in line:
			instance._status = 3

	if instance._n_vertices != 0 and \
		instance._n_agents != 0 and \
		instance._encoding != "" and \
		instance._search != "" and \
		instance._time != 0 and \
		instance._status != -1:
		table.add_instance(instance)

outfile = open(table_file, "w+")
table.to_file(outfile)
