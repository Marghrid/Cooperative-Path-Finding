import os
import sys

import get_filename
import constants
import Table

table_filename = constants.table_filename
if len(sys.argv) > 1:
	table_filename = sys.argv[1]


table = Table.Table()

## Create instances ##

_, _, files = next(os.walk(constants.instances_dir), (None, None, []))

instance_type = ""
instance_n_agents = -1
instance_o_prob = 0.1  # This is a default
instance_seed = -1

for filename in files:
	filename = filename.replace(".cpf", "")
	instance_type = get_filename.get_instance_type(filename)

	if instance_type == "":
		continue  # This is not a valid instance file

	instance_n_agents = get_filename.get_n_agents(filename)

	if instance_n_agents == -1:
		continue # This is not a valid instance file

	#  instance_o_prob = get_filename.get_o_prob(filename)

	instance_seed = get_filename.get_seed(filename)

	if instance_seed == -1:
		continue # This is not a valid instance file

	instance = Table.Instance(instance_type, instance_n_agents, instance_o_prob, instance_seed, filename)
	table.add_instance(instance)


## Create solutions ##

_, _, files = next(os.walk(constants.stat_files_dir), (None, None, []) )


for filename in files:
	solution_instance = None
	solution_encoding = ""
	solution_search   = ""
	solution_CPUtime  = -1
	solution_status   = -1
	solution_makespan = -1

	for inst in table.instances:
		if filename.startswith(inst.filename):
			solution_instance = inst

	if solution_instance == None:
		continue # This is not a valid solution file

	content = open(constants.stat_files_dir + "/" + filename)
	for line in content:
		if "encoding:" in line:
			#print(line.split(" ")[-1])
			solution_encoding = line.split(" ")[-1][:-1]

		elif "search:" in line:
			solution_search = line.split(" ")[-1][:-1]

		elif "CPU time:" in line:
			solution_CPUtime = float(line.split(" ")[-2])

		elif "(SAT)" in line:
			solution_status = 1
			solution_makespan = int(line.split(" ")[-1])

		elif "(UNSAT)" in line:
			solution_status = 2

		elif "Out of memory" in line:
			solution_status = 3

	if solution_encoding == "" or \
		solution_search  == "" or \
		solution_CPUtime == -1 or \
		solution_status  == -1:

		continue # This is not a valid solution file

	solution = Table.Solution(solution_instance, solution_encoding, solution_search, solution_CPUtime, solution_status, solution_makespan)
	table.add_solution(solution)

table.print_to_file(table_filename)
