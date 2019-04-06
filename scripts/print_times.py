import os

import get_filename
import constants

_, _, files = next(os.walk(constants.instances_dir), (None, None, []))
print(str(len(files)) + " files.")

for filename in files:
	filename = filename.replace(".cpf", "")
	#print(filename)
	instance_type = get_filename.get_instance_type(filename)

	if instance_type == "":
		continue  # This is not a valid instance file

	instance_n_agents = get_filename.get_n_agents(filename)

	if instance_n_agents == -1:
		continue  # This is not a valid instance file

	#  instance_o_prob = get_filename.get_o_prob(filename)

	instance_seed = get_filename.get_seed(filename)

	if instance_seed == -1:
		continue  # This is not a valid instance file


	#print(constants.stat_files_dir + "/" + filename + "_UNSAT-SAT.txt")
	try:
		content = open(constants.stat_files_dir + "/" + filename + "_UNSAT-SAT.txt")
		solvedID = True
	except OSError:
		solvedID = False

	#print(constants.stat_files_dir + "/" + filename + "_UNSAT-SAT.txt")
	try:
		noIDcontent = open("../noIDstats/" + filename + "_UNSAT-SAT.txt")
		solved_noID = True
	except OSError:
		solved_noID = False

	CPUtime = -1
	noIDCPUtime = -1

	if solvedID and solved_noID and ( instance_type == "grid 08x08" and instance_n_agents <= 7 or instance_type == "grid 16x16" and instance_n_agents <= 26 or instance_type == "grid 32x32" and instance_n_agents <= 11):

		print(instance_type + ", " + str(instance_n_agents) , end=', ')

		for line in content:
			if "CPU time:" in line:
				CPUtime = float(line.split(" ")[-2])

		if CPUtime == -1:
			raise ValueError("This is bad!")

		if True:
			print("ID, " + str(CPUtime), end=', ')


		for line in noIDcontent:
			if "CPU time:" in line:
				noIDCPUtime = float(line.split(" ")[-2])

		if noIDCPUtime == -1:
			raise ValueError("This is bad!")

		if True:
			print("noID, " + str(noIDCPUtime))