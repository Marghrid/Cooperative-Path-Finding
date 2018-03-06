import constants


def generate_command(instance_name, encoding="simplified", search="UNSAT-SAT", verbosity=0):
	command  = "../CPFSolver/bin/CPFSolver"
	command += " -i ../handmade_instances/" + instance_name + ".cpf"
	command += " -o " + constants.solutions_dir  + instance_name + "_" + search + ".out"
	command += " -s " + constants.stat_files_dir + instance_name + "_" + search + ".txt"
	command += " -e " + encoding
	command += " -search " + search
	command += " -v " + str(verbosity)

	return command