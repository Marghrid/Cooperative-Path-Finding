import constants


def get_solve_command(instance_name, encoding="simplified", search="UNSAT-SAT", verbosity=0, timeout = -1):
	command = ["../CPFSolver/bin/CPFSolver"]
	command += ["-i"] + [constants.instances_dir + instance_name + ".cpf"]
	command += ["-o"] + [constants.solutions_dir + instance_name + "_" + search + ".out"]
	command += ["-s"] + [constants.stat_files_dir + instance_name + "_" + search + ".txt"]
	command += ["-e"] + [encoding]
	command += ["-search"] + [search]
	command += ["-v "] + [str(verbosity)]
	command += ["-t "] + [str(timeout)]

	return command


def get_generate_grid_command(x_size, y_size, n_robots, obst_prob, seed, filename):
	command = ["../reLOC-0.13-odaiba_037/src/gridgen_reLOC"]
	command += ["--x-size=" + str(x_size)]
	command += ["--y-size=" + str(y_size)]
	command += ["--N-robots=" + str(n_robots)]
	command += ["--obstacle-probability=" + str(obst_prob)]
	command += ["--seed=" + str(seed)]
	command += ["--multirobot-file=../instances/" + filename + ".cpf"]

	return command


def get_generate_hyper_command(dim, n_robots, seed, filename):
	command = ["../reLOC-0.13-odaiba_037/src/hypergen_reLOC"]
	command += ["--dimmension=" + str(dim)]
	command += ["--N-robots=" + str(n_robots)]
	command += ["--seed=" + str(seed)]
	command += ["--multirobot-file=../instances/" + filename + ".cpf"]

	return command
