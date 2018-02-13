from os import system

for size in (4, 8, 16, 32, 64):
	makespan_limit = size * size *2
	for robots_prob in (0.2, 0.3, 0.4):
		for obst_prob in (0.1, 0.2, 0.3, 0.4):
			n_robots = int(size * size * robots_prob);
			for seed in (5, 31, 327, 616, 895):

				filename  = "grid_" + str(size) + "x" + str(size)
				filename += "_a" + str(n_robots)
				filename += "_o" + str(obst_prob)
				filename += "_s" + str(seed)

				aux  = " -i ../M_instances/"  + filename + ".cpf"
				aux += " -o ../M_instances/" + filename + ".out"
				aux += " -v 0"

				output = ""
				print ("../CPFSolver/bin/a.out" + aux)
				system("../CPFSolver/bin/a.out" + aux)

#for dim in (3, 9):
#	makespan_limit = 2**dim * 2
#	for robots_prob in (0.1, 0.2, 0.4):
#		aux = 2**dim * robots_prob
#		n_robots = int(aux);
#		for seed in (5, 31, 327, 616, 895):
#			aux = " --input-file=../instances/hyper_dim" + str(dim) + "_a" + str(n_robots) + "_" + str(seed) + ".cpf"
#			aux += " --output-file=../solutions/hyper_dim" + str(dim) + "_a" + str(n_robots) + "_" + str(seed) + ".out"
#			aux += " --makespan-limit=" + str(makespan_limit)
#			aux += " --strategy=binary"
#
#			output = " > ../solver_files/hyper_dim" + str(dim) + "_a" + str(n_robots) + "_" + str(seed) + ".txt 2>&1"
#			print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
#			system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
