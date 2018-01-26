from os import system

for size in [ 64]:
	makespan_limit = size * size *2
	for robots in (4, 8, 16, 32):
		for obst_prob in [0.1]:
			if(robots > 0.85*size*size):
				continue
			for seed in (5, 31, 327, 616, 895):

				filename  = "grid_" + str(size) + "x" + str(size)
				filename += "_a" + str(robots)
				filename += "_o" + str(obst_prob)
				filename += "_s" + str(seed)

				aux  = " --input-file=simple_instances/"  + filename + ".cpf"
				aux += " --output-file=simple_instances/" + filename + ".out"
				aux += " --makespan-limit=" + str(makespan_limit)

				output = " > solver_files/" + filename + ".txt 2>&1"
				print ("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
				system("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)

for dim in (3, 9):
	for robots in (4, 8, 16, 32):
		if(robots > 0.85 * 2**dim):
			continue
		for seed in (5, 31, 327, 616, 895):
			aux  = " --input-file=simple_instances/hyper_dim"  + str(dim) + "_a" + str(robots) + "_" + str(seed) + ".cpf"
			aux += " --output-file=simple_instances/hyper_dim" + str(dim) + "_a" + str(robots) + "_" + str(seed) + ".out"
			aux += " --makespan-limit=" + str(makespan_limit)
			output = " > solver_files/hyper_dim" + str(dim) + "_a" + str(robots) + "_" + str(seed) + ".txt 2>&1"
			print ("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
			system("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
