from os import system

for size in [4, 8, 16, 32]:
	for robots_prob in [0.1, 0.4, 0.8]:
		aux = size * size * robots_prob;
		n_robots = int(aux);
		for seed in [5, 31, 327, 616, 895]:
			aux  = " --input-file=instances/grid_" + str(size) + "x" + str(size) + "_r" + str(n_robots) + "_" + str(seed) + ".cpf"
			aux += " --output-file=solutions/grid_" + str(size) + "x" + str(size) + "_r" + str(n_robots) + "_" + str(seed) + ".out"
			output = " > solver_files/grid_" + str(size) + "x" + str(size) + "_r" + str(n_robots) + "_" + str(seed) + ".txt"
			print ("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
			system("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)

for dim in [3, 9, 12]:
	for robots_prob in [0.1, 0.4, 0.8]:
		aux = 2**dim * robots_prob
		n_robots = int(aux);
		for seed in [5, 31, 327, 616, 895]:
			aux  = " --input-file=instances/hyper_dim" + str(dim) + "_r" + str(n_robots) + "_" + str(seed) + ".cpf"
			aux  += " --output-file=solutions/hyper_dim" + str(dim) + "_r" + str(n_robots) + "_" + str(seed) + ".out"
			output = " > solver_files/hyper_dim" + str(dim) + "_r" + str(n_robots) + "_" + str(seed) + ".txt"
			print ("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
			system("reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
