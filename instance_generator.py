from os import system

for size in [4, 8, 16, 32]:
	for robots_prob in [0.1, 0.4, 0.8]:
		aux = size * size * robots_prob;
		n_robots = int(aux);
		for seed in [5, 31, 327, 616, 895]:
			aux  = " --x-size=" + str(size) + " --y-size=" + str(size)
			aux += " --N-robots=" + str(n_robots)
			aux += " --obstacle-probability=0.1 --seed=" + str(seed)
			aux += " --multirobot-file=instances/grid_" + str(size) + "x" + str(size) + "_r" + str(n_robots) + "_" + str(seed) + ".cpf"
			print ("reLOC-0.13-odaiba_037/src/gridgen_reLOC" + aux)
			system("reLOC-0.13-odaiba_037/src/gridgen_reLOC" + aux)

for dim in [3, 9, 12]:
	for robots_prob in [0.1, 0.4, 0.8]:
		aux = 2**dim * robots_prob
		n_robots = int(aux);
		for seed in [5, 31, 327, 616, 895]:
			aux  = " --dimmension=" + str(dim)
			aux += " --N-robots=" + str(n_robots)
			aux += "--seed=" + str(seed)
			aux += " --multirobot-file=instances/hyper_dim" + str(dim) + "_r" + str(n_robots) + "_" + str(seed) + ".cpf"
			print ("reLOC-0.13-odaiba_037/src/hypergen_reLOC " + aux)
			system("reLOC-0.13-odaiba_037/src/hypergen_reLOC " + aux)
