from os import system

for size in [4, 8, 16, 32]:
	for robots_prob in [0.1, 0.4, 0.8]:
		aux = size * size * robots_prob;
		n_robots = int(aux);
		for seed in [5, 31, 327, 616, 895]:
			aux = " --walk --x-size=" + str(size) + " --y-size=" + str(size) + " --N-robots=" + str(n_robots) + " --obstacle-probability=0.1 --seed=" + str(seed) + " --multirobot-file=try/grid_" + str(size) + "x" + str(size) + "_r" + str(n_robots) + "_" + str(seed) + ".cpf"
			print("./gridgen_reLOC" + aux)
			system("./gridgen_reLOC" + aux)
			
