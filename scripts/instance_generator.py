from os import system

for size in (4, 8, 16):
	for robots_prob in (0.125, 0.25, 0.375, 0.5):
		for obst_prob in [0.1]:
			n_robots = int(size * size * robots_prob);
			for seed in (5, 31, 132, 327, 518, 616, 783, 895):
				aux  = " --x-size=" + str(size) + " --y-size=" + str(size)
				aux += " --N-robots=" + str(n_robots)
				aux += " --obstacle-probability=" + str(obst_prob)
				aux += " --seed=" + str(seed)

				filename  = "grid_" + str(size).zfill(2) + "x" + str(size).zfill(2)
				filename += "_a" + str(n_robots).zfill(3)
				#filename += "_o" + str(obst_prob)
				filename += "_s" + str(seed).zfill(3)

				aux += " --multirobot-file=../instances/" + filename  + ".cpf"
				print ("../reLOC-0.13-odaiba_037/src/gridgen_reLOC" + aux)
				system("../reLOC-0.13-odaiba_037/src/gridgen_reLOC" + aux)

for dim in (3, 6, 9):
	for robots_prob in (0.125, 0.25, 0.375, 0.5):
		aux = 2**dim * robots_prob
		n_robots = int(aux);
		for seed in (5, 31, 327, 616, 895):
			aux  = " --dimmension=" + str(dim)
			aux += " --N-robots=" + str(n_robots)
			aux += " --seed=" + str(seed)
			aux += " --multirobot-file=../instances/hyper_dim" + str(dim) + "_a" + str(n_robots).zfill(3) + "_" + str(seed).zfill(3) + ".cpf"
			print ("../reLOC-0.13-odaiba_037/src/hypergen_reLOC " + aux)
			system("../reLOC-0.13-odaiba_037/src/hypergen_reLOC " + aux)
