from os import system

for size in [4, 8, 16, 32, 64]:

	for robots in (4, 8, 16, 32):
		for obst_prob in [0.1]:
			if(robots > 0.85*size*size):
				continue
			for seed in (5, 31, 327, 616, 895):
				aux  = " --x-size=" + str(size) + " --y-size=" + str(size)
				aux += " --N-robots=" + str(robots)
				aux += " --obstacle-probability=" + str(obst_prob)
				aux += " --seed=" + str(seed)

				filename  = "grid_" + str(size) + "x" + str(size)
				filename += "_a" + str(robots)
				filename += "_o" + str(obst_prob)
				filename += "_s" + str(seed)

				aux += " --multirobot-file=../instances/" + filename  + ".cpf"
				print ("../reLOC-0.13-odaiba_037/src/gridgen_reLOC" + aux)
				system("../reLOC-0.13-odaiba_037/src/gridgen_reLOC" + aux)

for dim in (3, 9):
	for robots in (4, 8, 16, 32):
		if(robots > 0.85 * 2**dim):
			continue
		for seed in (5, 31, 327, 616, 895):
			aux  = " --dimmension=" + str(dim)
			aux += " --N-robots=" + str(robots)
			aux += " --seed=" + str(seed)
			aux += " --multirobot-file=../instances/hyper_dim" + str(dim) + "_a" + str(robots) + "_" + str(seed) + ".cpf"
			print ("../reLOC-0.13-odaiba_037/src/hypergen_reLOC " + aux)
			system("../reLOC-0.13-odaiba_037/src/hypergen_reLOC " + aux)
