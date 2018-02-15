from os import system

for filename in ["subway"]:
	aux  = " --input-file=../handmade_instances/"  + filename + ".cpf"
	aux += " --output-file=../handmade_instances/" + filename + ".out"
	aux += " --makespan-limit=64"

output = " > ../solver_files/" + filename + ".txt 2>&1"
print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)

for size in (4, 8, 16, 32):
	for robots_prob in (0.2, 0.3, 0.4, 0.6):
		for obst_prob in (0.1, 0.2, 0.3):
			n_robots = int(size * size * robots_prob);
			makespan_limit = size * size * n_robots
			for seed in (5, 31, 327, 616, 895):

				filename  = "grid_" + str(size).zfill(2) + "x" + str(size).zfill(2)
				filename += "_a" + str(n_robots).zfill(4)
				filename += "_o" + str(obst_prob)
				filename += "_s" + str(seed).zfill(3)

				aux  = " --input-file=../instances/"  + filename + ".cpf"
				aux += " --output-file=../instances/" + filename + ".out"
				aux += " --makespan-limit=" + str(makespan_limit)
				aux += " --strategy=binary"

				output = " > ../solver_files/" + filename + ".txt 2>&1"
				print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
				system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)

for dim in (3, 9):
	for robots_prob in (0.1, 0.2, 0.4):
		aux = 2**dim * robots_prob
		n_robots = int(aux);
		makespan_limit = 2**dim * n_robots

		for seed in (5, 31, 327, 616, 895):
			aux = " --input-file=../instances/hyper_dim" + str(dim) + "_a" + str(n_robots).zfill(3) + "_" + str(seed).zfill(3) + ".cpf"
			aux += " --output-file=../solutions/hyper_dim" + str(dim) + "_a" + str(n_robots).zfill(3) + "_" + str(seed).zfill(3) + ".out"
			aux += " --makespan-limit=" + str(makespan_limit)
			aux += " --strategy=binary"

			output = " > ../solver_files/hyper_dim" + str(dim) + "_a" + str(n_robots) + "_" + str(seed) + ".txt 2>&1"
			print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
			system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux + output)
