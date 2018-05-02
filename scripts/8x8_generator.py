import get_filename
import get_command
import subprocess
import random

timeout = 600  # 10 minutes

for n_robots in range(2, 37, 2):
	for obst_prob in [0.1]:
		if(n_robots < 4):
			continue
		random.seed(377)
		seeds = random.sample(range(1, 1000), 100)
		for seed in seeds:

			filename = get_filename.get_grid_filename(8, 8, n_robots, obst_prob, seed)
			command = get_command.get_generate_8grid_command(8, 8, n_robots, obst_prob, seed, filename)

			print(command)
			subprocess.run(args=command, timeout=timeout)
