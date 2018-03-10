import get_filename
import get_command
import subprocess
import random

timeout = 600  # 10 minutes

for size in (4, 8, 16, 32):
	for robots_prob in [0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375, 0.5]:
		for obst_prob in [0.1]:
			n_robots = int(size * size * robots_prob)
			random.seed(377)
			seeds = random.sample(range(1, 1000), 100)
			for seed in seeds:

				filename = get_filename.get_grid_filename(size, size, n_robots, obst_prob, seed)
				command = get_command.get_generate_grid_command(size, size, n_robots, obst_prob, seed, filename)

				print(command)
				subprocess.run(args=command, timeout=timeout)

for dim in (3, 6, 9):
	for robots_prob in [0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375, 0.5]:
		n_robots = int(2 ** dim * robots_prob)
		random.seed(377)
		seeds = random.sample(range(1, 1000), 100)
		for seed in seeds:

			filename = get_filename.get_grid_filename(dim, n_robots, seed)
			command = get_command.get_generate_hyper_command(dim, n_robots, seed, filename)

			print(command)
			subprocess.run(args=command, timeout=timeout)
