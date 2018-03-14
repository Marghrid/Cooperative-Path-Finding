def get_grid_filename(x_size, y_size, n_agents, obst_prob, seed):
	filename = "grid_" + str(x_size).zfill(2) + "x" + str(y_size).zfill(2)
	filename += "_a" + str(n_agents).zfill(3)
	# filename += "_o" + str(obst_prob)
	filename += "_s" + str(seed).zfill(3)
	return filename


def get_hyper_filename(dim, n_agents, seed):
	filename = "hyper" + str(dim)
	filename += "_a" + str(n_agents).zfill(3)
	filename += "_s" + str(seed).zfill(3)
	return filename
