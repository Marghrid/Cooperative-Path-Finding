def get_grid_filename(x_size, y_size, n_agents, obst_prob, seed):
    filename = "grid_" + str(x_size).zfill(2) + "x" + str(y_size).zfill(2)
    filename += "_a" + str(n_agents).zfill(3)
    # filename += "_o" + str(obst_prob)
    filename += "_s" + str(seed).zfill(3)
    return filename


def get_hyper_filename(dim, n_agents, seed):
    filename = "hyper" + str(dim).zfill(2)
    filename += "_a" + str(n_agents).zfill(3)
    filename += "_s" + str(seed).zfill(3)
    return filename


def get_instance_type(filename):
    if filename.startswith("grid"):
        return filename.split("_")[0] + " " + filename.split("_")[1]

    elif filename.startswith("hyper"):
        return filename.split("_")[0]

    # print(filename + " of unknown file type.")
    return ""


def get_n_agents(filename):
    args = filename.split("_")
    args.pop(0)  # removes 'grid' or 'hyperX'
    for arg in args:
        if "a" in arg:
            return int(arg[1:])  # Remove the 'a'
    return -1


def get_seed(filename):
    args = filename.split("_")
    args.pop(0)  # removes 'grid' or 'hyperX'
    for arg in args:
        if "s" in arg:
            return int(arg[1:])  # Remove the 's'
    return -1
