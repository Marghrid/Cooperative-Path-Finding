import constants


def get_solve_command(instance_name, encoding='simplified', search='UNSAT-SAT', verbosity=0, timeout=-1):
    command = [constants.exec_command]
    command += ['-i'] + [constants.instances_dir + '/' + instance_name + '.cpf']
    command += ['-o'] + [constants.solutions_dir + '/' + instance_name + '_' + search + '.out']
    command += ['-s'] + [constants.stat_files_dir + '/' + instance_name + '_' + search + '.txt']
    command += ['-e'] + [encoding]
    command += ['-search'] + [search]
    command += ['-v'] + [str(verbosity)]
    command += ['-t'] + [str(timeout)]

    return command


def get_sury_solve_command(instance_name, encoding='simplicial', search='linear-up', makespan=32):
    command = [constants.sury_exec_command]
    command += ['--input-file=' + constants.sury_instances_dir + '/' + instance_name + '.cpf']
    command += ['--output-file=' + constants.sury_solutions_dir + '/' + instance_name + '_' + search + '.out']
    #command += ['--encoding=' + encoding]
    command += ['--strategy=' + search]
    command += ['--makespan-limit=' + str(makespan)]

    return command


def get_generate_grid_command(x_size, y_size, n_robots, obst_prob, seed, filename):
    command = ['../../reLOC-0.16-takao_013/src/gridgen_reLOC']
    command += ['--x-size=' + str(x_size)]
    command += ['--y-size=' + str(y_size)]
    command += ['--N-robots=' + str(n_robots)]
    command += ['--obstacle-probability=' + str(obst_prob)]
    command += ['--seed=' + str(seed)]
    command += ['--multirobot-file=' + constants.instances_dir + '/' + filename + '.cpf']

    return command


def get_generate_8grid_command(x_size, y_size, n_robots, obst_prob, seed, filename):
    command = ['../../reLOC-0.16-takao_013/src/gridgen_reLOC']
    command += ['--x-size=' + str(x_size)]
    command += ['--y-size=' + str(y_size)]
    command += ['--N-robots=' + str(n_robots)]
    command += ['--obstacle-probability=' + str(obst_prob)]
    command += ['--seed=' + str(seed)]
    command += ['--multirobot-file=' + constants.grid8_instances_dir + '/' + filename + '.cpf']

    return command


def get_generate_hyper_command(dim, n_robots, seed, filename):
    command = ['../../reLOC-0.16-takao_013/src/hypergen_reLOC']
    command += ['--dimmension=' + str(dim)]
    command += ['--N-robots=' + str(n_robots)]
    command += ['--seed=' + str(seed)]
    command += ['--multirobot-file=../instances/' + filename + '.cpf']

    return command
