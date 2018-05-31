import subprocess
import threading
import constants
import datetime
import random
import time
import glob
import sys
import re

import constants
import get_command

timeout = 1800  # 30 minutes
begin = ''
current_command = []
n_threads = 0
n_max_threads = 16
surynek = True
all_commands = []

timed_out_filename = constants.timed_out_commands_file
executed_filename = constants.executed_commands_file
segfault_filename = constants.segfault_commands_file

timed_out_file = open(timed_out_filename, 'w+');
timed_out_file.close()

executed_file = open(executed_filename, 'w+')
executed_file.close()

segfault_file = open(segfault_filename, 'w+')
segfault_file.close()

for i in range(len(sys.argv)):
    if sys.argv[i] == '-b':
        begin = sys.argv[i + 1]
        continue
    elif sys.argv[i] == '-t':
        timeout = int(sys.argv[i + 1])
        continue
    elif sys.argv[i] == '-s':
        surynek = True
        print ("Solving using Surynek's solver")


def run_in_thread(thread_command):
    global n_threads
    global file
    global timed_out_filename
    global executed_filename
    n_threads += 1
    print(str(datetime.datetime.now()))
    print(thread_command)

    executed_file = open(executed_filename, 'a')
    executed_file.write(str(datetime.datetime.now()) + ' : ' \
                        + thread_command[2] + ' ' + thread_command[10] + '\n')
    executed_file.close()

    try:
        process = subprocess.run(args=thread_command, timeout=timeout * 1.2)

        if process.returncode == -6:
            segfault_file = open(segfault_filename, 'a')
            segfault_file.write(str(datetime.datetime.now()) + ' : ' \
                                + thread_command[2] + ' ' + thread_command[10] + '\n')
            segfault_file.close()

    except subprocess.TimeoutExpired:
        timed_out_file = open(timed_out_filename, 'a')
        timed_out_file.write(str(datetime.datetime.now()) + ' : ' \
                             + thread_command[2] + ' ' + thread_command[10] + '\n')
        timed_out_file.close()

    n_threads -= 1
    return


def run_in_thread_s(thread_command):
    global n_threads
    global file
    global timed_out_filename
    global executed_filename
    n_threads += 1
    print(str(datetime.datetime.now()))
    print(thread_command)

    commands_file = open(executed_filename, 'a')
    commands_file.write(str(datetime.datetime.now()) + ' : ' + thread_command[1] + ' ' + thread_command[3] + '\n')
    commands_file.close()

    try:
        process = subprocess.run(args=thread_command, timeout=timeout * 1.2)

        if process.returncode == -6:
            segfault_file = open(segfault_filename, 'a')
            segfault_file.write(str(datetime.datetime.now()) + ' : ' + thread_command[1] + ' ' + thread_command[3] + '\n')
            segfault_file.close()
            n_threads -= 1
            return

    except subprocess.TimeoutExpired:
        timed_out_commands_file = open(timed_out_filename, 'a')
        timed_out_commands_file.write(str(datetime.datetime.now()) + ' : ' + thread_command[1] + ' '  + thread_command[3] + '\n')
        timed_out_commands_file.close()
        n_threads -= 1
        return        
 
    output_filename = re.sub('--output-file=', '', thread_command[2])
    solved = glob.glob(constants.sury_solutions_dir + '/' + output_filename)
    if len(solved) == 0:
        output_file = open(output_filename, 'w')
        output_file.write('No solution\n')
        output_file.close

    n_threads -= 1
    return


#######################  MAIN ##########################

filenames = glob.glob(constants.instances_dir + '/' + '*.cpf')

for filename in filenames:
    instance = re.sub(constants.instances_dir + '/', '', filename)
    instance = re.sub('.cpf', '', instance)

    # current_command = get_command.get_solve_command(instance, search='binary', verbosity=0,timeout=timeout)
    # all_commands.append(current_command)

    current_command = get_command.get_sury_solve_command(instance, makespan=32)
    all_commands.append(current_command)

n_commands_total = len(all_commands)
print('All ' + str(n_commands_total) + ' commands ready')
random.shuffle(all_commands)
#all_commands.sort(reverse=True)


while len(all_commands) > 0:
    if n_threads < n_max_threads:
        current_command = all_commands.pop()
        print('solving instance ' + str(n_commands_total - len(all_commands)) + ' out of ' + str(
            n_commands_total) + '.')
        print(str(int(100 * (n_commands_total - len(all_commands)) / n_commands_total)) + '% done.')
        print(str(n_threads) + ' threads active.')
        if not surynek:
            thread = threading.Thread(target=run_in_thread, args=[current_command])
        else:
            thread = threading.Thread(target=run_in_thread_s, args=[current_command])
        thread.start()
    else:
        # if n_threads > 20:
        time.sleep(10)
