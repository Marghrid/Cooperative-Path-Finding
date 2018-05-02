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

timeout = 600  # 10 minutes
begin = ''
current_command = []
n_threads = 0
n_max_threads = 16
surynek = False
all_commands = []

timed_out_filename = constants.timed_out_commands_file
executed_filename = constants.executed_commands_file

timed_out_file = open(timed_out_filename, 'w+');
timed_out_file.close()

commands_file = open(executed_filename, 'w+')
commands_file.close()

for i in range(len(sys.argv)):
	if sys.argv[i] == '-b':
		begin = sys.argv[i+1]
		continue
	elif sys.argv[i] == '-t':
		timeout = int(sys.argv[i+1])
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

	commands_file = open(executed_filename, 'a')
	commands_file.write(str(datetime.datetime.now()) + ' : ' + str(thread_command) + '\n')
	commands_file.close()

	try:
		subprocess.run(args=thread_command, timeout=timeout*1.2)
	except subprocess.TimeoutExpired:
		timed_out_commands_file = open(timed_out_filename, 'a')
		timed_out_commands_file.write(str(datetime.datetime.now()) + ' : ' + str(thread_command) + '\n')
		timed_out_commands_file.close()

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
	commands_file.write(str(datetime.datetime.now()) + ' : ' + str(thread_command) + '\n')
	commands_file.close()

	try:
		subprocess.run(args=thread_command, timeout=timeout*1.2)
	except subprocess.TimeoutExpired:
		timed_out_commands_file = open(timed_out_filename, 'a')
		timed_out_commands_file.write(str(datetime.datetime.now()) + ' : ' + str(thread_command) + '\n')
		timed_out_commands_file.close()

	n_threads -= 1
	return



#######################  MAIN ##########################

filenames = glob.glob(constants.instances_dir + '/' + begin + '*.cpf')

for filename in filenames:
	instance = re.sub(constants.instances_dir + '/', '', filename)
	instance = re.sub('.cpf', '', instance)

	stats = glob.glob(constants.stat_files_dir + '/' + instance + '*.txt')
	if len(stats) > 0:
		print('Instance ' + instance + ' has already been solved.')
		continue

	#current_command = get_command.get_solve_command(instance, search='binary', verbosity=0,timeout=timeout)
	#all_commands.append(current_command)

	if not surynek:
		current_command = get_command.get_solve_command(instance, search='UNSAT-SAT', verbosity=0,timeout=timeout)
	else:
		current_command = get_command.get_sury_solve_command(instance, makespan=32)

	all_commands.append(current_command)

all_commands.sort()
n_commands_total = len(all_commands)
print('All ' + str(n_commands_total) + ' commands ready')
# random.shuffle(all_commands)


while len(all_commands) > 0:
	if n_threads < n_max_threads:
		current_command = all_commands.pop()
		print('solving instance ' + str(n_commands_total - len(all_commands)) + ' out of ' + str(n_commands_total) + '.')
		print(str(int(100 * (n_commands_total - len(all_commands)) / n_commands_total)) + '% done.')
		print(str(n_threads) + ' threads active.')
		if not surynek:
			thread = threading.Thread(target=run_in_thread, args=[current_command])
		else:
			thread = threading.Thread(target=run_in_thread_s, args=[current_command])
		thread.start()
	else:
    #if n_threads > 20:
		time.sleep(10)
