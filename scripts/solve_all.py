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

timeout = 900  # 15 minutes
begin = ''
current_command = []
n_threads = 0
n_max_threads = 28
all_commands = []

timed_out_filename = constants.timed_out_commands_file
executed_filename = constants.executed_commands_file

timed_out_file = open(timed_out_filename, 'w+');
timed_out_file.close()

commands_file = open(executed_filename, 'w+')
commands_file.close()

if len(sys.argv) >= 2:
	timeout = int(sys.argv[1])

if len(sys.argv) >= 3:
	begin = sys.argv[2]


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


filenames = glob.glob(constants.instances_dir + '/' + begin + '*.cpf')

for filename in filenames:
	instance = re.sub(constants.instances_dir, '', filename)
	instance = re.sub('.cpf', '', instance)

	# current_command = get_command.get_solve_command(instance, search='UNSAT-SAT', verbosity=0, timeout=timeout)
	# all_commands.append(current_command)

	current_command = get_command.get_solve_command(instance, search='binary', verbosity=0,timeout=timeout)
	all_commands.append(current_command)

n_commands_total = len(all_commands)
random.shuffle(all_commands)



while len(all_commands) > 0:
	if n_threads < n_max_threads:
		current_command = all_commands.pop()
		print(
			'solving instance ' + str(n_commands_total - len(all_commands)) + ' out of ' + str(n_commands_total) + '.')
		print(str(int(100 * (n_commands_total - len(all_commands)) / n_commands_total)) + '% done.')
		print(str(n_threads) + ' threads active.')
		thread = threading.Thread(target=run_in_thread, args=[current_command])
		thread.start()
	else:
    #if n_threads > 20:
		time.sleep(10)
