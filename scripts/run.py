import subprocess
import threading
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
n_max_threads = 24
all_commands = []

timed_out_commands_file = open("timed_out.txt", "w+");
timed_out_commands_file.close()

commands_file = open("executed_commands.txt", "w+")
commands_file.close()

if len(sys.argv) >= 2:
	timeout = int(sys.argv[1])

if len(sys.argv) >= 3:
	begin = sys.argv[2]


def run_in_thread(thread_command):
	global n_threads
	global file
	global timed_out_commands_file
	global commands_file
	n_threads += 1
	print(thread_command)
	print(str(datetime.datetime.now()))

	commands_file = open("executed_commands.txt", "a")
	commands_file.write(str(datetime.datetime.now()) + " : " + str(thread_command) + "\n")
	commands_file.close()

	try:
		subprocess.run(args=thread_command, timeout=timeout)
	except subprocess.TimeoutExpired:
		timed_out_commands_file = open("timed_out.txt", "a")
		timed_out_commands_file.write(str(datetime.datetime.now()) + " : " + str(thread_command) + "\n")
		timed_out_commands_file.close()

	n_threads -= 1
	return


filenames = glob.glob(constants.instances_dir + begin + '*.cpf')

for filename in filenames:
	instance = re.sub(constants.instances_dir, '', filename)
	instance = re.sub(".cpf", '', instance)

	current_command = get_command.get_solve_command(instance, search="UNSAT-SAT", verbosity=0, timeout=timeout)
	all_commands.append(current_command)

	current_command = get_command.get_solve_command(instance, search="binary", verbosity=0,timeout=timeout)
	all_commands.append(current_command)

n_commands_total = len(all_commands)
random.shuffle(all_commands)



while len(all_commands) > 0:
	if n_threads < n_max_threads:
		current_command = all_commands.pop()
		print(
			"solving instance " + str(n_commands_total - len(all_commands)) + " out of " + str(n_commands_total) + ".")
		print(str(int(100 * (n_commands_total - len(all_commands)) / n_commands_total)) + "% done.")
		print(str(n_threads) + " threads active.")
		thread = threading.Thread(target=run_in_thread, args=[current_command])
		thread.start()
	if n_threads > 20:
		time.sleep(10)
