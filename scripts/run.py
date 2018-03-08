import subprocess
import threading
import datetime
import random
import time
import glob
import sys
import re

import constants
import generate_command

timeout = 14400  # 2 hours
begin = ''
command = ''
n_threads = 0
commands = []
expired = []
file = open("timedout_commands.txt", "w+")
file.close()

if len(sys.argv) >= 2:
	timeout = int(sys.argv[1])

if len(sys.argv) >= 3:
	begin = sys.argv[2]


def run_in_thread(command):
	global n_threads
	global expired
	global file
	n_threads += 1
	print(command)
	print(str(datetime.datetime.now()))
	
	try:
		subprocess.run(args=command, timeout=timeout)
	except subprocess.TimeoutExpired:
		expired += [command]
		print("timeout expired on " + str(command))
		file = open("timedout_commands.txt", "a")
		file.write("timeout expired on " + str(command))
		file.close()
	
	
	n_threads -= 1
	return


filenames  = glob.glob(constants.instances_dir + begin + '*.cpf')


for filename in filenames:
	instance = re.sub(constants.instances_dir, '', filename)
	instance = re.sub('.cpf', '', instance)

	command = generate_command.generate_command(instance, search="UNSAT-SAT")
	commands.append(command)

	command = generate_command.generate_command(instance, search="binary")
	commands.append(command)

n_commands_total = len(commands)
random.shuffle(commands)

while len(commands) > 0:
	if n_threads < 20:
		command = commands.pop()
		print( "solving instance " + str(n_commands_total - len(commands)) + " out of " + str(n_commands_total) + ".")
		print( str( int(100 * (n_commands_total - len(commands) ) / n_commands_total)) + "% done.")
		print( str(n_threads) + " threads active.")
		thread = threading.Thread(target=run_in_thread, args=[command])
		thread.start()
	if n_threads > 16:
		time.sleep(10)

print( str( len(expired) ) + " commands timed out:")
for command in expired:
	print(command)


