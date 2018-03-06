import subprocess
import threading
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

if len(sys.argv) >= 2:
	timeout = int(sys.argv[1])

if len(sys.argv) >= 3:
	begin = sys.argv[2]


def run_in_thread(command):
	global n_threads
	n_threads += 1
	subprocess.run(args=command, timeout=timeout)
	n_threads -= 1
	return


filenames  = glob.glob(constants.instances_dir + begin + '*.cpf')

commands = []

for filename in filenames:
	instance = re.sub(constants.instances_dir, '', filename)
	instance = re.sub('.cpf', '', instance)

	command = generate_command.generate_command(instance)
	commands.append(command)

n_commands_total = len(commands);

while n_threads < 20:
	command = commands.pop()
	print( "solving instance " + str(n_commands_total - len(commands)) + " out of " + str(n_commands_total) + ".")
	print( str( int(100 * (n_commands_total - len(commands) ) / n_commands_total)) + "% done.")
	print( str(n_threads) + " threads active.")
	thread = threading.Thread(target=run_in_thread, args=[command])
	thread.start()
	if n_threads > 10:
		time.sleep(10)
