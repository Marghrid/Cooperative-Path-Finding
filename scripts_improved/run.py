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
counter = 0

if len(sys.argv) >= 2:
	timeout = int(sys.argv[1])

if len(sys.argv) >= 3:
	begin = sys.argv[2]


def run_in_thread(command):
	global counter
	counter += 1
	subprocess.run(args=command, timeout=timeout)
	counter -= 1
	return


filenames  = glob.glob(constants.instances_dir + begin + '*.cpf')
commands = []

for filename in filenames:
	instance = re.sub(constants.instances_dir, '', filename)
	instance = re.sub('.cpf', '', instance)

	command = generate_command.generate_command(instance)
	commands.append(command)

while counter < 20:
	thread = threading.Thread(target=run_in_thread, args=commands.pop())
	thread.start()
	time.sleep(20)
