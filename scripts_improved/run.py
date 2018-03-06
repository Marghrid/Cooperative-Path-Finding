import threading
import subprocess
import time
import sys
import os

import generate_command

timeout = 14400 # 2 hours
begin = ''

if len(sys.argv) >= 2:
	timeout = int(sys.argv[1])

if len(sys.argv) >= 3:
	begin = sys.argv[2]


def run_in_thread(command):
	subprocess.run(args=command, timeout=timeout)
	return


_, _, filenames = next(os.walk("../instances/" + begin + "*.cpf"), (None, None, []))

commands = []

for filename in filenames:
	instance = filename[:-4]
	command = generate_command.generate_command(instance)
	commands.append(command)


counter = 0

while counter < 20:
	thread = threading.Thread(target=run_in_thread, args=command)
	thread.start()
	time.sleep(20)




