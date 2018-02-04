import glob
import re
import os

cpfs = glob.glob("instances/*.cpf")
outs = glob.glob("instances/*.out")

for filename in outs:
	filename = re.sub('instances/', '', filename)
	filename = re.sub('.out', '', filename)


for filename in cpfs:
	filename = re.sub('instances/', '', filename)
	filename = re.sub('.cpf', '', filename)

	if filename not in outs:
		os.rename("instances/" + filename + ".cpf", "unsolved_instances/" + filename + ".cpf")