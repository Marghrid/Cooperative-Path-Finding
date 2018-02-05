import glob
import re
import os

cpfs = glob.glob("instances/*.cpf")
outs = glob.glob("instances/*.out")

croped_outs = []
for filename in outs:
	filename = re.sub('instances/', '', filename)
	filename = re.sub('.out', '', filename)
	croped_outs += [filename]


for filename in cpfs:
	filename = re.sub('instances/', '', filename)
	filename = re.sub('.cpf', '', filename)

	if filename not in croped_outs:
		os.rename("instances/" + filename + ".cpf", "unsolved_instances/" + filename + ".cpf")