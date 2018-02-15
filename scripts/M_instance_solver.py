from os import system
from os import walk

# filenames will contain the names of all files in the directory intances
# walk() will go through all files and subdirectories of instances.
# next will select the first result of walk.
#  If none is returned, the second argument is used.
_, _, instances = next(walk("../instances"), (None, None, []))
_, _, handmade  = next(walk("../handmade_instances"), (None, None, []))

instances += handmade;

for instance in instances:
	aux  = " -i ../instances/"  + instance
	instance = instance.split('.')[0]
	aux += " -o ../M_solutions/" + instance + ".out"
	aux += " -s ../M_stats_files/" + instance + ".txt"
	aux += " -e simplified"
	aux += " -search binary"
	aux += " -v 1"

	output = ""
	print ("../CPFSolver/bin/CPFSolver" + aux)
	system("../CPFSolver/bin/CPFSolver" + aux)
