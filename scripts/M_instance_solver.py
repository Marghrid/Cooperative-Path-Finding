from os import system
from os import walk

# filenames will contain the names of all files in the directory intances
# walk() will go through all files and subdirectories of instances.
# next will select the first result of walk.
#  If none is returned, the second argument is used.
_, _, instances = next(walk("../instances"), (None, None, []))
_, _, handmade  = next(walk("../handmade_instances"), (None, None, []))

instances += handmade;

for filename in instances:
	instance = filename[:-4]
	aux  = " -i ../instances/"  + filename
	aux += " -o ../M_solutions/"   + instance + "_binary.out"
	aux += " -s ../M_stats_files/" + instance + "_binary.txt"
	aux += " -e simplified"
	aux += " -search binary"
	aux += " -v 0"

	print ("../CPFSolver/bin/CPFSolver" + aux)
	system("../CPFSolver/bin/CPFSolver" + aux)

	aux  = " -i ../instances/"  + filename
	aux += " -o ../M_solutions/"   + instance + "_unsat-sat.out"
	aux += " -s ../M_stats_files/" + instance + "_unsat-sat.txt"
	aux += " -e simplified"
	aux += " -search unsat-sat"
	aux += " -v 0"

	print ("../CPFSolver/bin/CPFSolver" + aux)
	system("../CPFSolver/bin/CPFSolver" + aux)
