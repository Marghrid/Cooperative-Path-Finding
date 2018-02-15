from os import system
from os import walk

# filenames will contain the names of all files in the directory intances
# walk() will go through all files and subdirectories of instances.
# next will select the first result of walk.
#  If none is returned, the second argument is used.
_, _, instances = next(walk("../instances"), (None, None, []))
_, _, handmade  = next(walk("../handmade_instances"), (None, None, []))

instances += handmade;

def get_makespan_limit(instance):
	if(instance.startswith("grid")):
		agents = instance.split("_a")[1]
		agents = agents.split("_")[0]
		agents = int(agents)

		dim = instance.split("grid_")[1]
		dim = dim.split("_a")[0]

		dim1 = dim.split("x")[0]
		dim2 = dim.split("x")[1]

		dim1 = int(dim1)
		dim2 = int(dim2)

		return dim1 * dim2 * agents

	elif(instance.startswith("hyper")):
		agents = instance.split("_a")[1]
		agents = agents.split("_")[0]
		agents = int(agents)

		dim = instance.split("dim")[1]
		dim = dim.split("_a")[0]
		dim = int(dim)

		return 2**dim*agents

	return 64


for filename in instances:
	instance = filename[:-4]
	aux  = " --input-file=../instances/"  + filename
	aux += " --output-file=../S_solutions/" + instance + "_unsat-sat.out"
	makespan_limit = get_makespan_limit(instance)
	aux += " --makespan-limit=" + str(makespan_limit)
	aux += " --strategy=linear-up"
	aux += " > ../S_solver_files/" + instance + "_unsat-sat.txt 2>&1"

	print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)
	system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)

	aux  = " --input-file=../instances/"  + filename
	aux += " --output-file=../S_solutions/" + instance + "_binary.out"
	makespan_limit = get_makespan_limit(instance)
	aux += " --makespan-limit=" + str(makespan_limit)
	aux += " --strategy=binary"
	aux += " > ../S_solver_files/" + instance + "_binary.txt 2>&1"

	print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)
	system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)

