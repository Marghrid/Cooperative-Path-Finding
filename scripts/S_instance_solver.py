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


for instance in instances:
	aux  = " --input-file=../instances/"  + instance
	instance = instance.split(',')[0]
	aux += " --output-file=../S_solutions/" + instance + "unsat-sat.out"
	makespan_limit = get_makespan_limit(instance)
	aux += " --makespan-limit=" + str(makespan_limit)
	aux += " --strategy=linear-up"
	aux += " > ../S_solver_files/" + instance + "unsat-sat.txt 2>&1"

	print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)
	system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)

