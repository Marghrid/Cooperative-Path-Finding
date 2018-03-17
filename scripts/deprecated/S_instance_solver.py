from os import system
from os import walk
import signal
import sys

stop = False

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    stop = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


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

# filenames will contain the names of all files in the directory intances
# walk() will go through all files and subdirectories of instances.
# next will select the first result of walk.
#  If none is returned, the second argument is used.
_, _, instances = next(walk("../instances"), (None, None, []))
#_, _, handmade  = next(walk("../handmade_instances"), (None, None, []))

#instances += handmade;

system("ulimit -s 1000000")

for filename in instances:
	if stop:
                break

        if not filename.startswith("grid_04x04_a004"):
                continue
    
        instance = filename[:-4]
	aux  = " --input-file=../instances/"  + filename
	aux += " --output-file=../S_solutions/" + instance + "_unsat-sat.out"
	makespan_limit = get_makespan_limit(instance)
	aux += " --makespan-limit=" + str(makespan_limit)
	aux += " --strategy=linear-up"
	aux += " > ../S_solver_files/" + instance + "_unsat-sat.txt 2>&1"

        runsolver  = "../../runsover/runsolver -w ../S_runsolver/" + instance + "_wsol.txt"
        runsolver += " -v ../S_runsolver/" + instance + "_vsol.txt"
 
	print (runsolver + " ../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)
	system(runsolver + " ../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)

	aux  = " --input-file=../instances/"  + filename
	aux += " --output-file=../S_solutions/" + instance + "_binary.out"
	makespan_limit = get_makespan_limit(instance)
	aux += " --makespan-limit=" + str(makespan_limit)
	aux += " --strategy=binary"
	aux += " > ../S_solver_files/" + instance + "_binary.txt 2>&1"

	print ("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)
	system("../reLOC-0.13-odaiba_037/src/solver_reLOC" + aux)

