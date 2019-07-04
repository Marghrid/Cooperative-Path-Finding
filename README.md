# CPF
NTIA project: Cooperative Path-Finding

**Folders overview:**

**CPFSolver**: Contains all the real project's code: a program that when given a CPF instance file provides a solution. The instance is solved using SAT, being the program's responsibility to make all necessary calls to a SAT solver.

**NonIncCPFSolver**: Old, worse version of the CPFSolver that doesn't use incremental solving.

**instances**: Contains a few instances generated with Dr. Pavel Surynek's reLOC generator and their solution file, when available.

**handmade_instances**: Instances created using Duarte's [CPFMaker](https://github.com/drcd1/CPFMaker).

**website**: All files related to the instance viewer [website](https://web.tecnico.ulisboa.pt/~ist180832/CPF/).

**glucose**: The SAT solver library

**scripts**: Very useful scripts that run the program and gather data.
