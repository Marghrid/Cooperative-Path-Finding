# CPF
NTIA project: Cooperative Path-Finding

**Folders overview:**

**CPFSolver**: Contains all the real project's code. The goal is to create a program that when given a CPF instance file provides a solution. The instance is solved using SAT, being the program's responsability to make all necessary calls to a SAT solver.

**reLOC-0.12-beersheva_077** and **reLOC-0.13-odaiba_037**: Dr. Pavel Surynek's reLOC source code and executables.

**instances**: Contains a few instances generated with Dr. Pavel Surynek's reLOC generator and their solution file, when available.

**simple_instances**: Contains simpler instances, to be solved by fragile solvers.

**handmade_instances**: Instances created using Duarte's [CPFMaker](https://github.com/drcd1/CPFMaker).

**solver_files**: Contains Dr. Pavel Surynek's reLOC solver's output for each instance (both to stdout and stderr).

**website**: All files related to the insatnce viewer website.
