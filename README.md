# CPF
NTIA project: Cooperative Path-Finding

**Folders overview:**

**code**: Contains all the real project's code. The goal is to create a program that when given a CPF instance file provides a solution. The instance is solved using SAT, being the program's responsability to make all necessary calls to a SAT solver.

**minisat** and **glucose**: Have all the minisat and glucose SAT solvers' code and executable files.

**reLOC-0.12-beersheva_077** and **reLOC-0.13-odaiba_037**: Surynek's source code and executables.

**instances**: Contains a few instances generated with Surynek's generator and their solution file, when available.

**solver_files**: Contains Surynek's solver's output for each instance (both to stdout and stderr).

**website**: All files related to the insatnce viewer website.
