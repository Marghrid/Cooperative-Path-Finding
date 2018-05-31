import datetime

instances_dir  = '/home/macf/CPF/icaart2017_experiments/raw/grid/cpfs'
solutions_dir  = '../solutions30'
stat_files_dir = '../stat_files30'
exec_command = '../CPFSolver/bin/CPFSolver'

#sury_exec_command = '../../reLOC-0.16-takao_013/src/insolver_reLOC'
sury_exec_command = '../../reLOC-0.13-odaiba_037/src/solver_reLOC'
#sury_instances_dir = '../instances'
sury_instances_dir = '/home/macf/CPF/icaart2017_experiments/raw/grid/cpfs'

sury_solutions_dir  = '../sury_solutions'

date = datetime.datetime.now().strftime("%m%d%H%M") 

print(date)

executed_commands_file = 'executed_' + date + '.txt'
timed_out_commands_file = 'timed_out_' + date + '.txt'
segfault_commands_file = 'segfault_' + date + '.txt'

table_filename = 'table_' + date + '.csv'
