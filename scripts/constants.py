import datetime

instances_dir  = '../instances'
solutions_dir  = '../solutions'
stat_files_dir = '../stat_files'
exec_command = '../CPFSolver/bin/CPFSolver'

sury_exec_command = '../../reLOC-0.16-takao_013/src/insolver_reLOC'
sury_instances_dir = '../instances'
sury_solutions_dir  = '../solutions'

executed_commands_file = 'executed_04_' + str(datetime.datetime.now()) + '.txt'
timed_out_commands_file = 'timed_out_04_' + str(datetime.datetime.now()) + '.txt'

table_filename = 'table_04_' + str(datetime.datetime.now()) + '.csv'
