import datetime

instances_dir  = '../instances'
solutions_dir  = '../solutions_04'
stat_files_dir = '../stat_files_04'
exec_command = '../CPFSolver/bin/CPFSolver'

executed_commands_file = 'executed_04_' + str(datetime.datetime.now()) + '.txt'
timed_out_commands_file = 'timed_out_04_' + str(datetime.datetime.now()) + '.txt'

table_filename = 'table.csv'
