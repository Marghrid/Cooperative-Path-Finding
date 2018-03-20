import datetime

instances_dir  = '../instances'
solutions_dir  = '../solutions_06'
stat_files_dir = '../stat_files_06'
exec_command = '../CPFSolver/bin/CPFSolver'

executed_commands_file = 'executed_06_' + str(datetime.datetime.now()) + '.txt'
timed_out_commands_file = 'timed_out_06_' + str(datetime.datetime.now()) + '.txt'

table_filename = 'table_06_' + str(datetime.datetime.now()) + '.csv'
