import datetime

instances_dir  = '../instances/'
solutions_dir  = '../solutions_05'
stat_files_dir = '../stat_files_05'
exec_command = '../CPFSolver/bin/CPFSolver'

executed_commands_file = 'executed_05_' + str(datetime.datetime.now()) + '.txt'
timed_out_commands_file = 'timed_out_05_' + str(datetime.datetime.now()) + '.txt'

table_filename = 'table.csv'
