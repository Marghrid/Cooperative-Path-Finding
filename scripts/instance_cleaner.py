import glob
import re

cpfs = glob.glob('../instances/*.cpf')
# outs  = glob.glob('../M_solutions/*.out')
stats = glob.glob('../M_stats_files/*.txt')

cropped_stats = []

for filename in stats:
    filename = re.sub('../M_stats_files/', '', filename)
    filename = re.sub('.txt', '', filename)
    filename = re.sub('_unsat-sat', '', filename)
    filename = re.sub('_binary', '', filename)
    if filename not in cropped_stats:
        cropped_stats += [filename]

cropped_stats.sort()
cpfs.sort()
print(cropped_stats)

for filename in cpfs:
    filename = re.sub('../instances/', '', filename)
    filename = re.sub('.cpf', '', filename)

    if filename not in cropped_stats \
            and (filename.startswith('grid_04x04') or \
                 filename.startswith('grid_08x08')):
        # shutil.copy2('../instances/' + filename + '.cpf', '../unsolved_instances/' + filename + '.cpf')
        print('found ' + filename)

#    elif filename in cropped_stats:
#        cropped_stats.remove(filename)
#        print('removed ' + filename)
#
# print(cropped_stats)
#
