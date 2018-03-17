import glob

filenames  = glob.glob(constants.stat_files_dir + begin + '*.cpf')

for filename in filenames:
	content = open(filename)
	instance = re.sub(constants.stat_files_dir, '', filename)
	for line in content:
		if "Out of memory" in line:
			print(instance)
		elif "Unknown error" in line:
			print("unknown: " + instance)

