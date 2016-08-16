import sys

def read_file(filename):
	data = []
	f = open(filename, 'r')
	s = f.readline().rstrip()
	while s != '':
		data.append(s.rsplit(' '))
		s = f.readline().rstrip()
	f.close()
	return data


if __name__ == "__main__":
	if len(sys.argv) < 3:
		print ""
		print "Use python", sys.argv[0], "<sync.txt> <opt.txt>"
		print ""
	else:
		sync_file = sys.argv[1]
		opt_file = sys.argv[2]
		
		sync_data = read_file(sync_file)
		opt_data = read_file(opt_file)

		# check if num lines is correct
		if len(sync_data) != len(opt_data):
			print ""
			print "Error: The files have different size"
			print ""
		else:
			i = 0
			while i < len(sync_data):
				print sync_data[i][3], sync_data[i][4], opt_data[i][0], opt_data[i][1]
				i = i + 1

