import sys

def main(filename):
	data = []
	f = open(filename, 'r')
	s = f.readline().rstrip()
	while s != '':
		data.append(s.rsplit(' '))
		s = f.readline().rstrip()
	f.close()

	x0_x = float(data[0][3])
	x0_y = float(data[0][4])
	for x in data:
		xi_x = float(x[3])
		xi_y = float(x[4])
		print xi_x - x0_x, xi_y - x0_y

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print ""
		print "Use", sys.argv[0], "<nome do arquivo de poses x,y,theta,time>"
		print ""
	else:
		main(sys.argv[1])
