
import sys


if __name__ == "__main__":
	args = sys.argv
	if len(args) < 5:
		print("")
		print("python %s <log_name> <init message> <final message> <output_log>" % args[0])
		print("")
	else:
		log_name = args[1]
		init_msg = int(args[2])
		final_msg = int(args[3])
		output_log = args[4]

		f = open(log_name, 'r')
		g = open(output_log, 'w')
		header = ''
		n = 0
		nh = nm = 0

		s = f.readline()
		while s != '':
			s = s.lstrip().rstrip()
			if (s[0] == '#' or 'PARAM' in s):
				g.write(s + '\n')
				nh += 1
			else:
				if n >= init_msg:
					g.write(s + '\n')
					nm += 1
				n += 1
				if (n >= final_msg):
					break
			s = f.readline()

		f.close()
		g.close()

		print("Done!")
		print("%d header lines" % nh)
		print("%d messages" % nm)
