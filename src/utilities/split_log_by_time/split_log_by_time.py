#!/usr/bin/python
import sys, os

if __name__ == "__main__":
	args = sys.argv
	if len(args) < 5:
		print('\n{}  <log_name>  <init time>  <final time>  <output_log>\n'.format(os.path.basename(args[0])))
		sys.exit()

	in_log = args[1]
	init_time = float(args[2])
	final_time = float(args[3])
	out_log = args[4]

	in_log_file = open(in_log, 'r')
	out_log_file = open(out_log, 'w')
	rec_count = head_count = message_count = 0

	for line in in_log_file:
		rec_count += 1
		tokens = line.split()
		if not tokens:
			continue
		if tokens[0] in ('#', 'PARAM'):
			out_log_file.write(line)
			head_count += 1
			continue
		line_time = float(tokens[-1])
		if line_time < init_time:
			continue
		if (line_time > final_time):
			break
		out_log_file.write(line)
		message_count += 1
		if '_IN_FILE' in tokens[0]:
			in_file_name = tokens[1]
			if in_file_name[0] == '/':
				pos = in_file_name.find(in_log)
				if pos == -1:
					print('Line #{}: IN_FILE: {} does not match with log name: {}'.format(rec_count, in_file_name, in_log))
					continue
				out_file_name = out_log + in_file_name[pos + len(in_log):]
			else:
				out_file_name = out_log + in_file_name
				in_file_name = in_log + in_file_name
			out_dir = os.path.dirname(out_file_name)
			if os.system('mkdir -p {1} && cp {0} {1}'.format(in_file_name, out_dir)) != 0:
				print('Line #{}: Could not copy IN_FILE: {} to the output directory: {}'.format(rec_count, in_file_name, out_dir))

	in_log_file.close()
	out_log_file.close()

	print("Done!")
	print("{} header lines".format(head_count))
	print("{} messages".format(message_count))
