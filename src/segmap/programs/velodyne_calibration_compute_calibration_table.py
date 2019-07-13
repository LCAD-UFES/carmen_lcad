import pprint
import numpy as np


def update_table(table, readings_to_average):
	already_processed = []

	for i in range(len(readings_to_average)):
		ref_laser_id = int(readings_to_average[i][0])
		ref_raw = int(readings_to_average[i][1])
	
		if (ref_laser_id, ref_raw) in already_processed:
			continue
	
		s = 0
		ss = 0
		c = 0
	
		for j in range(len(readings_to_average)):
			laser_id = int(readings_to_average[j][0])
			raw = int(readings_to_average[j][1])
			
			if ref_laser_id != laser_id:
				# we normalize for numerical stability
				raw_norm = raw / 255.0
				s += raw_norm
				ss += raw_norm ** 2
				c += 1
	
		if c > 0:
			table["sum"][ref_laser_id, ref_raw] += s 
			table["count"][ref_laser_id, ref_raw] += c
			table["sum_squared"][ref_laser_id, ref_raw] += ss
	
		already_processed.append((ref_laser_id, ref_raw))


def write_mat(f, mat):
	f.write("%d %d\n" % (mat.shape[0], mat.shape[1]))
	for i in range(mat.shape[0]):
		for j in range(mat.shape[1]):
			f.write("%lf " % mat[i, j])
		f.write("\n")


def save_calib_file(table):
	with open("calib_table.txt", "w") as f:
		write_mat(f, table["sum"])
		write_mat(f, table["count"])
		write_mat(f, table["sum_squared"])


if __name__ == "__main__":
	f = open("calib-data-sorted.txt", "r")
	s = f.readline().rstrip()

	MIN_NUM_READINGS_TO_AVG = 2
	first = True
	prev_cx = prev_cy = 0
	readings_to_average = []

	table = {
		"sum": np.zeros((32, 256)),
		"count": np.zeros((32, 256)),
		"sum_squared": np.zeros((32, 256)),
	}

	line_count = 0

	while s != "":
		if line_count % 100000 == 0:
			print("Line %d of %d" % (line_count, 281283391))
			save_calib_file(table)

		line = s.rsplit()

		cell_x = int(line[7])
		cell_y = int(line[8])

		if cell_x != prev_cx or cell_y != prev_cy or first:
			if len(readings_to_average) >= MIN_NUM_READINGS_TO_AVG:
				update_table(table, readings_to_average)
			
			readings_to_average = []
			first = False
	
		readings_to_average.append(line)
		prev_cx = cell_x
		prev_cy = cell_y

		s = f.readline().rstrip()
		line_count += 1


	save_calib_file(table)