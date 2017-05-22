/*
 * box_positions_to_pixels.cpp
 *
 *  Created on: May 22, 2017
 *      Author: luan
 */

#include <stdio.h>
#include <vector>

typedef struct {
	double x_global_pos;
	double y_global_pos;
	double timestamp;
	int id;
	char tipo[10];
	int oclusion;
	double alpha;
	double height;
	double width;
	double length;
	double pos_x_obj;
	double pos_y_obj;
	double l10;
	double orientation_obj;
	int l12;
	double pos_x_iara;
	double pos_y_iara;
	double l15;
	double orientation_iara;
	double velocity_obj;
} object_data_t;

std::vector<object_data_t> object_list;


void
read_file(char *filename)
{
	FILE *input_file = fopen(filename, "r");
	object_data_t object;

	while(!feof(input_file))
	{
		//reads the parameters from the file
		fscanf(input_file, "%lf %lf %lf %d %[^ ] %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf\n",
				&object.x_global_pos,
				&object.y_global_pos,
				&object.timestamp,
				&object.id,
				object.tipo,
				&object.oclusion,
				&object.alpha,
				&object.height,
				&object.width,
				&object.length,
				&object.pos_x_obj,
				&object.pos_y_obj,
				&object.l10,
				&object.orientation_obj,
				&object.l12,
				&object.pos_x_iara,
				&object.pos_y_iara,
				&object.l15,
				&object.orientation_iara,
				&object.velocity_obj);
	}

	fclose(input_file);

	object_list.push_back(object);
}


int
main(int argc, char **argv)
{
	char *input_filename;

	if (argc > 1)
		input_filename = argv[1];
	else
	{
		printf("Invalid filename\n");
		return 1;
	}

	read_file(input_filename);



	return (0);
}

