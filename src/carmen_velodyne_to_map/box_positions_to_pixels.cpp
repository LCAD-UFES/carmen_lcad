/*
 * box_positions_to_pixels.cpp

 *
 *  Created on: May 22, 2017
 *      Author: luan
 */

#include <carmen/carmen.h>

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


static void _mkdir(const char *dir)
{
	char tmp[256];
	char *p = NULL;
	size_t len;

	snprintf(tmp, sizeof(tmp),"%s",dir);
	len = strlen(tmp);
	if(tmp[len - 1] == '/')
		tmp[len - 1] = 0;
	for(p = tmp + 1; *p; p++)
		if(*p == '/')
		{
			*p = 0;
			mkdir(tmp, S_IRWXU);
			*p = '/';
		}
	mkdir(tmp, S_IRWXU);
}


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

		object_list.push_back(object);

	}

	fclose(input_file);

}


carmen_vector_2D_t
rotate_point(carmen_vector_2D_t p_in, double theta)
{
	carmen_vector_2D_t p_out;

	p_out.x = p_in.x * cos(theta) - p_in.y * sin(theta);
	p_out.y = p_in.x * sin(theta) + p_in.y * cos(theta);

	return p_out;
}


carmen_vector_2D_t
translate_point(carmen_vector_2D_t p_in, carmen_vector_2D_t offset)
{
	carmen_vector_2D_t p_out;

	p_out.x = p_in.x + offset.x;
	p_out.y = p_in.y + offset.y;

	return p_out;
}


void
generate_boundign_box(FILE * out_file, double length, double width, double theta, carmen_vector_2D_t origin)
{
	carmen_vector_2D_t p1, p2, p3, p4;
	double xt, yt, xb, yb;
	double min_x, min_y, max_x, max_y;

	double map_resolution = 0.05;
	double x_min = -30.0;
	double x_max = 30.0;
	double y_min = -15.0;
	double y_max = 15.0;

	int w = (x_max - x_min)/map_resolution;
	int h = (y_max - y_min)/map_resolution;

	p1.x = length/2.0;
	p1.y = width/2.0;
	p2.x = length/2.0;
	p2.y = -width/2.0;
	p3.x = -length/2.0;
	p3.y = -width/2.0;
	p4.x = -length/2.0;
	p4.y = width/2.0;

	p1 = translate_point(rotate_point(p1, theta), origin);
	p2 = translate_point(rotate_point(p2, theta), origin);
	p3 = translate_point(rotate_point(p3, theta), origin);
	p4 = translate_point(rotate_point(p4, theta), origin);

	min_x = std::min(p1.x, std::min(p2.x, std::min(p3.x, p4.x)));
	max_x = std::max(p1.x, std::max(p2.x, std::max(p3.x, p4.x)));
	min_y = std::min(p1.y, std::min(p2.y, std::min(p3.y, p4.y)));
	max_y = std::max(p1.y, std::max(p2.y, std::max(p3.y, p4.y)));

	xt = (min_x - x_min)/map_resolution;
	yt = h - (max_y - y_min)/map_resolution;

	xb = (max_x - x_min)/map_resolution;
	yb = h - (min_y - y_min)/map_resolution;

	fprintf(out_file, "%.2f %.2f %.2f %.2f ", xt, yt, xb, yb);

	(void) w;
}


void
write_files()
{
	double current_timestamp = object_list[0].timestamp;
	char filename[256];
	FILE *output_file;

	_mkdir("/dados/dataset/labels");
	sprintf(filename, "/dados/dataset/labels/%lf.txt", current_timestamp);
	output_file = fopen(filename, "w");
	carmen_vector_2D_t obj_pos;

	printf("tam: %ld\n", object_list.size());

	for (unsigned int i = 0; i < object_list.size(); i++)
	{
		if (current_timestamp != object_list[i].timestamp)
		{
			current_timestamp = object_list[i].timestamp;
			fclose(output_file);
			sprintf(filename, "/dados/dataset/labels/%lf.txt", current_timestamp);
			output_file = fopen(filename, "w");
		}
		fprintf(output_file, "%s 0.00 0 0.00 ", object_list[i].tipo);

		obj_pos.x = object_list[i].pos_x_obj - object_list[i].pos_x_iara;
		obj_pos.y = object_list[i].pos_y_obj - object_list[i].pos_y_iara;
		obj_pos = rotate_point(obj_pos, -object_list[i].orientation_iara);

		obj_pos.x = obj_pos.x - 0.572;

		generate_boundign_box(output_file, object_list[i].length, object_list[i].width, object_list[i].orientation_obj - object_list[i].orientation_iara, obj_pos);
		fprintf(output_file, "0.00 0.00 0.00 0.00 0.00 0.00 0\n");
	}

	fclose(output_file);

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

	write_files();

	return (0);
}

