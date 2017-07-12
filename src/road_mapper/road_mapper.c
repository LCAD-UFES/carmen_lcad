#include <carmen/road_mapper.h>

road_prob*
road_mapper_double_to_prob(double *value)
{
	return (road_prob*) value;
}

void
road_mapper_cell_color(road_prob *cell, unsigned char *blue, unsigned char *green, unsigned char *red)
{
	if (cell->broken_marking > cell->off_road ||
			cell->solid_marking > cell->off_road ||
			cell->lane_center > cell->off_road)
	{
		*blue = (unsigned char) round(255.0 * cell->broken_marking / MAX_PROB);
		*green = (unsigned char) round(255.0 * cell->lane_center / MAX_PROB);
		*red = (unsigned char) round(255.0 * cell->solid_marking / MAX_PROB);
	}
	else
	{
		*blue = 255;
		*green = 255;
		*red = 255;
	}
}

