#include "rddf_graph.h"

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
	else if (cell->broken_marking == cell->off_road &&
			cell->solid_marking == cell->off_road &&
			cell->lane_center == cell->off_road)
	{	// Completely unknown = bluish color
		*blue = 255;
		*green = 144;
		*red = 30;
	}
	else
	{	// Off road = white color
		*blue = 255;
		*green = 255;
		*red = 255;
	}
}

void
road_mapper_cell_black_and_white(road_prob *cell, unsigned char *cell_class, const int class_bits)
{
	// See: https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/road_mapper#description
	// Cell classes (currently, class_bits is always 4):
	//		0: OFF_ROAD_CLASS, 1: SOLID_CLASS, 2: BROKEN_CLASS,
	//		3: SOLID_CLASS (50%), 4: BROKEN_CLASS (50%), 5: LANE_CLASS closest to the center, ..., 5 + MAX_CLASS: LANE_CLASS most distant to the center
	//
	// Subclass bit field length must be in range (0, 6)
	int bits = (class_bits < 0) ? 0 : (class_bits > 6) ? 6 : class_bits;

	if (cell->lane_center >= cell->broken_marking &&
		cell->lane_center >= cell->solid_marking &&
		cell->lane_center >= cell->off_road)
	{
		*cell_class = LANE_CLASS + (bits > 0 ? OFFSET_CLASS + (unsigned char) round(MAX_CLASS(bits) * (1.0 - (cell->lane_center - 1.0) / MAX_PROB)) : 0);
	}
	else if (cell->broken_marking >= cell->solid_marking &&
			 cell->broken_marking >= cell->off_road)
	{
		*cell_class = BROKEN_CLASS + (bits > 0 ? ((cell->broken_marking - 1.0) / MAX_PROB <= 0.5 ? OFFSET_CLASS : 0) : 0);
	}
	else if (cell->solid_marking >= cell->off_road)
	{
		*cell_class = SOLID_CLASS + (bits > 0 ? ((cell->solid_marking - 1.0) / MAX_PROB <= 0.5 ? OFFSET_CLASS : 0) : 0);
	}
	else
	{
		*cell_class = OFF_ROAD_CLASS;
	}
}
