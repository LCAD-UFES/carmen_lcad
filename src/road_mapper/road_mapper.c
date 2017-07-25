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
road_mapper_cell_black_and_white(road_prob *cell, unsigned char *intensity, const int class_bits)
{
	int bits = (class_bits < 0) ? 0 : (class_bits > 6) ? 6 : class_bits;

	if (cell->lane_center >= cell->broken_marking &&
			cell->lane_center >= cell->solid_marking &&
			cell->lane_center >= cell->off_road)
	{
		*intensity = LANE_CLASS | (unsigned char) round(MAX_CLASS(bits) * cell->lane_center / MAX_PROB);
	}
	else if (cell->broken_marking >= cell->solid_marking &&
			cell->broken_marking >= cell->off_road)
	{
		*intensity = BROKEN_CLASS | (unsigned char) round(MAX_CLASS(bits) * cell->broken_marking / MAX_PROB);
	}
	else if (cell->solid_marking >= cell->off_road)
	{
		*intensity = SOLID_CLASS | (unsigned char) round(MAX_CLASS(bits) * cell->solid_marking / MAX_PROB);
	}
	else
	{
		*intensity = OFF_ROAD_CLASS;
	}
}
