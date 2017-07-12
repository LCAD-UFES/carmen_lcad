
#ifndef __ROAD_MAPPER_H_
#define __ROAD_MAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PROB (pow(2.0, 16) - 1.0)

typedef struct						/* Probabilities of a pixel in the lane map */
{
	unsigned short off_road;		/* Probability of a pixel off road */
	unsigned short solid_marking;	/* Probability of pixel in the lane's solid marking */
	unsigned short broken_marking;	/* Probability of pixel in the lane's broken marking */
	unsigned short lane_center;		/* Probability of pixel in lane center */
} road_prob;

inline road_prob*
road_mapper_double_to_prob(double *value)
{
	return (road_prob*) value;
}

inline void
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

#ifdef __cplusplus
}
#endif

#endif

