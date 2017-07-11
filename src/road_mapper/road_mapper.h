
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

road_prob*
road_mapper_double_to_prob(double *value)
{
	return (road_prob*) value;
}

void
road_mapper_cell_color(road_prob *cell, uchar *blue, uchar *green, uchar *red)
{
	*blue = (uchar) round(255.0 * cell->broken_marking / MAX_PROB);
	*green = (uchar) round(255.0 * cell->lane_center / MAX_PROB);
	*red = (uchar) round(255.0 * cell->solid_marking / MAX_PROB);
}

#ifdef __cplusplus
}
#endif

#endif

