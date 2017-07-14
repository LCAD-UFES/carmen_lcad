#include <math.h>

#ifndef ROAD_MAPPER_H_
#define ROAD_MAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PROB (pow(2.0, 16) - 1.0)

typedef struct						/* Probabilities of a pixel in the road map */
{
	unsigned short off_road;		/* Probability of being off the road */
	unsigned short solid_marking;	/* Probability of being in a lane solid marking */
	unsigned short broken_marking;	/* Probability of being in a lane broken marking */
	unsigned short lane_center;		/* Probability of being at the center of a lane */
} road_prob;

road_prob* road_mapper_double_to_prob(double *value);

void road_mapper_cell_color(road_prob *cell,
							unsigned char *blue,
							unsigned char *green,
							unsigned char *red);

#ifdef __cplusplus
}
#endif

#endif /* ROAD_MAPPER_H_ */

