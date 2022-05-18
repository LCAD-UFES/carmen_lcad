/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <prob_map.h>
#include "localize_ackerman_core.h"

#define      HUGE_DISTANCE     32000


/* compute minimum distance to all occupied cells */

void
carmen_localize_ackerman_create_distance_map(carmen_localize_ackerman_map_p lmap, carmen_map_p cmap,
		double minimum_occupied_prob)
{
	int x, y, i, j, border;
	double v;

	for (x = 0; x < lmap->config.x_size; x++)
	{
		for (y = 0; y < lmap->config.y_size; y++)
		{
			lmap->distance[x][y] = HUGE_DISTANCE;
			lmap->x_offset[x][y] = HUGE_DISTANCE;
			lmap->y_offset[x][y] = HUGE_DISTANCE;
		}
	}

	/* Initialize the distance measurements before dynamic programming */
	for (x = 0; x < lmap->config.x_size; x++)
		for (y = 0; y < lmap->config.y_size; y++)
			if (cmap->map[x][y] > minimum_occupied_prob)
			{
				border = 0;
				for (i = -1; i <= 1; i++)
					for (j = -1; j <= 1; j++)
						if (!border && x + i >= 0 && y + j >= 0
								&& x + i < lmap->config.x_size
								&& y + j < lmap->config.y_size
								&& (i != 0 || j != 0))
						{
							if (cmap->map[x + i][y + j] < minimum_occupied_prob
									&& cmap->map[x + i][y + j] != -1)
								border = 1;
						}
				if (border)
				{
					lmap->distance[x][y] = 0;
					lmap->x_offset[x][y] = 0;
					lmap->y_offset[x][y] = 0;
				}
			}
//	/* Initialize the distance measurements before dynamic programming */
//	for (x = 0; x < lmap->config.x_size; x++)
//	{
//		for (y = 0; y < lmap->config.y_size; y++)
//		{
//			if (cmap->map[x][y] > minimum_occupied_prob)
//			{
//				lmap->distance[x][y] = 0.0;
//				lmap->x_offset[x][y] = 0.0;
//				lmap->y_offset[x][y] = 0.0;
//			}
//		}
//	}

	/* Use dynamic programming to estimate the minimum distance from
     every map cell to an occupied map cell */

	/* pass 1 */
	for(x = 0; x < lmap->config.x_size; x++)
		for(y = 0; y < lmap->config.y_size; y++)
			for(i = -1; i <= 1; i++)
				for(j = -1; j <= 1; j++)
					if(x + i >= 0 && y + j >= 0 && x + i < lmap->config.x_size &&
							y + j < lmap->config.y_size && (i != 0 || j != 0)) {
						v = lmap->distance[x + i][y + j] + ((i * j != 0) ? 1.414 : 1);
						if(v < lmap->distance[x][y]) {
							lmap->distance[x][y] = v;
							lmap->x_offset[x][y] = lmap->x_offset[x + i][y + j] + i;
							lmap->y_offset[x][y] = lmap->y_offset[x + i][y + j] + j;
						}
					}

	/* pass 2 */
	for(x = lmap->config.x_size - 1; x >= 0; x--)
		for(y = lmap->config.y_size - 1; y >= 0; y--)
			for(i = -1; i <= 1; i++)
				for(j = -1; j <= 1; j++)
					if(x + i >= 0 && y + j >= 0 && x + i < lmap->config.x_size &&
							y + j < lmap->config.y_size && (i != 0 || j != 0)) {
						v = lmap->distance[x + i][y + j] + ((i * j != 0) ? 1.414 : 1);
						if(v < lmap->distance[x][y]) {
							lmap->distance[x][y] = v;
							lmap->x_offset[x][y] = lmap->x_offset[x + i][y + j] + i;
							lmap->y_offset[x][y] = lmap->y_offset[x + i][y + j] + j;
						}
					}
}


void
create_likelihood_map(carmen_localize_ackerman_map_p lmap, double **prob, double std)
{
	int x, y;
	double p, max;

	/* Compute the probability of each cell given the standard deviation,
     or "fuzziness" of the likelihood map */
	max = 0;
	for(x = 0; x < lmap->config.x_size; x++)
		for(y = 0; y < lmap->config.y_size; y++) {
			p = exp(-0.5 * carmen_square(lmap->distance[x][y] *
					lmap->config.resolution / std));
			if(p > max)
				max = p;
			prob[x][y] = p;
		}

	/* Correct the map so most likely reading has probability 1 */
	for(x = 0; x < lmap->config.x_size; x++)
		for(y = 0; y < lmap->config.y_size; y++) {
			prob[x][y] /= max;
			if(prob[x][y] < SMALL_PROB)
				prob[x][y] = SMALL_PROB;
			prob[x][y] = log(prob[x][y]);
		}
}


double
carmen_localize_ackerman_create_stretched_probability_map(double **prob, carmen_localize_ackerman_map_p lmap, double std)
{
	double max = 0.0;

	/* Compute the probability of each cell given the standard deviation,
	 or "fuzziness" of the likelihood map */
	for (int x = 0; x < lmap->config.x_size; x++)
	{
		for (int y = 0; y < lmap->config.y_size; y++)
		{
			double p = exp(-0.5 * carmen_square((lmap->distance[x][y] * lmap->config.resolution) / std));
			if (p > max)
				max = p;

			prob[x][y] = p;
		}
	}
	return max;
}


void
carmen_localize_ackerman_create_stretched_log_likelihood_map(double **prob, carmen_localize_ackerman_map_p lmap, double std,
		double min_likelihood, double max_likelihood, int use_log_odds)
{
	double max;

	/* Compute the probability of each cell given the standard deviation,
    	   or "fuzziness" of the likelihood map */
	max = carmen_localize_ackerman_create_stretched_probability_map(prob, lmap, std);
	double **carmen_map = lmap->carmen_map.map;
	/* Correct the map so most likely reading has probability 1 */
	for (int x = 0; x < lmap->config.x_size; x++)
	{
		for (int y = 0; y < lmap->config.y_size; y++)
		{	// Alberto: @@@ Precisa de uma referencia para o codigo abaixo
			double cell_prob = (prob[x][y] / max) * max_likelihood;
			if (cell_prob < min_likelihood)
				cell_prob = min_likelihood;

			if (use_log_odds)
			{
				if (carmen_map[x][y] != -1.0)
					prob[x][y] = carmen_prob_models_probabilistic_to_log_odds(cell_prob);
				else
					prob[x][y] = carmen_prob_models_probabilistic_to_log_odds(0.5);
			}
			else
			{
				if (carmen_map[x][y] != -1.0)
					prob[x][y] = log(cell_prob);
				else
					prob[x][y] = log(0.5);
			}
		}
	}
//	carmen_map_t temp_map;
//	temp_map.config = lmap->config;
//	temp_map.map = prob;
//	temp_map.complete_map = prob[0];
//	carmen_grid_mapping_save_map((char *) "test.map", &temp_map);
}


void
carmen_localize_ackerman_initialize_localize_map(carmen_localize_ackerman_map_p lmap,	carmen_map_p cmap)
{
	int i;

	if (lmap->complete_distance == NULL)
	{
		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

		/* add raw map into likelihood map */
		lmap->carmen_map = *cmap;

		/* allocate distance map */
		lmap->complete_distance = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_distance);

		lmap->distance = (double **) calloc(lmap->config.x_size,
				sizeof(double *));
		carmen_test_alloc(lmap->distance);

		/* allocate prob map */
		lmap->complete_prob = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_prob);
		lmap->prob = (double **) calloc(lmap->config.x_size, sizeof(double *));
		carmen_test_alloc(lmap->prob);

		/* allocate gprob map */
		lmap->complete_gprob = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_gprob);
		lmap->gprob = (double **) calloc(lmap->config.x_size, sizeof(double *));
		carmen_test_alloc(lmap->gprob);

		/* allocate x offset map */
		lmap->complete_x_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_x_offset);
		lmap->x_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->x_offset);
		/* allocate y offset map */
		lmap->complete_y_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_y_offset);
		lmap->y_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->y_offset);
	}
	else if ((lmap->config.x_size != cmap->config.x_size) || // o novo mapa pode ser de tamanho diferente... lmap->config = cmap->config
			(lmap->config.y_size != cmap->config.y_size))
	{
		free(lmap->complete_x_offset);
		free(lmap->complete_y_offset);
		free(lmap->complete_distance);
		free(lmap->complete_prob);
		free(lmap->complete_gprob);
		free(lmap->x_offset);
		free(lmap->y_offset);
		free(lmap->distance);
		free(lmap->prob);
		free(lmap->gprob);

		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

		/* add raw map into likelihood map */
		lmap->carmen_map = *cmap;

		/* allocate distance map */
		lmap->complete_distance = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_distance);

		lmap->distance = (double **) calloc(lmap->config.x_size,
				sizeof(double *));
		carmen_test_alloc(lmap->distance);

		/* allocate prob map */
		lmap->complete_prob = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_prob);
		lmap->prob = (double **) calloc(lmap->config.x_size, sizeof(double *));
		carmen_test_alloc(lmap->prob);

		/* allocate gprob map */
		lmap->complete_gprob = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_gprob);
		lmap->gprob = (double **) calloc(lmap->config.x_size, sizeof(double *));
		carmen_test_alloc(lmap->gprob);

		/* allocate x offset map */
		lmap->complete_x_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_x_offset);
		lmap->x_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->x_offset);
		/* allocate y offset map */
		lmap->complete_y_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_y_offset);
		lmap->y_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->y_offset);
	}
	else
	{
		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

		/* add raw map into likelihood map */
		lmap->carmen_map = *cmap;

		memset(lmap->complete_distance, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(double));
		memset(lmap->distance, 0, lmap->config.x_size * sizeof(double *));
		memset(lmap->complete_prob, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(double));
		memset(lmap->prob, 0, lmap->config.x_size * sizeof(double *));
		memset(lmap->complete_gprob, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(double));
		memset(lmap->gprob, 0, lmap->config.x_size * sizeof(double *));
		memset(lmap->complete_x_offset, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(short int));
		memset(lmap->x_offset, 0, lmap->config.x_size * sizeof(short int *));
		memset(lmap->complete_y_offset, 0,
				lmap->config.x_size * lmap->config.y_size * sizeof(short int));
		memset(lmap->y_offset, 0, lmap->config.x_size * sizeof(short int *));
	}

	for (i = 0; i < lmap->config.x_size; i++)
	{
		lmap->distance[i] = lmap->complete_distance + i * lmap->config.y_size;
		lmap->prob[i] = lmap->complete_prob + i * lmap->config.y_size;
		lmap->gprob[i] = lmap->complete_gprob + i * lmap->config.y_size;
		lmap->x_offset[i] = lmap->complete_x_offset + i * lmap->config.y_size;
		lmap->y_offset[i] = lmap->complete_y_offset + i * lmap->config.y_size;
	}
}


void
carmen_localize_ackerman_initialize_likelihood_map_only(carmen_localize_ackerman_map_p lmap, carmen_map_p cmap)
{
	int i;

	if (lmap->complete_distance == NULL)
	{
		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

		/* add raw map into likelihood map */
		lmap->carmen_map = *cmap;

		/* allocate distance map */
		lmap->complete_distance = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_distance);

		lmap->distance = (double **) calloc(lmap->config.x_size,
				sizeof(double *));
		carmen_test_alloc(lmap->distance);

		/* allocate prob map */
		lmap->complete_prob = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_prob);
		lmap->prob = (double **) calloc(lmap->config.x_size, sizeof(double *));
		carmen_test_alloc(lmap->prob);

		/* allocate x offset map */
		lmap->complete_x_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_x_offset);
		lmap->x_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->x_offset);
		/* allocate y offset map */
		lmap->complete_y_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_y_offset);
		lmap->y_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->y_offset);
	}
	else if ((lmap->config.x_size != cmap->config.x_size) || // o novo mapa pode ser de tamanho diferente... lmap->config = cmap->config
			(lmap->config.y_size != cmap->config.y_size))
	{
		free(lmap->complete_x_offset);
		free(lmap->complete_y_offset);
		free(lmap->complete_distance);
		free(lmap->complete_prob);
		free(lmap->x_offset);
		free(lmap->y_offset);
		free(lmap->distance);
		free(lmap->prob);

		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

		/* add raw map into likelihood map */
		lmap->carmen_map = *cmap;

		/* allocate distance map */
		lmap->complete_distance = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_distance);

		lmap->distance = (double **) calloc(lmap->config.x_size,
				sizeof(double *));
		carmen_test_alloc(lmap->distance);

		/* allocate prob map */
		lmap->complete_prob = (double *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(double));
		carmen_test_alloc(lmap->complete_prob);
		lmap->prob = (double **) calloc(lmap->config.x_size, sizeof(double *));
		carmen_test_alloc(lmap->prob);

		/* allocate x offset map */
		lmap->complete_x_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_x_offset);
		lmap->x_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->x_offset);
		/* allocate y offset map */
		lmap->complete_y_offset = (short int *) calloc(
				lmap->config.x_size * lmap->config.y_size, sizeof(short int));
		carmen_test_alloc(lmap->complete_y_offset);
		lmap->y_offset = (short int **) calloc(lmap->config.x_size,
				sizeof(short int *));
		carmen_test_alloc(lmap->y_offset);
	}
	else
	{
		/* copy map parameters from carmen map */
		lmap->config = cmap->config;

		/* add raw map into likelihood map */
		lmap->carmen_map = *cmap;
	}

	for (i = 0; i < lmap->config.x_size; i++)
	{
		lmap->distance[i] = lmap->complete_distance + i * lmap->config.y_size;
		lmap->prob[i] = lmap->complete_prob + i * lmap->config.y_size;
		lmap->x_offset[i] = lmap->complete_x_offset + i * lmap->config.y_size;
		lmap->y_offset[i] = lmap->complete_y_offset + i * lmap->config.y_size;
	}
}


void
carmen_to_localize_ackerman_map_only_prob(carmen_map_p cmap,
		carmen_map_p mean_remission_map, carmen_map_p variance_remission_map,
		carmen_localize_ackerman_map_p lmap, carmen_localize_ackerman_param_p param)
{
	carmen_localize_ackerman_initialize_localize_map(lmap, cmap);

	static carmen_prob_models_distance_map *distance_map = NULL;

	if (distance_map == NULL)
	{
		distance_map =  (carmen_prob_models_distance_map *) calloc(1, sizeof(carmen_prob_models_distance_map));
		carmen_prob_models_initialize_distance_map(distance_map, cmap->config);
	}

	/*add remission map into localize map*/
	if (mean_remission_map)
		lmap->carmen_mean_remission_map = *mean_remission_map;
	else
		lmap->carmen_mean_remission_map.complete_map = lmap->carmen_map.complete_map;	// so porque a mensagem de mapas de localizacao exige
	if (variance_remission_map)
		lmap->carmen_variance_remission_map = *variance_remission_map;
	else
		lmap->carmen_variance_remission_map.complete_map = lmap->carmen_map.complete_map;	// so porque a mensagem de mapas de localizacao exige

	carmen_prob_models_create_distance_map(distance_map, cmap, param->occupied_prob);
	lmap->complete_distance = distance_map->complete_distance;
	lmap->complete_x_offset = distance_map->complete_x_offset;
	lmap->complete_y_offset = distance_map->complete_y_offset;
	lmap->distance = distance_map->distance;
	lmap->x_offset = distance_map->x_offset;
	lmap->y_offset = distance_map->y_offset;

//	create_likelihood_map(lmap, lmap->prob, param->lmap_std); 
//	create_likelihood_map(lmap, lmap->gprob, param->global_lmap_std); 

	carmen_localize_ackerman_create_stretched_log_likelihood_map(lmap->prob, lmap, param->lmap_std, param->tracking_beam_minlikelihood,
			param->tracking_beam_maxlikelihood, param->use_log_odds);
}


void
carmen_to_localize_ackerman_map(carmen_map_p cmap,
		carmen_map_p mean_remission_map, carmen_map_p variance_remission_map,
		carmen_localize_ackerman_map_p lmap, carmen_localize_ackerman_param_p param)
{
	carmen_to_localize_ackerman_map_only_prob(cmap, mean_remission_map, variance_remission_map, lmap, param);
	carmen_localize_ackerman_create_stretched_log_likelihood_map(lmap->gprob, lmap, param->global_lmap_std, param->global_beam_minlikelihood,
			param->global_beam_maxlikelihood, param->use_log_odds);
}


void
carmen_to_localize_ackerman_likelihood_map_only(carmen_map_p cmap, carmen_localize_ackerman_map_p lmap, carmen_localize_ackerman_param_p param)
{
	carmen_localize_ackerman_initialize_likelihood_map_only(lmap, cmap);

	static carmen_prob_models_distance_map *distance_map = NULL;

	if (distance_map == NULL)
	{
		distance_map =  (carmen_prob_models_distance_map *) calloc(1, sizeof(carmen_prob_models_distance_map));
		carmen_prob_models_initialize_distance_map(distance_map, cmap->config);
	}

	carmen_prob_models_create_distance_map(distance_map, cmap, param->occupied_prob);
	lmap->complete_distance = distance_map->complete_distance;
	lmap->complete_x_offset = distance_map->complete_x_offset;
	lmap->complete_y_offset = distance_map->complete_y_offset;
	lmap->distance = distance_map->distance;
	lmap->x_offset = distance_map->x_offset;
	lmap->y_offset = distance_map->y_offset;

	carmen_localize_ackerman_create_stretched_log_likelihood_map(lmap->prob, lmap, param->lmap_std, param->tracking_beam_minlikelihood,
			param->tracking_beam_maxlikelihood, param->use_log_odds);
}


/* Writes a carmen map out to a ppm file */
void
carmen_localize_ackerman_write_map_to_ppm(char *filename,
		carmen_localize_ackerman_map_p map)
{
	FILE *fp;
	int i, j;
	double max;

	max = -1e6;
	for(i = 0; i < map->config.x_size; i++)
		for(j = 0; j < map->config.y_size; j++)
			if(map->carmen_map.map[i][j] > max)
				max = map->carmen_map.map[i][j];

	fp = fopen(filename, "w");
	fprintf(fp, "P6\n");
	fprintf(fp, "%d %d\n", map->config.x_size, map->config.y_size);
	fprintf(fp, "255\n");
	for(i = map->config.y_size - 1; i >= 0; i--)
		for(j = 0; j < map->config.x_size; j++)
			if(map->carmen_map.map[j][i] == -1) {
				fprintf(fp, "%c", 0);
				fprintf(fp, "%c", 0);
				fprintf(fp, "%c", 255);
			}
			else {
				fprintf(fp, "%c", 255 - (int)(map->carmen_map.map[j][i] /
						max * 255.0));
				fprintf(fp, "%c", 255 - (int)(map->carmen_map.map[j][i] /
						max * 255.0));
				fprintf(fp, "%c", 255 - (int)(map->carmen_map.map[j][i] /
						max * 255.0));
			}
	fclose(fp);
}

/* Writes a distance map out to a ppm file */

void carmen_localize_ackerman_write_distance_map_to_ppm(char *filename, 
		carmen_localize_ackerman_map_p map)
{
	FILE *fp;
	int i, j;
	double max;

	max = -1e6;
	for(i = 0; i < map->config.x_size; i++)
		for(j = 0; j < map->config.y_size; j++)
			if(map->distance[i][j] > max)
				max = map->distance[i][j];

	fp = fopen(filename, "w");
	fprintf(fp, "P6\n");
	fprintf(fp, "%d %d\n", map->config.x_size, map->config.y_size);
	fprintf(fp, "255\n");
	for(i = map->config.y_size - 1; i >= 0; i--)
		for(j = 0; j < map->config.x_size; j++) {
			fprintf(fp, "%c", 255 - (int)(map->distance[j][i] / max * 255.0));
			fprintf(fp, "%c", 255 - (int)(map->distance[j][i] / max * 255.0));
			fprintf(fp, "%c", 255 - (int)(map->distance[j][i] / max * 255.0));
		}

	fclose(fp);
}

/* Writes a likelihood map out to a ppm file */

void carmen_localize_ackerman_write_likelihood_map_to_ppm(char *filename, 
		carmen_localize_ackerman_map_p map,
		double **prob)
{
	FILE *fp;
	int i, j;
	double p, max;

	max = -1e6;
	for(i = 0; i < map->config.x_size; i++)
		for(j = 0; j < map->config.y_size; j++)
			if(prob[i][j] > max)
				max = prob[i][j];
	max = exp(max);

	fp = fopen(filename, "w");
	fprintf(fp, "P6\n");
	fprintf(fp, "%d %d\n", map->config.x_size, map->config.y_size);
	fprintf(fp, "255\n");
	for(i = map->config.y_size - 1; i >= 0; i--)
		for(j = 0; j < map->config.x_size; j++) {
			p = exp(prob[j][i]);
			fprintf(fp, "%c", 255 - (int)(p / max * 255.0));
			fprintf(fp, "%c", 255 - (int)(p / max * 255.0));
			fprintf(fp, "%c", 255 - (int)(p / max * 255.0));
		}
	fclose(fp);
}
