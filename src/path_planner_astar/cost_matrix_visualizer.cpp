#include "path_planner_astar.h"


#define HEURISTIC_THETA_SIZE 72
#define HEURISTIC_MAP_SIZE 101
#define HEURISTIC_GRID_RESOLUTION 0.2
#define FILE_NAME "cost_matrix_02_101x101x72.data"

cost_heuristic_node_p ***cost_map;


static int to_ppm(char *output_filename, int theta)
{
	int x, y;
	char c;
	FILE *fp;

	fp = fopen(output_filename, "w");
	if (fp == NULL)
	{
		printf("fopen falhou\n");
		return -1;
	}

	fprintf(fp, "P6\n%d %d\n255\n", int(HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION), int(HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION));
	for(x = 0; x < HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION; x++)
	{
		for(y = 0; y < HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION; y++)
		{
//			printf("x = %d, y= %d cost = %lf\n", x, y, cost_map[x][y][theta]->h);
			if(cost_map[x][y][theta]->h == -1 || cost_map[x][y][theta]->h >= 255)
			{
				fputc(255, fp); fputc(255, fp); fputc(255, fp);
			}
			else
			{
//				c = 255 - (cost_map[x][y][theta]->h) * 100;
				c =  cost_map[x][y][theta]->h;
				fputc(c, fp); fputc(c, fp); fputc(c, fp);
			}
		}
	}
	fclose(fp);

	return 0;
}


void
alloc_cost_map()
{
	int i, j, z;
	int x_size = round(HEURISTIC_MAP_SIZE  / HEURISTIC_GRID_RESOLUTION);
	int y_size = round(HEURISTIC_MAP_SIZE / HEURISTIC_GRID_RESOLUTION);

	cost_map = (cost_heuristic_node_p ***)calloc(x_size, sizeof(cost_heuristic_node_p**));
	carmen_test_alloc(cost_map);

	for (i = 0; i < x_size; i++)
	{
		cost_map[i] = (cost_heuristic_node_p **)calloc(y_size, sizeof(cost_heuristic_node_p*));
		carmen_test_alloc(cost_map[i]);

		for (j = 0; j < y_size; j++)
		{
			cost_map[i][j] = (cost_heuristic_node_p*)calloc(HEURISTIC_THETA_SIZE, sizeof(cost_heuristic_node_p));
			carmen_test_alloc(cost_map[i][j]);

			for (z = 0; z < HEURISTIC_THETA_SIZE; z++)
			{
				cost_map[i][j][z]= (cost_heuristic_node_p) malloc(sizeof(cost_heuristic_node));
				carmen_test_alloc(cost_map[i][j][z]);


			}
		}
	}
}

static int
open_cost_map()
{
	FILE *fp;
	int i, j, k, result;

	fp = fopen (FILE_NAME, "rw");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}

	for (i = 0; i < (HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION); i++)
	{
		for (j = 0; j < (HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION); j++)
		{
			for (k = 0; k < HEURISTIC_THETA_SIZE; k++)
			{
				result = fscanf(fp, "%lf ", &cost_map[i][j][k]->h);
//				printf("aqui %d %d %d\n", i, j, k);
				if (result == EOF)
				{
					printf("acho que acabou\n");
					break;
				}
			}
		}
	}
	fclose (fp);
	return 0;
}


void
clear_cost_map()
{
	int i, j, z;
	int x_size = round(HEURISTIC_MAP_SIZE  / HEURISTIC_GRID_RESOLUTION);
	int y_size = round(HEURISTIC_MAP_SIZE / HEURISTIC_GRID_RESOLUTION);

	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (z = 0; z < HEURISTIC_THETA_SIZE; z++)
				cost_map[i][j][z] = NULL;
}

int
main(int argc, char **argv)
{

	alloc_cost_map();
	open_cost_map();
	char szBuffer[1024];
	char file_name[1024];

	for (int k = 0; k < HEURISTIC_THETA_SIZE; k++)
	{
		memset (&file_name, 0, sizeof (file_name) );
		snprintf (szBuffer, sizeof (szBuffer), "temp_maps/trim_%d.ppm", k);
		//strcat(file_name, FILE_NAME);
		strcat(file_name, szBuffer);
		to_ppm(file_name, k);

}	clear_cost_map();

	return 0;
}


