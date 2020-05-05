#include <carmen/offroad_planner.h>
#include <vector>

#define ANALISYS

#define DISTANCE_MAP_HUGE_DISTANCE (1050 * M_SQRT2) // tem que ser maior que o tamanho da diagonal do mapa
#define dist_pvi(x1, y1, x2, y2) (sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)))
#define d1 (double) 1.0
#define d2 (double) M_SQRT2


void
initialize_distance_measurements(int x_size, int y_size, int goal_x, int goal_y, double *utility, double **distance)
{
	int total_size = x_size * y_size;
	std::fill_n(utility, total_size, DISTANCE_MAP_HUGE_DISTANCE);

	distance[goal_x][goal_y] = 0.0; // distance aponta para utility (ver alloc_maps_old(), logo, eh inicializado acima em suas outras celulas)
}


inline int
compute_intermediate_pixel_distance(int x, int y, double **distance)
{
	int distance_changed = 0;

	for (int i = -1; i < 2; i++)
	{
		for (int j = -1; j < 2; j++)
		{
			double v = distance[x + i][y + j] + ((i * j != 0) ? M_SQRT2 : 1.0);
			if (v < distance[x][y])
			{
				distance[x][y] = v;

				distance_changed = 1;
			}
		}
	}

	return (distance_changed);
}


void
alloc_maps(double *utility_map, double *cost_map, int x_size, int y_size, double **&distance, double **&cost)
{
	cost = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(cost);
	for (int i = 0; i < x_size; i++)
		cost[i] = cost_map + i * y_size;

	distance = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(distance);
	for (int i = 0; i < x_size; i++)
		distance[i] = utility_map + i * y_size;
}


void
free_maps(double **distance, double **cost)
{
	free(cost);
	free(distance);
}


int
update_distance_map(double *utility_map, double *cost_map, int x_size, int y_size, int goal_x, int goal_y)
{	// Implementacao de Jarvis https://www.researchgate.net/publication/274622763_The_Exact_Euclidean_Distance_Transform_A_New_Algorithm_for_Universal_Path_Planning
	double **cost, **distance;

	alloc_maps(utility_map, cost_map, x_size, y_size, distance, cost);

	initialize_distance_measurements(x_size, y_size, goal_x, goal_y, utility_map, distance);

	// Use dynamic programming to estimate the minimum distance from every map cell to the goal
	double minimum_occupied_prob = 0.5;
	int iterations = 0;
	int distance_changed;
	do
	{
		distance_changed = 0;
		// pass 1
		for (int x = 1; x < x_size - 1; x++)
			for (int y = 1; y < y_size - 1; y++)
				if (cost[x][y] < minimum_occupied_prob)
					distance_changed += compute_intermediate_pixel_distance(x, y, distance);

		// pass 2
		for (int x = x_size - 2; x >= 1; x--)
			for (int y = y_size - 2; y >= 1; y--)
				if (cost[x][y] < minimum_occupied_prob)
					distance_changed += compute_intermediate_pixel_distance(x, y, distance);
		iterations++;

	} while (distance_changed != 0);

	free_maps(distance, cost);

	return (iterations);
}


void
free_maps_new(double **distance, double **cost,
		short int *p_x_complete, short int *p_y_complete, short int **p_x, short int **p_y)
{
	free(cost);
	free(distance);
	free(p_x_complete);
	free(p_x);
	free(p_y_complete);
	free(p_y);
}


void
alloc_maps_new(double *utility_map, double *cost_map, int x_size, int y_size, double **&distance, double **&cost, short int *&p_x_complete,
		short int *&p_y_complete, short int **&p_x, short int **&p_y)
{
	cost = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(cost);
	for (int i = 0; i < x_size; i++)
		cost[i] = cost_map + i * y_size;

	distance = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(distance);
	for (int i = 0; i < x_size; i++)
		distance[i] = utility_map + i * y_size;

	p_x_complete = (short int *) (malloc(x_size * y_size * sizeof(short int)));
	carmen_test_alloc(p_x_complete);
	p_x = (short int **) (malloc(x_size * sizeof(short int*)));
	carmen_test_alloc(p_x);
	for (int i = 0; i < x_size; i++)
		p_x[i] = p_x_complete + i * y_size;

	p_y_complete = (short int *) (malloc(x_size * y_size * sizeof(short int)));
	carmen_test_alloc(p_y_complete);
	p_y = (short int **) (malloc(x_size * sizeof(short int*)));
	carmen_test_alloc(p_y);
	for (int i = 0; i < x_size; i++)
		p_y[i] = p_y_complete + i * y_size;
}


typedef struct
{
	int x;
	int y;
	double dist;
} bp2_t;


bool
BHM(double **cost_map, int x0, int y0, int x1, int y1, int INVERSE, int SIGN)
{
	int twoDY = 2 * (y1 - y0), twoDX = 2 * (x1 - x0), DX = x1 - x0;
	int incY = ((twoDY < 0) ? -1 : 1);
	twoDY = abs(twoDY);
	int p = twoDY - DX;
	int x = x0, y = y0;
	int *_x, *_y;
	if (INVERSE == 1)
	{
		if (SIGN == 1)
		{
			if (cost_map[-y][-x] > 0.5)
				return true;				//
		}
		else
		{
			_x = &y;
			_y = &x;
			if (cost_map[*_x][*_y] > 0.5)
				return true;
		}
	}
	else
	{
		_x = &x;
		_y = &y;
		if (cost_map[*_x][*_y] > 0.5)
			return true;
	}

	while (x < x1)
	{
		++x;
		if (!(p < 0))
		{
			y += incY;
			p -= twoDX;
		}
		p += twoDY;
		if (INVERSE == 1 && SIGN == 1)
		{
			if (cost_map[-y][-x] > 0.5)
				return true;				//
		}
		else
		{
			if (cost_map[*_x][*_y] > 0.5)
				return true;
		}
	}
	return false;
}


bool
obtacleInLine(double **cost_map, int x0, int y0, int x1, int y1)
{
	int INVERSE = 0;
	int SIGN = 0;
	double m = 0;
	int t;
	int infinity = (x1 == x0);
	if (!infinity)
		m = (y1 - y0) / (double) (x1 - x0);
	if (m < 0)
		SIGN = 1;
	if (infinity || fabs(m) > 1)
	{
		INVERSE = 1;
		if (SIGN)
		{
			t = -x0;
			x0 = -y0;
			y0 = t;
			t = -x1;
			x1 = -y1;
			y1 = t;
		}
		else
		{
			t = x0;
			x0 = y0;
			y0 = t;
			t = x1;
			x1 = y1;
			y1 = t;
		}
	}
	if (x1 < x0)
		return BHM(cost_map, x1, y1, x0, y0, INVERSE, SIGN);
	else
		return BHM(cost_map, x0, y0, x1, y1, INVERSE, SIGN);
}


bool
neighbor(double **cost_map, bp2_t **p2, int x0, int y0, int x1, int y1, bool first)
{
	if (x1 == x0)
	{
		if (p2[x1][y1 - 1].x == x0 && p2[x1][y1 - 1].y == y0)
			return true;
		if (p2[x1][y1 + 1].x == x0 && p2[x1][y1 + 1].y == y0)
			return true;
		else
			return false;
	}

	if (y1 == y0)
	{
		if (p2[x1 - 1][y1].x == x0 && p2[x1 - 1][y1].y == y0)
			return true;
		if (p2[x1 + 1][y1].x == x0 && p2[x1 + 1][y1].y == y0)
			return true;
		else
			return false;
	}

	if (first)
	{
		if (abs(x1 - x0) == abs(y1 - y0))
		{
			if (x1 > x0 && y1 > y0)
			{
				if (p2[x1 - 1][y1 - 1].x == x0 && p2[x1 - 1][y1 - 1].y == y0)
					return true;
				else
					return false;
			}
			if (x1 < x0 && y1 > y0)
			{
				if (p2[x1 + 1][y1 - 1].x == x0 && p2[x1 + 1][y1 - 1].y == y0)
					return true;
				else
					return false;
			}
		}

		if (x1 > x0 && y1 > y0)
		{
			if ((x1 - x0) < (y1 - y0))
			{
				if ((p2[x1 - 1][y1 - 1].x == x0 && p2[x1 - 1][y1 - 1].y == y0) && (p2[x1][y1 - 1].x == x0 && p2[x1][y1 - 1].y == y0))
					return true;
				else
					return !obtacleInLine(cost_map, x0, y0, x1, y1);			//false;
			}
		}

		if (x1 < x0 && y1 > y0)
		{
			if ((x0 - x1) < (y1 - y0))
			{
				if ((p2[x1 + 1][y1 - 1].x == x0 && p2[x1 + 1][y1 - 1].y == y0) && (p2[x1][y1 - 1].x == x0 && p2[x1][y1 - 1].y == y0))
					return true;
				else
					return !obtacleInLine(cost_map, x0, y0, x1, y1);			//false;
			}
		}
	}
	else if (!first)
	{
		if (abs(x1 - x0) == abs(y1 - y0))
		{
			if (x1 < x0 && y1 < y0)
			{
				if (p2[x1 + 1][y1 + 1].x == x0 && p2[x1 + 1][y1 + 1].y == y0)
					return true;
				else
					return false;
			}
			if (x1 > x0 && y1 < y0)
			{
				if (p2[x1 - 1][y1 + 1].x == x0 && p2[x1 - 1][y1 + 1].y == y0)
					return true;
				else
					return false;
			}
		}
		if (x1 < x0 && y1 < y0)
		{
			if ((x0 - x1) < (y0 - y1))
			{
				if ((p2[x1 + 1][y1 + 1].x == x0 && p2[x1 + 1][y1 + 1].y == y0) && (p2[x1][y1 + 1].x == x0 && p2[x1][y1 + 1].y == y0))
					return true;
				else
					return !obtacleInLine(cost_map, x0, y0, x1, y1);			//false;
			}
		}
		if (x1 > x0 && y1 < y0)
		{
			if ((x1 - x0) < (y0 - y1))
			{
				if ((p2[x1 - 1][y1 + 1].x == x0 && p2[x1 - 1][y1 + 1].y == y0) && (p2[x1][y1 + 1].x == x0 && p2[x1][y1 + 1].y == y0))
					return true;
				else
					return !obtacleInLine(cost_map, x0, y0, x1, y1);			//false;
			}
		}
	}

	///Verifica (0-45 grados)
	if (x1 < x0 && y1 > y0)
	{
		if (x0 - x1 > (y1 - y0))
		{
			if ((p2[x1 + 1][y1 - 1].x == x0 && p2[x1 + 1][y1 - 1].y == y0) && (p2[x1 + 1][y1].x == x0 && p2[x1 + 1][y1].y == y0))
				return true;
			else
				return !obtacleInLine(cost_map, x0, y0, x1, y1);			//false;
		}
	}

	///Verifica (315-360 grados)
	if (x1 < x0 && y1 < y0)
	{
		if ((x0 - x1) > (y0 - y1))
		{
			if ((p2[x1 + 1][y1 + 1].x == x0 && p2[x1 + 1][y1 + 1].y == y0) && (p2[x1 + 1][y1].x == x0 && p2[x1 + 1][y1].y == y0))
				return true;
			else
				return !obtacleInLine(cost_map, x0, y0, x1, y1);			//false;
		}
	}

	///Verifica 135-180
	if (x1 > x0 && y1 > y0)
	{
		if ((x1 - x0) > (y1 - y0))
		{
			if ((p2[x1 - 1][y1 - 1].x == x0 && p2[x1 - 1][y1 - 1].y == y0) && (p2[x1 - 1][y1].x == x0 && p2[x1 - 1][y1].y == y0))
				return true;
			else
				return !obtacleInLine(cost_map, x0, y0, x1, y1);				//false;
		}
	}
	///Verifica (180-225 grados)
	if (x1 > x0 && y1 < y0)
	{
		if ((x1 - x0) > (y0 - y1))
		{
			if ((p2[x1 - 1][y1 + 1].x == x0 && p2[x1 - 1][y1 + 1].y == y0) && (p2[x1 - 1][y1].x == x0 && p2[x1 - 1][y1].y == y0))
				return true;
			else
				return !obtacleInLine(cost_map, x0, y0, x1, y1);				//false;
		}
	}
	return false;
}


int
update_distance_map_new(double *distance_map_complete, double *cost_map_complete, int x_size, int y_size, int goal_x, int goal_y)
{
	double *dist_complete;
	double **dist;

	bp2_t *p2_complete;
	bp2_t **p2;

	double **distance_map;
	double **cost_map;

	dist_complete = (double *) (malloc(x_size * y_size * sizeof(double)));
	carmen_test_alloc(dist_complete);
	dist = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(dist);
	for (int i = 0; i < x_size; i++)
		dist[i] = dist_complete + i * x_size;

	p2_complete = (bp2_t *) (malloc(x_size * y_size * sizeof(bp2_t)));
	carmen_test_alloc(p2_complete);
	p2 = (bp2_t **) (malloc(x_size * sizeof(bp2_t *)));
	carmen_test_alloc(p2);
	for (int i = 0; i < x_size; i++)
		p2[i] = p2_complete + i * x_size;

	distance_map = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(distance_map);
	for (int i = 0; i < x_size; i++)
		distance_map[i] = distance_map_complete + i * x_size;

	cost_map = (double **) (malloc(x_size * sizeof(double *)));
	carmen_test_alloc(cost_map);
	for (int i = 0; i < x_size; i++)
		cost_map[i] = cost_map_complete + i * x_size;

	// initialize map
	for (int y = 0; y < y_size; y++)
		for (int x = 0; x < x_size; x++)
		{
			distance_map[x][y] = DISTANCE_MAP_HUGE_DISTANCE;
			p2[x][y].x = -1;
			p2[x][y].y = -1;
			p2[x][y].dist = DISTANCE_MAP_HUGE_DISTANCE;
			dist[x][y] = sqrt(x * x + y * y);
		}

	p2[goal_x][goal_y].x = goal_x;
	p2[goal_x][goal_y].y = goal_y;
	p2[goal_x][goal_y].dist = 0;
	distance_map[goal_x][goal_y] = 0;

	int count = 1;
	int ciclos = 0;
	while (count != 0)
	{
		count = 0;
		//perform the first pass
		for (int y = 1; y < y_size - 1; y++)
		{
			for (int x = 1; x < x_size - 1; x++)
			{
				if (cost_map[x][y] < 0.5)
				{
					if (p2[x - 1][y - 1].x > -1)
						if (dist[abs(x - p2[x - 1][y - 1].x)][abs(y - p2[x - 1][y - 1].y)] + p2[x - 1][y - 1].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x - 1][y - 1].x, p2[x - 1][y - 1].y, x, y, true))
							{
								p2[x][y] = p2[x - 1][y - 1];
								distance_map[x][y] = dist[abs(x - p2[x - 1][y - 1].x)][abs(y - p2[x - 1][y - 1].y)] + p2[x - 1][y - 1].dist;
								count++;
							}
							else if (distance_map[x - 1][y - 1] + d2 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x - 1][y - 1] + d2;
								p2[x][y].x = x - 1;
								p2[x][y].y = y - 1;
								p2[x][y].dist = distance_map[x - 1][y - 1];
								count++;
							}
						}

					if (p2[x][y - 1].x > -1)
						if (dist[abs(x - p2[x][y - 1].x)][abs(y - p2[x][y - 1].y)] + p2[x][y - 1].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x][y - 1].x, p2[x][y - 1].y, x, y, true))
							{
								p2[x][y] = p2[x][y - 1];
								distance_map[x][y] = dist[abs(x - p2[x][y - 1].x)][abs(y - p2[x][y - 1].y)] + p2[x][y - 1].dist;
								count++;
							}
							else if (distance_map[x][y - 1] + d1 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x][y - 1] + d1;
								p2[x][y].x = x;
								p2[x][y].y = y - 1;
								p2[x][y].dist = distance_map[x][y - 1];
								count++;
							}
						}
					if (p2[x + 1][y - 1].x > -1)
						if (dist[abs(x - p2[x + 1][y - 1].x)][abs(y - p2[x + 1][y - 1].y)] + p2[x + 1][y - 1].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x + 1][y - 1].x, p2[x + 1][y - 1].y, x, y, true))
							{
								p2[x][y] = p2[x + 1][y - 1];
								distance_map[x][y] = dist[abs(x - p2[x + 1][y - 1].x)][abs(y - p2[x + 1][y - 1].y)] + p2[x + 1][y - 1].dist;
								count++;
							}
							else if (distance_map[x + 1][y - 1] + d2 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x + 1][y - 1] + d2;
								p2[x][y].x = x + 1;
								p2[x][y].y = y - 1;
								p2[x][y].dist = distance_map[x + 1][y - 1];
								count++;
							}
						}
					if (p2[x - 1][y].x > -1)
						if (dist[abs(x - p2[x - 1][y].x)][abs(y - p2[x - 1][y].y)] + p2[x - 1][y].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x - 1][y].x, p2[x - 1][y].y, x, y, true))
							{
								p2[x][y] = p2[x - 1][y];
								distance_map[x][y] = dist[abs(x - p2[x - 1][y].x)][abs(y - p2[x - 1][y].y)] + p2[x - 1][y].dist;
								count++;
							}
							else if (distance_map[x - 1][y] + d1 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x - 1][y] + d1;
								p2[x][y].x = x - 1;
								p2[x][y].y = y;
								p2[x][y].dist = distance_map[x - 1][y];
								count++;
							}
						}

				}
			}
			for (int x = x_size - 2; x > 0; x--)
			{
				if (cost_map[x][y] < 0.5)
				{
					if (p2[x + 1][y].x > -1)
						if (dist[abs(x - p2[x + 1][y].x)][abs(y - p2[x + 1][y].y)] + p2[x + 1][y].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x + 1][y].x, p2[x + 1][y].y, x, y, true))
							{
								p2[x][y] = p2[x + 1][y];
								distance_map[x][y] = dist[abs(x - p2[x + 1][y].x)][abs(y - p2[x + 1][y].y)] + p2[x + 1][y].dist;
								count++;
							}
							else if (distance_map[x + 1][y] + d1 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x + 1][y] + d1;
								p2[x][y].x = x + 1;
								p2[x][y].y = y;
								p2[x][y].dist = distance_map[x + 1][y];
								count++;
							}
						}
				}
			}
		}

		//perform the final pass
		for (int y = y_size - 2; y > 0; y--)
		{
			for (int x = x_size - 2; x > 0; x--)
			{
				if (cost_map[x][y] < 0.5)
				{
					if (p2[x + 1][y].x > -1)
						if (dist[abs(x - p2[x + 1][y].x)][abs(y - p2[x + 1][y].y)] + p2[x + 1][y].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x + 1][y].x, p2[x + 1][y].y, x, y, false))
							{
								p2[x][y] = p2[x + 1][y];
								distance_map[x][y] = dist[abs(x - p2[x + 1][y].x)][abs(y - p2[x + 1][y].y)] + p2[x + 1][y].dist;//sqrt( (x-p2[x][y].x)*(x-p2[x][y].x) + (y-p2[x][y].y)*(y-p2[x][y].y) ) +p2[x][y].dist;
								count++;
							}
							else if (distance_map[x + 1][y] + d1 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x + 1][y] + d1;
								p2[x][y].x = x + 1;
								p2[x][y].y = y;
								p2[x][y].dist = distance_map[x + 1][y];
								count++;
							}
						}
					if (p2[x - 1][y + 1].x > -1)
						if (dist[abs(x - p2[x - 1][y + 1].x)][abs(y - p2[x - 1][y + 1].y)] + p2[x - 1][y + 1].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x - 1][y + 1].x, p2[x - 1][y + 1].y, x, y, false))
							{
								p2[x][y] = p2[x - 1][y + 1];
								distance_map[x][y] = dist[abs(x - p2[x - 1][y + 1].x)][abs(y - p2[x - 1][y + 1].y)] + p2[x - 1][y + 1].dist;
								count++;
							}
							else if (distance_map[x - 1][y + 1] + d2 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x - 1][y + 1] + d2;
								p2[x][y].x = x - 1;
								p2[x][y].y = y + 1;
								p2[x][y].dist = distance_map[x - 1][y + 1];
								count++;
							}
						}

					if (p2[x][y + 1].x > -1)
						if (dist[abs(x - p2[x][y + 1].x)][abs(y - p2[x][y + 1].y)] + p2[x][y + 1].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x][y + 1].x, p2[x][y + 1].y, x, y, false))
							{
								p2[x][y] = p2[x][y + 1];
								distance_map[x][y] = dist[abs(x - p2[x][y + 1].x)][abs(y - p2[x][y + 1].y)] + p2[x][y + 1].dist;
								count++;
							}
							else if (distance_map[x][y + 1] + d1 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x][y + 1] + d1;
								p2[x][y].x = x;
								p2[x][y].y = y + 1;
								p2[x][y].dist = distance_map[x][y + 1];
								count++;
							}
						}
					if (p2[x + 1][y + 1].x > -1)
						if (dist[abs(x - p2[x + 1][y + 1].x)][abs(y - p2[x + 1][y + 1].y)] + p2[x + 1][y + 1].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x + 1][y + 1].x, p2[x + 1][y + 1].y, x, y, false))
							{
								p2[x][y] = p2[x + 1][y + 1];
								distance_map[x][y] = dist[abs(x - p2[x + 1][y + 1].x)][abs(y - p2[x + 1][y + 1].y)] + p2[x + 1][y + 1].dist;
								count++;
							}
							else if (distance_map[x + 1][y + 1] + d2 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x + 1][y + 1] + d2;
								p2[x][y].x = x + 1;
								p2[x][y].y = y + 1;
								p2[x][y].dist = distance_map[x + 1][y + 1];
								count++;
							}
						}
				}
			}
			for (int x = 1; x < x_size - 1; x++)
			{
				if (cost_map[x][y] < 0.5)
				{
					if (p2[x - 1][y].x > -1)
						if (dist[abs(x - p2[x - 1][y].x)][abs(y - p2[x - 1][y].y)] + p2[x - 1][y].dist < distance_map[x][y])
						{
							if (neighbor(cost_map, p2, p2[x - 1][y].x, p2[x - 1][y].y, x, y, false))
							{
								p2[x][y] = p2[x - 1][y];
								distance_map[x][y] = dist[abs(x - p2[x - 1][y].x)][abs(y - p2[x - 1][y].y)] + p2[x - 1][y].dist;
								count++;
							}
							else if (distance_map[x - 1][y] + d1 < distance_map[x][y])
							{
								distance_map[x][y] = distance_map[x - 1][y] + d1;
								p2[x][y].x = x - 1;
								p2[x][y].y = y;
								p2[x][y].dist = distance_map[x - 1][y];
								count++;
							}
						}

				}

			}
		}
		ciclos++;

	}

	free(dist_complete);
	free(dist);

	free(p2_complete);
	free(p2);

	free(distance_map);
	free(cost_map);

	return (ciclos);
}
